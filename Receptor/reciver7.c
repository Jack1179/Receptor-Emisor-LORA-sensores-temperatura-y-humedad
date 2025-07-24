#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>

// Registros LoRa (SX1276) - consistentes con el transmisor
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_IRQ_FLAGS            0x12
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_VERSION              0x42

// Modos de operación
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_RX_CONTINUOUS       0x05

// Máscaras de IRQ
#define IRQ_RX_DONE_MASK         0x40
#define IRQ_CRC_ERR_MASK         0x20

int spi_fd;
const char *spi_device = "/dev/spidev2.0";

// Configuración SPI
uint8_t spi_mode = SPI_MODE_0;
uint8_t spi_bits = 8;
uint32_t spi_speed = 1000000;  // 10 MHz

// Declaraciones de funciones
uint8_t spi_transfer(uint8_t data);
uint8_t lora_read_reg(uint8_t reg);
void lora_write_reg(uint8_t reg, uint8_t value);
void lora_init(long frequency);
void setup_lora_receiver();
int receive_packet(uint8_t *buffer, int *length, int *rssi, float *snr);

// Transferencia SPI
uint8_t spi_transfer(uint8_t data) {
    uint8_t rx_buf;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)&data,
        .rx_buf = (unsigned long)&rx_buf,
        .len = 1,
        .delay_usecs = 0,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits,
    };
    
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("Error SPI");
        return 0;
    }
    return rx_buf;
}

// Leer registro
uint8_t lora_read_reg(uint8_t reg) {
    uint8_t tx[2] = {reg & 0x7F, 0x00};  // MSB=0 para lectura
    uint8_t rx[2];
    
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .delay_usecs = 0,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits,
    };
    
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    return rx[1];
}

// Escribir registro
void lora_write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg | 0x80, value};  // MSB=1 para escritura
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = 0,
        .len = 2,
        .delay_usecs = 0,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits,
    };
    
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

// Inicialización LoRa sin reset físico
void lora_init(long frequency) {
    // Abrir dispositivo SPI
    spi_fd = open(spi_device, O_RDWR);
    if (spi_fd < 0) {
        perror("Error abriendo SPI");
        exit(1);
    }
    
    // Configurar SPI
    ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    
    // Reset por software: modo sleep
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    usleep(10000);
    
    // Verificar versión del chip
    uint8_t version = lora_read_reg(REG_VERSION);
    printf("Versión del chip: 0x%02X\n", version);
    if (version != 0x12) {
        fprintf(stderr, "Error: Chip no reconocido\n");
        exit(1);
    }
    
    // Configurar frecuencia (433 MHz)
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    lora_write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF);
    lora_write_reg(REG_FRF_MID, (frf >> 8) & 0xFF);
    lora_write_reg(REG_FRF_LSB, frf & 0xFF);
    
    // Modo standby
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    usleep(50000);
}

// Configurar modo receptor
void setup_lora_receiver() {
    // Configuración del modem para receptor
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);  // BW=125kHz, CR=4/5, Header implícito
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);  // SF=7, CRC habilitado
    
    // Configuración adicional para recepción
    lora_write_reg(REG_PAYLOAD_LENGTH, 255);    // Longitud máxima del payload
    
    // Modo recepción continua
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

// Recibir un paquete
int receive_packet(uint8_t *buffer, int *length, int *rssi, float *snr) {
    // Verificar flags de interrupción
    uint8_t irq_flags = lora_read_reg(REG_IRQ_FLAGS);
    
    // Si se recibió un paquete sin errores CRC
    if ((irq_flags & IRQ_RX_DONE_MASK) && !(irq_flags & IRQ_CRC_ERR_MASK)) {
        // Limpiar flags
        lora_write_reg(REG_IRQ_FLAGS, irq_flags);
        
        // Obtener longitud del paquete
        *length = lora_read_reg(REG_RX_NB_BYTES);
        
        // Obtener dirección actual del FIFO
        uint8_t current_addr = lora_read_reg(REG_FIFO_RX_CURRENT_ADDR);
        lora_write_reg(REG_FIFO_ADDR_PTR, current_addr);
        
        // Leer datos del FIFO
        for (int i = 0; i < *length; i++) {
            buffer[i] = lora_read_reg(REG_FIFO);
        }
        
        // Calcular RSSI (ajuste para 433 MHz)
        *rssi = lora_read_reg(REG_PKT_RSSI_VALUE) - 157;
        
        // Calcular SNR
        int8_t raw_snr = (int8_t)lora_read_reg(REG_PKT_SNR_VALUE);
        *snr = raw_snr * 0.25;
        
        return 1;  // Paquete recibido correctamente
    }
    
    // Limpiar flag de error CRC
    if (irq_flags & IRQ_CRC_ERR_MASK) {
        lora_write_reg(REG_IRQ_FLAGS, IRQ_CRC_ERR_MASK);
    }
    
    return 0;  // No hay paquete válido
}

int main() {
    // Inicializar LoRa a 433 MHz
    lora_init(433000000);
    
    // Configurar como receptor
    setup_lora_receiver();
    
    printf("Esperando paquetes LoRa...\n");
    
    uint8_t rx_buffer[255];
    int length;
    int rssi;
    float snr;
    
    // Bucle principal de recepción
    while (1) {
        if (receive_packet(rx_buffer, &length, &rssi, &snr)) {
            printf("\nPaquete recibido (%d bytes):\n", length);
            printf("Contenido: ");
            for (int i = 0; i < length; i++) {
                printf("%c", rx_buffer[i]);
            }
            printf("\nRSSI: %d dBm, SNR: %.2f dB\n", rssi, snr);
        }
        usleep(10);  // Pequeña pausa para reducir carga de CPU
    }
    
    close(spi_fd);
    return 0;
}
