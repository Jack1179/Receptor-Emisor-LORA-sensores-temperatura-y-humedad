#include "lora_v2.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

// Configuración SPI
#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_SPEED  8000000  // 8 MHz

// Función auxiliar para control GPIO simple
static void gpio_set_value(int gpio, int value) {
    char path[50];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(path, O_WRONLY);
    if (fd >= 0) {
        char val = value ? '1' : '0';
        write(fd, &val, 1);
        close(fd);
    }
}

// Lectura de registro SPI
static uint8_t lora_read_register(LoRaContext* ctx, uint8_t reg) {
    uint8_t tx[2] = { reg & 0x7F, 0 };
    uint8_t rx[2] = {0};
    
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .delay_usecs = 0,
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };
    
    ioctl(ctx->spi_fd, SPI_IOC_MESSAGE(1), &tr);
    return rx[1];
}

// Escritura de registro SPI
static void lora_write_register(LoRaContext* ctx, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg | 0x80, value };
    
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = 0,
        .len = 2,
        .delay_usecs = 0,
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };
    
    ioctl(ctx->spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

// Inicialización del módulo LoRa
int lora_init(LoRaContext* ctx, long frequency) {
    memset(ctx, 0, sizeof(LoRaContext));
    
    // Configurar GPIO de reset
    ctx->reset_gpio = 13; // Ajusta según tu hardware
    
    // Configurar GPIO de reset
    int export_fd = open("/sys/class/gpio/export", O_WRONLY);
    if (export_fd >= 0) {
        char gpio_str[10];
        snprintf(gpio_str, sizeof(gpio_str), "%d", ctx->reset_gpio);
        write(export_fd, gpio_str, strlen(gpio_str));
        close(export_fd);
        
        // Esperar a que se cree el directorio
        usleep(100000);
        
        // Configurar dirección
        char dir_path[100];
        snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", ctx->reset_gpio);
        int dir_fd = open(dir_path, O_WRONLY);
        if (dir_fd >= 0) {
            write(dir_fd, "out", 3);
            close(dir_fd);
        }
    }

    // Resetear hardware
    gpio_set_value(ctx->reset_gpio, 0); // LOW
    usleep(10000); // 10 ms
    gpio_set_value(ctx->reset_gpio, 1); // HIGH
    usleep(10000); // 10 ms
    
    // Abrir dispositivo SPI
    ctx->spi_fd = open(SPI_DEVICE, O_RDWR);
    if (ctx->spi_fd < 0) {
        perror("Error opening SPI device");
        return -1;
    }
    
    // Configurar modo SPI
    uint8_t mode = SPI_MODE_0;
    if (ioctl(ctx->spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Error setting SPI mode");
        close(ctx->spi_fd);
        return -1;
    }
    
    // Configurar velocidad SPI
    uint32_t speed = SPI_SPEED;
    if (ioctl(ctx->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("Error setting SPI speed");
        close(ctx->spi_fd);
        return -1;
    }
    
    // Verificar versión del chip
    uint8_t version = lora_read_register(ctx, REG_VERSION);
    if (version != 0x12) {
        fprintf(stderr, "Invalid chip version: 0x%02X\n", version);
        close(ctx->spi_fd);
        return -1;
    }
    
    // Configuración inicial
    lora_write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    lora_set_frequency(ctx, frequency);
    
    // Configuración básica
    lora_write_register(ctx, REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_register(ctx, REG_FIFO_RX_BASE_ADDR, 0);
    lora_write_register(ctx, REG_LNA, lora_read_register(ctx, REG_LNA) | 0x03); // LNA gain
    lora_write_register(ctx, REG_MODEM_CONFIG_1, 0x72); // BW=125kHz, CR=4/5
    lora_write_register(ctx, REG_MODEM_CONFIG_2, 0x74); // SF=7, CRC on
    lora_write_register(ctx, REG_SYNC_WORD, 0x34);      // Sync word
    
    lora_set_tx_power(ctx, 17); // 17 dBm
    
    // Poner en modo standby
    lora_write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    
    return 0;
}

// Inicio de transmisión
int lora_begin_packet(LoRaContext* ctx, bool implicitHeader) {
    // Verificar si ya está transmitiendo
    if ((lora_read_register(ctx, REG_OP_MODE) & MODE_TX) == MODE_TX) {
        return 0;
    }
    
    // Poner en modo standby
    lora_write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    usleep(1000);
    
    // Configurar modo de cabecera
    ctx->implicit_header = implicitHeader;
    uint8_t config1 = lora_read_register(ctx, REG_MODEM_CONFIG_1);
    if (implicitHeader) {
        lora_write_register(ctx, REG_MODEM_CONFIG_1, config1 | 0x01);
    } else {
        lora_write_register(ctx, REG_MODEM_CONFIG_1, config1 & 0xFE);
    }
    
    // Reiniciar FIFO
    lora_write_register(ctx, REG_FIFO_ADDR_PTR, 0);
    lora_write_register(ctx, REG_PAYLOAD_LENGTH, 0);
    
    return 1;
}

// Finalización de transmisión (siempre síncrona)
int lora_end_packet(LoRaContext* ctx) {
    // Iniciar transmisión
    lora_write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    
    // Esperar a que termine la transmisión
    while ((lora_read_register(ctx, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        usleep(1000);
    }
    
    // Limpiar flag
    lora_write_register(ctx, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    
    return 1;
}

// Escritura de datos
int lora_write(LoRaContext* ctx, const uint8_t* buffer, size_t size) {
    int currentLength = lora_read_register(ctx, REG_PAYLOAD_LENGTH);
    
    // Verificar límite FIFO
    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - currentLength;
    }
    
    // Escribir datos
    for (size_t i = 0; i < size; i++) {
        lora_write_register(ctx, REG_FIFO, buffer[i]);
    }
    
    // Actualizar longitud
    lora_write_register(ctx, REG_PAYLOAD_LENGTH, currentLength + size);
    
    return size;
}

// Configurar recepción
void lora_receive(LoRaContext* ctx) {
    // Iniciar recepción continua
    lora_write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

// Verificar si hay paquetes recibidos
int lora_parse_packet(LoRaContext* ctx) {
    uint8_t irqFlags = lora_read_register(ctx, REG_IRQ_FLAGS);
    
    if (irqFlags & IRQ_RX_DONE_MASK) {
        // Limpiar flag
        lora_write_register(ctx, REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);
        
        // Obtener longitud del paquete
        int packetLength = ctx->implicit_header ? 
            lora_read_register(ctx, REG_PAYLOAD_LENGTH) : 
            lora_read_register(ctx, REG_RX_NB_BYTES);
        
        // Configurar puntero FIFO
        lora_write_register(ctx, REG_FIFO_ADDR_PTR, lora_read_register(ctx, REG_FIFO_RX_CURRENT_ADDR));
        
        ctx->packet_index = 0;
        return packetLength;
    }
    
    return 0;
}

// Leer datos recibidos
int lora_read(LoRaContext* ctx, uint8_t* buffer, size_t size) {
    int available = lora_available(ctx);
    if (available <= 0) {
        return -1;
    }
    
    if (size > (size_t)available) {
        size = available;
    }
    
    for (size_t i = 0; i < size; i++) {
        buffer[i] = lora_read_register(ctx, REG_FIFO);
        ctx->packet_index++;
    }
    
    return size;
}

// Obtener bytes disponibles
int lora_available(LoRaContext* ctx) {
    return lora_read_register(ctx, REG_RX_NB_BYTES) - ctx->packet_index;
}

// Configurar frecuencia
void lora_set_frequency(LoRaContext* ctx, long frequency) {
    ctx->frequency = frequency;
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    
    lora_write_register(ctx, REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_register(ctx, REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_register(ctx, REG_FRF_LSB, (uint8_t)(frf >> 0));
}

// Configurar potencia de transmisión
void lora_set_tx_power(LoRaContext* ctx, int level) {
    if (level > 17) level = 17;
    if (level < 2) level = 2;
    
    lora_write_register(ctx, REG_PA_CONFIG, 0x80 | (level - 2));
}

// Cerrar recursos
void lora_close(LoRaContext* ctx) {
    if (ctx->spi_fd >= 0) close(ctx->spi_fd);
    
    // Unexport GPIO
    int unexport_fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (unexport_fd >= 0) {
        char gpio_str[10];
        snprintf(gpio_str, sizeof(gpio_str), "%d", ctx->reset_gpio);
        write(unexport_fd, gpio_str, strlen(gpio_str));
        close(unexport_fd);
    }
}

// Ejemplo de uso
int main() {
    LoRaContext lora;
    
    if (lora_init(&lora, 868000000) != 0) {
        fprintf(stderr, "LoRa initialization failed\n");
        return 1;
    }
    
    // Iniciar recepción
    lora_receive(&lora);
    
    while (1) {
        // Verificar si hay paquetes recibidos
        int packetSize = lora_parse_packet(&lora);
        if (packetSize > 0) {
            printf("Received packet: %d bytes\n", packetSize);
            
            uint8_t buffer[256];
            int len = lora_read(&lora, buffer, packetSize);
            if (len > 0) {
                printf("Data: ");
                for (int i = 0; i < len; i++) {
                    printf("%02X ", buffer[i]);
                }
                printf("\n");
            }
        }
        
        // Transmitir periódicamente
        static int counter = 0;
        if (counter++ % 100 == 0) {
            uint8_t data[] = {0x48, 0x65, 0x6C, 0x6C, 0x6F}; // "Hello"
            lora_begin_packet(&lora, false);
            lora_write(&lora, data, sizeof(data));
            lora_end_packet(&lora);
            printf("Packet sent\n");
        }
        
        usleep(10000); // Esperar 10 ms
    }
    
    lora_close(&lora);
    return 0;
}
