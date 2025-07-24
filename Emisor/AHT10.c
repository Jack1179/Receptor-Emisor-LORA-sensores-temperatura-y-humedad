#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <time.h>

// Definiciones del sensor (de AHTxx.h)
#define AHTXX_ADDRESS_X38         0x38
#define AHTXX_SOFT_RESET_REG      0xBA
#define AHT1X_INIT_REG            0xE1
#define AHTXX_START_MEASUREMENT_REG 0xAC
#define AHTXX_STATUS_REG          0x71

#define AHTXX_INIT_CTRL_CAL_ON    0x08
#define AHT1X_INIT_CTRL_NORMAL_MODE 0x00
#define AHTXX_INIT_CTRL_NOP       0x00
#define AHTXX_START_MEASUREMENT_CTRL 0x33
#define AHTXX_START_MEASUREMENT_CTRL_NOP 0x00

#define AHTXX_STATUS_CTRL_BUSY    0x80
#define AHTXX_STATUS_CTRL_CAL_ON  0x08

// Delays (ms)
#define AHTXX_CMD_DELAY           10
#define AHTXX_MEASUREMENT_DELAY   80
#define AHT1X_POWER_ON_DELAY      40
#define AHTXX_SOFT_RESET_DELAY    20

// Errores
#define AHTXX_NO_ERROR            0
#define AHTXX_BUSY_ERROR          1
#define AHTXX_ACK_ERROR           2
#define AHTXX_DATA_ERROR          3

#define LORA_PAYLOAD_SIZE 32 // Tamaño máximo para payload LoRa
typedef struct {
    int i2c_fd;
    uint8_t address;
} AHT10_DEV;

// Prototipos de funciones
AHT10_DEV* aht10_init(const char* i2c_device, uint8_t address);
int aht10_soft_reset(AHT10_DEV* dev);
int aht10_start_measurement(AHT10_DEV* dev);
int aht10_read_data(AHT10_DEV* dev, float* temperature, float* humidity);
void aht10_close(AHT10_DEV* dev);
int aht10_initialize(AHT10_DEV* dev);
void format_sensor_data(float temperature, float humidity, uint8_t *buffer, size_t *length);

int main() {

    

    AHT10_DEV* sensor = aht10_init("/dev/i2c-1", AHTXX_ADDRESS_X38);
    if (!sensor) {
        fprintf(stderr, "Error al inicializar el sensor\n");
        return EXIT_FAILURE;
    }

    // Esperar inicialización del sensor
    usleep(AHT1X_POWER_ON_DELAY * 1000);
    
    // Configuración completa del sensor
    if (aht10_initialize(sensor) != AHTXX_NO_ERROR) {
        fprintf(stderr, "Error en configuración inicial\n");
        aht10_close(sensor);
        return EXIT_FAILURE;
    }

 // Buffer para payload LoRa (cada elemento es un uint8_t representando un carácter ASCII)
    uint8_t lora_payload[LORA_PAYLOAD_SIZE];
    size_t payload_length = 0;

    while (1) {
        float temp, hum;
        int status = aht10_read_data(sensor, &temp, &hum);
        
        if (status == AHTXX_NO_ERROR) {
            // Aplicar offset de +2°C a la temperatura
            temp += 2.0;
            
            printf("Temperatura: %.2f C | Humedad: %.2f%%\n", temp, hum);
            
            // Formatear datos para LoRa
            format_sensor_data(temp, hum, lora_payload, &payload_length);
            
            if (payload_length > 0) {
                // Mostrar contenido del vector ASCII
                printf("Payload LoRa (%zu bytes): [", payload_length);
                for (size_t i = 0; i < payload_length; i++) {
                    printf("%c", lora_payload[i]); // Mostrar como caracter
                }
                printf("]\n");
                
                // Mostrar valores hexadecimales
                printf("Hex: ");
                for (size_t i = 0; i < payload_length; i++) {
                    printf("%02X ", lora_payload[i]); // Mostrar hex
                }
                printf("\n");
                
                // Aquí iría el envío real por LoRa:
                // send_lora_data(lora_payload, payload_length);
            }
        } else {
            fprintf(stderr, "Error en lectura: %d\n", status);
        }
        
        sleep(15);
    }

    aht10_close(sensor);
    //return EXIT_SUCCESS;
}


AHT10_DEV* aht10_init(const char* i2c_device, uint8_t address) {
    int fd = open(i2c_device, O_RDWR);
    if (fd < 0) {
        perror("Error al abrir dispositivo I2C");
        return NULL;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        perror("Error en ioctl");
        close(fd);
        return NULL;
    }

    AHT10_DEV* dev = malloc(sizeof(AHT10_DEV));
    dev->i2c_fd = fd;
    dev->address = address;
    return dev;
}

// Función mejorada con configuración completa
int aht10_initialize(AHT10_DEV* dev) {
    // 1. Soft Reset
    if (aht10_soft_reset(dev) != AHTXX_NO_ERROR) {
        return AHTXX_ACK_ERROR;
    }
    
    // 2. Inicialización del sensor
    uint8_t init_cmd[3] = {
        AHT1X_INIT_REG,  // Registro de inicialización
        AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_NORMAL_MODE, // Calibración + modo normal
        AHTXX_INIT_CTRL_NOP  // Byte adicional
    };
    
    if (write(dev->i2c_fd, init_cmd, sizeof(init_cmd)) != sizeof(init_cmd)) {
        perror("Error en inicialización");
        return AHTXX_ACK_ERROR;
    }
    
    // 3. Verificar calibración
    usleep(15000);  // Esperar 10ms para que se complete la calibración
    
    uint8_t status;
    if (read(dev->i2c_fd, &status, 1) != 1) {
        perror("Error leyendo estado");
        return AHTXX_DATA_ERROR;
    }
    
    if (!(status & AHTXX_STATUS_CTRL_CAL_ON)) {
        fprintf(stderr, "Error: Calibración no completada\n");
        return AHTXX_DATA_ERROR;
    }
    
    return AHTXX_NO_ERROR;
}

int aht10_soft_reset(AHT10_DEV* dev) {
    uint8_t cmd = AHTXX_SOFT_RESET_REG;
    if (write(dev->i2c_fd, &cmd, 1) != 1) {
        perror("Error en reset suave");
        return AHTXX_ACK_ERROR;
    }
    usleep(AHTXX_SOFT_RESET_DELAY * 1000);
    return AHTXX_NO_ERROR;
}

int aht10_start_measurement(AHT10_DEV* dev) {
    uint8_t cmd[3] = {
        AHTXX_START_MEASUREMENT_REG,
        AHTXX_START_MEASUREMENT_CTRL,
        AHTXX_START_MEASUREMENT_CTRL_NOP
    };
    
    if (write(dev->i2c_fd, cmd, 3) != 3) {
        perror("Error en inicio de medición");
        return AHTXX_ACK_ERROR;
    }
    return AHTXX_NO_ERROR;
}

int aht10_read_data(AHT10_DEV* dev, float* temperature, float* humidity) {
    // 1. Iniciar medición
    int status = aht10_start_measurement(dev);
    if (status != AHTXX_NO_ERROR) return status;
    
    // 2. Esperar hasta que el sensor esté listo
    usleep(AHTXX_MEASUREMENT_DELAY * 1000);
    
    // 3. Leer datos (7 bytes: estado + 6 datos)
    uint8_t data[7] = {0};
    if (read(dev->i2c_fd, data, sizeof(data)) != sizeof(data)) {
        perror("Error en lectura de datos");
        return AHTXX_DATA_ERROR;
    }
    
    // 4. Verificar bit BUSY
    if (data[0] & AHTXX_STATUS_CTRL_BUSY) {
        return AHTXX_BUSY_ERROR;
    }
    
    // 5. Procesar datos de humedad (20 bits)
    uint32_t raw_humidity = ((uint32_t)data[1] << 12) | 
                            ((uint32_t)data[2] << 4) | 
                            (data[3] >> 4);
    
    // 6. Procesar datos de temperatura (20 bits)
    uint32_t raw_temperature = ((uint32_t)(data[3] & 0x0F) << 16) | 
                               ((uint32_t)data[4] << 8) | 
                               data[5];
    
    // 7. Convertir a valores reales
    *humidity = ((float)raw_humidity / (1 << 20)) * 100.0;
    *temperature = ((float)raw_temperature / (1 << 20)) * 200.0 - 50.0;
    
    *temperature -= 4.0
    ;
    return AHTXX_NO_ERROR;
}

// Función para formatear datos en vector ASCII (uint8_t)
void format_sensor_data(float temperature, float humidity, uint8_t *buffer, size_t *length) {
    // Convertir valores a strings
    char temp_str[8];
    char hum_str[8];
    snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
    snprintf(hum_str, sizeof(hum_str), "%.2f", humidity);
    
    // Formatear en buffer como caracteres ASCII individuales
    int len = snprintf((char*)buffer, LORA_PAYLOAD_SIZE, 
                      "T:%s,H:%s", 
                      temp_str, hum_str);
    
    // Establecer longitud real (sin incluir el null terminator)
    *length = (len > 0 && len < LORA_PAYLOAD_SIZE) ? len : 0;
}

void aht10_close(AHT10_DEV* dev) {
    if (dev) {
        close(dev->i2c_fd);
        free(dev);
    }
}
