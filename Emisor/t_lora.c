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
#include "lora_v2.h"

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

// Definiciones para ADS1115
#define ADS1115_ADDRESS 0x48
#define ADS1115_CONFIG_REG 0x01
#define ADS1115_CONVERSION_REG 0x00

// Configuración para AIN0 ±4.096V
#define ADS1115_CONFIG_MSB 0xC2  // OS=1 (iniciar), MUX=100 (AIN0), PGA=001 (±4.096V)
#define ADS1115_CONFIG_LSB 0x83  // DR=100 (128 SPS), COMP_QUE=11 (deshabilitar)

typedef struct {
    int i2c_fd;
    uint8_t address;
} ADS1115_DEV;

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
void format_sensor_data(float temperature, float humidity,float soil_moisture, uint8_t *buffer, size_t *length);
ADS1115_DEV* ads1115_init(const char* i2c_device, uint8_t address);
int ads1115_read_voltage(ADS1115_DEV* dev, float* voltage);
void ads1115_close(ADS1115_DEV* dev);

int main() {

    AHT10_DEV* sensor = aht10_init("/dev/i2c-1", AHTXX_ADDRESS_X38);
    if (!sensor) {
        fprintf(stderr, "Error al inicializar el sensor\n");
        return EXIT_FAILURE;
    }
    
    // Inicializar ADS1115 (bus I2C-2)
    ADS1115_DEV* ads1115 = ads1115_init("/dev/i2c-2", ADS1115_ADDRESS);
    if (!ads1115) {
        aht10_close(sensor);
        return EXIT_FAILURE;
    }


    // Esperar inicialización del sensor
    usleep(AHT1X_POWER_ON_DELAY * 1500);
    
    // Configuración completa del sensor
    if (aht10_initialize(sensor) != AHTXX_NO_ERROR) {
        fprintf(stderr, "Error en configuración inicial\n");
        aht10_close(sensor);
        return EXIT_FAILURE;
    }

 LoRaContext lora;
    if (lora_init(&lora, 433000000) != 0) {
        fprintf(stderr, "Error inicializando LoRa\n");
        aht10_close(sensor);
        return EXIT_FAILURE;
    }

    // Buffer para datos del sensor
    uint8_t lora_payload[LORA_PAYLOAD_SIZE];
    size_t payload_length ;

    while (1) {
        float temperatura, humedad;
        if (aht10_read_data(sensor, &temperatura, &humedad) == 0) {

            // Leer humedad del suelo (HT)
            float voltage, soil_moisture = 0.0;
            if (ads1115_read_voltage(ads1115, &voltage) == 0) {
                float voltaje_real = 3.3 - voltage;
                soil_moisture = voltaje_real * 100.0 / 3.3;
                if (soil_moisture < 0) soil_moisture = 0;
                if (soil_moisture > 100) soil_moisture = 100;
            }
            
            // Formatear datos para transmisión LoRa
            format_sensor_data(temperatura, humedad,soil_moisture, lora_payload, &payload_length);
            
				for(int i=0; i<15;i++){
                // Transmitir por LoRa
                lora_begin_packet(&lora, false);
                lora_write(&lora, lora_payload, payload_length);
                lora_end_packet(&lora);
                
                printf("Datos enviados: Temp=%.2f°C, Hum=%.2f%%, SoilH=%.1f%%\n ", temperatura, humedad, soil_moisture);
                // Mostrar valores hexadecimales
                printf("Hex: ");
                for (size_t i = 0; i < payload_length; i++) {
                    printf("%02X ", lora_payload[i]); // Mostrar hex
                }
                printf("\n");
                usleep(1000);
            }
        }
        sleep(10); // Esperar 15 segundos entre lecturas
    }

    // Limpieza (aunque el bucle es infinito)
    aht10_close(sensor);
    ads1115_close(ads1115);
    lora_close(&lora);
    return EXIT_SUCCESS;
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
    usleep(10000);  // Esperar 10ms para que se complete la calibración
    
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
void format_sensor_data(float temperature, float humidity,float soil_moisture, uint8_t *buffer, size_t *length) {
    // Convertir valores a strings
    char temp_str[8];
    char hum_str[8];
    char soil_str[8];
    snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
    snprintf(hum_str, sizeof(hum_str), "%.2f", humidity);
    snprintf(soil_str, sizeof(soil_str), "%.1f", soil_moisture);
    
    // Formatear en buffer como caracteres ASCII individuales
    int len = snprintf((char*)buffer, LORA_PAYLOAD_SIZE, 
                      "T:%s,H:%s,HT:%s", 
                      temp_str, hum_str,soil_str);
    
    // Establecer longitud real (sin incluir el null terminator)
    *length = (len > 0 && len < LORA_PAYLOAD_SIZE) ? len : 0;
}

void aht10_close(AHT10_DEV* dev) {
    if (dev) {
        close(dev->i2c_fd);
        free(dev);
    }
}

// Inicializar ADS1115
ADS1115_DEV* ads1115_init(const char* i2c_device, uint8_t address) {
    int fd = open(i2c_device, O_RDWR);
    if (fd < 0) {
        perror("Error al abrir I2C (ADS1115)");
        return NULL;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        perror("Error en ioctl (ADS1115)");
        close(fd);
        return NULL;
    }

    ADS1115_DEV* dev = malloc(sizeof(ADS1115_DEV));
    dev->i2c_fd = fd;
    dev->address = address;
    return dev;
}

// Leer voltaje del ADS1115
int ads1115_read_voltage(ADS1115_DEV* dev, float* voltage) {
    char config[3] = {
        ADS1115_CONFIG_REG,
        ADS1115_CONFIG_MSB,
        ADS1115_CONFIG_LSB
    };
    
    if (write(dev->i2c_fd, config, 3) != 3) {
        perror("Error configurando ADS1115");
        return -1;
    }
    
    usleep(8000);  // Esperar conversión
    
    char reg = ADS1115_CONVERSION_REG;
    if (write(dev->i2c_fd, &reg, 1) != 1) {
        perror("Error seleccionando registro");
        return -1;
    }
    
    char data[2];
    if (read(dev->i2c_fd, data, 2) != 2) {
        perror("Error leyendo datos ADS1115");
        return -1;
    }
    
    int16_t raw = (data[0] << 8) | data[1];
    *voltage = (float)raw * 4.096 / 32768.0;
    return 0;
}

void ads1115_close(ADS1115_DEV* dev) {
    if (dev) {
        close(dev->i2c_fd);
        free(dev);
    }
}
