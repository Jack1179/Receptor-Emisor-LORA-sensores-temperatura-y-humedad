#ifndef LORA_LINUX_H
#define LORA_LINUX_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Definiciones de registros
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_VERSION              0x42
#define REG_IRQ_FLAGS            0x12
#define REG_FIFO                 0x00
#define REG_PAYLOAD_LENGTH       0x22
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_RX_NB_BYTES          0x13
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_SYNC_WORD            0x39
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f

// Modos de operación
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// Máscaras de interrupción
#define IRQ_TX_DONE_MASK         0x08
#define IRQ_RX_DONE_MASK         0x40

#define MAX_PKT_LENGTH           255

typedef struct {
    int spi_fd;          // Descriptor SPI
    int reset_gpio;      // Pin GPIO para reset
    long frequency;      // Frecuencia actual
    bool implicit_header; // Modo de cabecera
    int packet_index;    // Índice del paquete
} LoRaContext;

// Prototipos de funciones
int lora_init(LoRaContext* ctx, long frequency);
void lora_close(LoRaContext* ctx);
int lora_begin_packet(LoRaContext* ctx, bool implicitHeader);
int lora_end_packet(LoRaContext* ctx);
int lora_parse_packet(LoRaContext* ctx);
int lora_write(LoRaContext* ctx, const uint8_t* buffer, size_t size);
int lora_read(LoRaContext* ctx, uint8_t* buffer, size_t size);
void lora_receive(LoRaContext* ctx);
void lora_set_frequency(LoRaContext* ctx, long frequency);
void lora_set_tx_power(LoRaContext* ctx, int level);
int lora_available(LoRaContext* ctx);

#endif // LORA_LINUX_H
