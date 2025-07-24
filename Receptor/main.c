
/**
 * @file main
 *
 */

/*********************
 *      INCLUDES
 *********************/
#define _DEFAULT_SOURCE /* needed for usleep() */
#include <unistd.h>
#define SDL_MAIN_HANDLED /*To fix SDL's "undefined reference to WinMain" issue*/
#include "lvgl.h"
#include "fb_files/fbdev.h"
#include "fb_files/evdev.h"
#include "ui/ui.h"
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#define DISP_BUF_SIZE (128 * 2048)
static void hal_init(void);
uint8_t spi_transfer(uint8_t data);
uint8_t lora_read_reg(uint8_t reg);
void lora_write_reg(uint8_t reg, uint8_t value);
void lora_init(long frequency);
void setup_lora_receiver();
int receive_packet(uint8_t *buffer, int *length, int *rssi, float *snr);

int main(int argc, char **argv)
{
  (void)argc; /*Unused*/
  (void)argv; /*Unused*/
  float temperatura = 0.0;
  float humedad = 0.0;
  float humedad_suelo = 0.0;
char buffer[20]; // Buffer para formatear texto
  /*Initialize LVGL*/
  lv_init();
  /*Initialize the HAL (display, input devices, tick) for LVGL*/
  hal_init();
  ui_init();
  // Inicializar LoRa a 433 MHz
    lora_init(433000000);
    
    // Configurar como receptor
    setup_lora_receiver();
    
 
    printf("Esperando paquetes LoRa...\n");   
    uint8_t rx_buffer[255];
    int length;
    int rssi;
    float snr;
  while(1) {
      /* Periodically call the lv_task handler.
       * It could be done in a timer interrupt or an OS task too.*/
      //lv_tick_inc(5); 
      lv_timer_handler();
    

     if (receive_packet(rx_buffer, &length, &rssi, &snr)) {
          // Convertir a cadena terminada en null
          char payload[length + 1];
          memcpy(payload, rx_buffer, length);
          payload[length] = '\0';
          
          printf("Payload recibido: %s\n", payload); // Depuración

          // Parsear los valores manualmente
          char* ptr = payload;
          while (*ptr) {
              if (strncmp(ptr, "T:", 2) == 0) {
                  sscanf(ptr + 2, "%f", &temperatura);
              }
              else if (strncmp(ptr, "H:", 2) == 0) {
                  sscanf(ptr + 2, "%f", &humedad);
              }
              else if (strncmp(ptr, "HT:", 3) == 0) {
                  sscanf(ptr + 3, "%f", &humedad_suelo);
              }
              
              // Avanzar al siguiente campo
              ptr = strchr(ptr, ',');
              if (ptr) ptr++;
              else break;
          }

          printf("Valores parseados: T=%.1f, H=%.1f, HS=%.1f\n", 
                 temperatura, humedad, humedad_suelo); // Depuración

          // Actualizar la interfaz de usuario
          lv_arc_set_value(ui_Arc1, (int)temperatura);
          lv_arc_set_value(ui_Arc2, (int)humedad);
          lv_arc_set_value(ui_Arc3, (int)humedad_suelo);
          
          // Actualizar etiquetas con valores numéricos
          snprintf(buffer, sizeof(buffer), "T:%.1f", temperatura);
          lv_label_set_text(ui_Label4, buffer);
          
          snprintf(buffer, sizeof(buffer), "H:%.1f", humedad);
          lv_label_set_text(ui_Label5, buffer);
          
          snprintf(buffer, sizeof(buffer), "HS:%.1f", humedad_suelo);
          lv_label_set_text(ui_Label6, buffer);
      }
      usleep(10);
  }

 
    return 0;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Initialize the Hardware Abstraction Layer (HAL) for LVGL
 */
static void hal_init(void)
{


	/*LVGL init*/
  lv_init();
  /*Linux frame buffer device init*/
  fbdev_init();
  /*A small buffer for LittlevGL to draw the screen's content*/
  static lv_color_t buf[DISP_BUF_SIZE];

  /*Initialize a descriptor for the buffer*/
  static lv_disp_draw_buf_t disp_buf;
  lv_disp_draw_buf_init(&disp_buf, buf, NULL, DISP_BUF_SIZE);
  /*Initialize and register a display driver*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.draw_buf   = &disp_buf;
  disp_drv.flush_cb   = fbdev_flush;
  disp_drv.hor_res    = 320;
  disp_drv.ver_res    = 240;
  lv_disp_drv_register(&disp_drv);

//evdev_init();
  
  
 
}


uint32_t custom_tick_get(void)
{
    static uint64_t start_ms = 0;
    if(start_ms == 0) {
        struct timeval tv_start;
        gettimeofday(&tv_start, NULL);
        start_ms = (tv_start.tv_sec * 1000000 + tv_start.tv_usec) / 1000;
    }

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    uint64_t now_ms;
    now_ms = (tv_now.tv_sec * 1000000 + tv_now.tv_usec) / 1000;

    uint32_t time_ms = now_ms - start_ms;
    return time_ms;
}


