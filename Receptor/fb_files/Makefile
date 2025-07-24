TARGET   = power_watch
CROSS    = aarch64-linux-gnu
CC       = $(CROSS)-gcc-10
AS       = $(CROSS)-as-10
LD       = $(CROSS)-ld-10
LIB_DIR  = -Lliblvgl 
LIBS     = -llvgl -lm
#LVGL_DIR = /home/carlos/Proyects/PowerMeter/PowerWatchSoftware/lvgl/littlevgl-8
LVGL_DIR = /Work/Proyects/PowerMeter/PowerWatchSoftware/lvgl/
CFLAGS   = -I$(LVGL_DIR)/lvgl -I$(LVGL_DIR)/lv_drivers -I$(LVGL_DIR)/ -Iui/
OBJECTS  = main.o  fbdev.o  evdev.o  ui.o  ui_helpers.o ui_Screen1.o mouse_cursor_icon.o

vpath = %.c ./ui:./ui/screens/

all: $(TARGET)

power_watch: $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@  ${LIB_DIR}  ${LIBS}  
#	chmod +x $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< 
%.o: %.S
	$(AS) $(AFLAGS) -c $< -o $@
clean:
	rm -rf  $(OBJECTS) *.elf *.bin .*~ *~ *.map *.d *.hex *.lst $(TARGET)
.PHONY: clean



