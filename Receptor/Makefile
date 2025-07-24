TARGET   = proyecto 
CROSS    = aarch64-linux-gnu
CROSS    =arm-linux-gnueabihf
#CROSS    = aarch64-none-linux-gnu
#CC       = $(CROSS)-gcc-10.2.0
#AS       = $(CROSS)-as-10.2.0
#LD       = $(CROSS)-ld-10.2.0
CC       = $(CROSS)-gcc
AS       = $(CROSS)-as
LD       = $(CROSS)-ld

LIB_DIR  = -Lliblvgl 
LIBS     = -llvgl -lm
#LVGL_DIR = /home/carlos/Proyects/PowerMeter/PowerWatchSoftware/lvgl/littlevgl-8
LVGL_DIR = /root/lvgl/littlevgl-8/lvgl
DRV_DIR  = /root/lvgl/littlevgl-8/lv_drivers/
CFLAGS   = -I$(LVGL_DIR)/ -I$(DRV_DIR) -Iui
OBJECTS  =  main.o  fbdev.o reciver7.o ui.o ui_Screen1.o
VPATH = ./ui:./ui/screens/:./fb_files/:./ui/images

all: $(TARGET)

proyecto: $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@  ${LIB_DIR}  ${LIBS}  
#	chmod +x $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< 
%.o: %.S
	$(AS) $(AFLAGS) -c $< -o $@
clean:
	rm -rf  $(OBJECTS) *.elf  .*~ *~ *.map *.d *.hex *.lst $(TARGET)
.PHONY: clean



