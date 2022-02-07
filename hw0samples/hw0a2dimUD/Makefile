TARGET=hw0a2dimUD

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        wait_period.c \
        hw0a2dimUD.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -lpthread -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
