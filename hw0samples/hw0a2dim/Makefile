TARGET=hw0a2dim

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        wait_period.c \
        hw0a2dim.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -lpthread -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
