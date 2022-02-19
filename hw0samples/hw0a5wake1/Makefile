TARGET=hw0a5wake1

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        wait_period.c \
        hw0a5wake1.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -lpthread -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
