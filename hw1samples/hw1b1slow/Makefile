TARGET=hw1b1slow

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        hw1b1slow.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
