TARGET=hw7kelly

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        hw7kelly.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -lpthread -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
