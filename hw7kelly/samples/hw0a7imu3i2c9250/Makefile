TARGET=hw0a7imu3i2c9250

SOURCES=import_registers.c \
        enable_pwm_clock.c \
        wait_period.c \
        wait_key.c \
        hw0a7imu3i2c9250.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc -g $(OBJECTS) -lpthread -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -g -c $< -o $@
