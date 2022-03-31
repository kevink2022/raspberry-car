TARGET=hw8kelly

SOURCES=raspicam_wrapper.cpp \
		import_registers.c \
        enable_pwm_clock.c \
        hw8kelly.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -lpthread -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@
