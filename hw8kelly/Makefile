TARGET=hw10kelly

SOURCES=raspicam_wrapper.cpp \
		import_registers.c \
		enable_pwm_clock.c \
        hw10kelly.c

OBJECTS=$(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SOURCES)))

all: $(OBJECTS)
	g++ $(OBJECTS) -lraspicam -lpthread -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@

%.o:%.cpp
	g++ -c $< -o $@