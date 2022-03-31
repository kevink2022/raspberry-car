TARGET=hw0a8cam2inst

SOURCES=raspicam_wrapper.cpp \
        hw0a8cam2inst.c

OBJECTS=$(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SOURCES)))

all: $(OBJECTS)
	g++ $(OBJECTS) -lraspicam -o $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< -o $@

%.o:%.cpp
	g++ -c $< -o $@
