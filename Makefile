
CC = g++
LD = g++
LDFLAGS = -lfreenect
CFLAGS=-g -Wall -I/usr/local/include/libfreenect
LIBS = -lSystem -lgcc -framework OpenGL -framework GLUT 

PROGS = kinect_record

all: $(PROGS)

kinect_record: kinect_record.o
	$(LD) $(LDFLAGS) $(LIBS) $^ -o $@

%.o: %.cpp
	$(CC) $(CFLAGS)  $(LIBS) -c $<

clean:
	rm -rf *.o $(PROGS)

