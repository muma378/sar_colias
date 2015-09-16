CC = g++
CFLAGS = -Wall -c -g -std=c++11

PLAYERCPP_CFLAGS = `pkg-config --cflags playerc++`
PLAYERCPP_LDFLAGS = `pkg-config --libs playerc++`
OPENCV_CFLAGS = `pkg-config --cflags opencv`
OPENCV_LDFLAGS = `pkg-config --libs opencv`

CFLAGS += $(PLAYERCPP_CFLAGS) $(OPENCV_CFLAGS)
LDFLAGS = $(PLAYERCPP_LDFLAGS) $(OPENCV_LDFLAGS)

sar_colias: main.o robot.o
	$(CC) main.o robot.o -o sar_colias $(LDFLAGS)

main.o: main.cc config.h
	$(CC) $(CFLAGS) main.cc

robot.o: robot.cc robot.h
	$(CC) $(CFLAGS) robot.cc

clean:
	rm -f *.o *.c~ *.h~ simple