CC = g++
CFLAGS = -Wall -c -g -std=c++11

PLAYERCPP_CFLAGS = `pkg-config --cflags playerc++`
PLAYERCPP_LDFLAGS = `pkg-config --libs playerc++`
OPENCV_CFLAGS = `pkg-config --cflags opencv`
OPENCV_LDFLAGS = `pkg-config --libs opencv`

CFLAGS += $(PLAYERCPP_CFLAGS) $(OPENCV_CFLAGS)
LDFLAGS = $(PLAYERCPP_LDFLAGS) $(OPENCV_LDFLAGS)

sar_colias: swarm.o robot.o utils.o
	$(CC) swarm.o robot.o utils.o -o sar_colias $(LDFLAGS)

swarm.o: swarm.cc swarm.h config.h
	$(CC) $(CFLAGS) swarm.cc

robot.o: robot.cc robot.h
	$(CC) $(CFLAGS) robot.cc

utils.o: utils.cc utils.h
	$(CC) $(CFLAGS) utils.cc

clean:
	rm -f *.o *.c~ *.h~ *.log sar_colias