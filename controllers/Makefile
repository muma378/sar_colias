CC = g++
CFLAGS = -Wall -c -g -std=c++11

PLAYERCPP_CFLAGS = `pkg-config --cflags playerc++`
PLAYERCPP_LDFLAGS = `pkg-config --libs playerc++`
OPENCV_CFLAGS = `pkg-config --cflags opencv`
OPENCV_LDFLAGS = `pkg-config --libs opencv`

CFLAGS += $(PLAYERCPP_CFLAGS) $(OPENCV_CFLAGS)
LDFLAGS = $(PLAYERCPP_LDFLAGS) $(OPENCV_LDFLAGS)

sar_colias: swarm.o robot.o utils.o targets.o
	$(CC) swarm.o robot.o utils.o targets.o -o sar_colias $(LDFLAGS)

utils.o: utils.cc utils.h
	$(CC) $(CFLAGS) utils.cc

targets.o: targets.cc targets.h
	$(CC) $(CFLAGS) targets.cc

robot.o: robot.cc robot.h
	$(CC) $(CFLAGS) robot.cc

swarm.o: swarm.cc swarm.h config.h
	$(CC) $(CFLAGS) swarm.cc

clean:
	rm -f *.o *.c~ *.h~ *.log sar_colias