#
# Makefile for Quadcopter Dynamics Loop Test
#
CC=g++
CFLAGS = -g -Wall
OBJ = dynamicsSimulation.o Quadcopter.o LinearUpdater.o RotationUpdater.o \
        GetLinearAccel.o GetAngularVel.o

#OBJ = dynamicslooptest.o Quadcopter.o LinearUpdater.o RotationUpdater.o \
#        GetLinearAccel.o GetAngularVel.o
#dynamicsloop: $(OBJ)
#	$(CC) $(CFLAGS) -o dynamicsloop $(OBJ)

#dynamicslooptest.o: dynamicslooptest.cpp
#	$(CC) $(CFLAGS) -c dynamicslooptest.cpp

dynamicsSimul: $(OBJ)
	$(CC) $(CFLAGS) -o dynamicsSimulation $(OBJ)

dynamicsSimulation.o: dynamicsSimulation.cpp
	$(CC) $(CFLAGS) -c dynamicsSimulation.cpp

Quadcopter.o: Quadcopter.cpp
	$(CC) $(CFLAGS) -c Quadcopter.cpp

LinearUpdater.o: LinearUpdater.cpp
	$(CC) $(CFLAGS) -c LinearUpdater.cpp

RotationUpdater.o: RotationUpdater.cpp
	$(CC) $(CFLAGS) -c RotationUpdater.cpp

GetLinearAccel.o: GetLinearAccel.cpp
	$(CC) $(CFLAGS) -c GetLinearAccel.cpp

GetAngularVel.o: GetAngularVel.cpp
	$(CC) $(CFLAGS) -c GetAngularVel.cpp

clean:
	rm dynamicsloop *.o
