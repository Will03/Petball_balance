
# All warnings
CPPFLAGS += -Wall



all: test
test: i2c_bus.o Pid.o lsm6.o main.o
	g++ -std=gnu++11 -o test Pid.o i2c_bus.o lsm6.o main.o -lwiringPi 
Pid.o: Pid.cpp
	g++ -std=gnu++11 -c Pid.cpp
i2c_bus.o: i2c_bus.cpp
	g++ -std=gnu++11 -c i2c_bus.cpp
lsm6.o: lsm6.cpp
	g++ -std=gnu++11 -c lsm6.cpp
main.o: main.cpp
	g++ -std=gnu++11 -c main.cpp -lwiringPi

clean:
	rm -f lsm6.o main.o i2c_bus.o Pid.o