# Compiler
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -pedantic -I. -fPIC
LDFLAGS = -ldl

# Targets
all: RobotWarz test_robot

# ============ RobotWarz ============
RobotWarz: main.o Arena.o RobotBase.o DeathTracker.o
	$(CXX) $(CXXFLAGS) main.o Arena.o RobotBase.o DeathTracker.o -o RobotWarz $(LDFLAGS)

main.o: main.cpp Arena.h
	$(CXX) $(CXXFLAGS) -c main.cpp

Arena.o: Arena.cpp Arena.h RobotBase.h DeathTracker.h
	$(CXX) $(CXXFLAGS) -c Arena.cpp

# ============ DeathTracker ============
DeathTracker.o: DeathTracker.cpp DeathTracker.h RobotBase.h
	$(CXX) $(CXXFLAGS) -c DeathTracker.cpp -o DeathTracker.o

# ============ RobotBase ============
RobotBase.o: RobotBase.cpp RobotBase.h RadarObj.h
	$(CXX) $(CXXFLAGS) -c RobotBase.cpp -o RobotBase.o

# ============ Test Robot ============
test_robot: test_robot.cpp RobotBase.o
	$(CXX) $(CXXFLAGS) test_robot.cpp RobotBase.o -ldl -o test_robot

# ============ Cleanup ============
clean:
	rm -f *.o RobotWarz test_robot *.so

run: RobotWarz
	./RobotWarz

.PHONY: all clean run	