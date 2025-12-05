#ifndef ARENA_H
#define ARENA_H
#include <vector>
#include <string>
#include "RobotBase.h"
#include "RadarObj.h"
#include "DeathTracker.h"


class Arena {
private:
    std::vector<RobotBase*> robots;
    std::vector<void*> library_handles;
    int rows;
    int cols;
    int current_round;
    std::vector<std::vector<char>> board;
    DeathTracker death_registry;
    bool is_live;

public:
    Arena();
    ~Arena();
    void run_sim(bool live = false);
    
private:
    void loadRobots();
    void placeRobots();
    void placeObstacles();
    void printBoard() const;
    void clearScreen();
    bool checkWinner() const;
    bool loadSingleRobot(const std::string& filename);
    char convertToRadarType(char display_char) const;
    void scanCell(int row, int col, std::vector<RadarObj>& results) const;
    std::vector<RadarObj> scanRadar(RobotBase* robot, int direction);
    void handleShooting(RobotBase* shooter, int target_row, int target_col);
    void handleMovement(RobotBase* robot, int direction, int distance);
    bool isValidMove(int row, int col, RobotBase* robot) const;
    void applyObstacleEffect(RobotBase* robot, char obstacle_type, int row, int col);
};

#endif