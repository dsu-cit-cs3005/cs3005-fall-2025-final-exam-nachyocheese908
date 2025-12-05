// Robot_YourName.cpp
#include "RobotBase.h"
#include <vector>
#include <iostream>
#include <cmath>

class Robot_YourName : public RobotBase {
private:
    // Add your private variables here
    int last_radar_direction = 1;
    int target_row = -1;
    int target_col = -1;
    bool enemy_spotted = false;
        
public:
    // Choose your stats: move (2-5), armor (2-5), weapon
    // move + armor must equal 7, min 2 each, max 5 each
    Robot_YourName() : RobotBase(3, 4, hammer) { // Example: 3 move, 4 armor, railgun
        // Initialize your robot
    }
    
    virtual void get_radar_direction(int& radar_direction) override {
        // Implement your radar strategy
        radar_direction = last_radar_direction;
        last_radar_direction = (last_radar_direction % 8) + 1; // Cycle through directions
    }
    
    virtual void process_radar_results(const std::vector<RadarObj>& radar_results) override {
        enemy_spotted = false;
        for (const auto& obj : radar_results) {
            if (obj.m_type == 'R') { // Enemy robot
                target_row = obj.m_row;
                target_col = obj.m_col;
                enemy_spotted = true;
                break;
            }
        }
    }
    
    virtual bool get_shot_location(int& shot_row, int& shot_col) override {
        if (enemy_spotted) {
            shot_row = target_row;
            shot_col = target_col;
            return true;
        }
        return false;
    }
    
    virtual void get_move_direction(int& direction, int& distance) override {
        if (enemy_spotted) {
            // Move toward enemy
            int current_row, current_col;
            get_current_location(current_row, current_col);
            
            // Simple movement toward target
            if (abs(target_row - current_row) > abs(target_col - current_col)) {
                direction = (target_row > current_row) ? 5 : 1; // Down or Up
            } else {
                direction = (target_col > current_col) ? 3 : 7; // Right or Left
            }
            distance = 1;
        } else {
            // Random or patrolling movement
            direction = 1 + (rand() % 8); // 1-8
            distance = 1;
        }
    }
};

extern "C" RobotBase* create_robot() {
    return new Robot_YourName();
}