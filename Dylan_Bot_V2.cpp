#include "RobotBase.h"
#include <cstdlib>
#include <ctime>
#include <set>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <algorithm>

class Hunter_Killer : public RobotBase 
{
private:
    // Simple tracking
    int target_row = -1;
    int target_col = -1;
    bool has_target = false;
    
    // Survival tracking
    int last_health = 100;
    int danger_count = 0;
    
    // Obstacle memory
    std::set<std::pair<int, int>> obstacles;
    
    // Grenade ammo tracking
    int grenade_count = 0;
    
    // Simple helper functions
    int distance(int r1, int c1, int r2, int c2) {
        return abs(r1 - r2) + abs(c1 - c2);
    }
    
    bool is_valid(int r, int c) {
        return r >= 0 && r < m_board_row_max && c >= 0 && c < m_board_col_max;
    }
    
    // ULTRA-AGGRESSIVE TARGETING
    void find_best_target(const std::vector<RadarObj>& radar_results) {
        int my_r, my_c;
        get_current_location(my_r, my_c);
        
        has_target = false;
        int best_dist = 9999;
        
        // First priority: ANY enemy robot
        for (const auto& obj : radar_results) {
            if (obj.m_type == 'R') {  // Live robot
                int d = distance(my_r, my_c, obj.m_row, obj.m_col);
                if (d < best_dist) {
                    best_dist = d;
                    target_row = obj.m_row;
                    target_col = obj.m_col;
                    has_target = true;
                }
            }
        }
        
        // Second priority: Dead robots for positioning
        if (!has_target) {
            for (const auto& obj : radar_results) {
                if (obj.m_type == 'X') {  // Dead robot = likely battle area
                    target_row = obj.m_row;
                    target_col = obj.m_col;
                    has_target = true;
                    break;
                }
            }
        }
        
        // Fallback: move to center
        if (!has_target) {
            target_row = m_board_row_max / 2;
            target_col = m_board_col_max / 2;
            has_target = true;
        }
    }
    
    // SIMPLE BUT EFFECTIVE MOVEMENT
    std::pair<int, int> calculate_move() {
        int my_r, my_c;
        get_current_location(my_r, my_c);
        
        // If low health, run away from target
        if (get_health() < 40 && has_target) {
            int dr = my_r - target_row;
            int dc = my_c - target_col;
            
            // Normalize
            if (dr != 0) dr = dr > 0 ? 1 : -1;
            if (dc != 0) dc = dc > 0 ? 1 : -1;
            
            int new_r = my_r + dr;
            int new_c = my_c + dc;
            
            // Bound check
            new_r = std::max(0, std::min(m_board_row_max - 1, new_r));
            new_c = std::max(0, std::min(m_board_col_max - 1, new_c));
            
            return {new_r, new_c};
        }
        
        // AGGRESSIVE: Move toward target
        if (has_target) {
            int dr = target_row - my_r;
            int dc = target_col - my_c;
            
            // Normalize
            if (dr != 0) dr = dr > 0 ? 1 : -1;
            if (dc != 0) dc = dc > 0 ? 1 : -1;
            
            // Hammer bots want to get CLOSE
            if (get_weapon() == hammer) {
                // Get as close as possible
                int new_r = my_r + dr;
                int new_c = my_c + dc;
                
                // Bound check
                new_r = std::max(0, std::min(m_board_row_max - 1, new_r));
                new_c = std::max(0, std::min(m_board_col_max - 1, new_c));
                
                return {new_r, new_c};
            }
            // Railgun bots want medium range (4-6 tiles)
            else if (get_weapon() == railgun) {
                int dist = distance(my_r, my_c, target_row, target_col);
                if (dist > 6) {
                    // Move toward
                    int new_r = my_r + dr;
                    int new_c = my_c + dc;
                    new_r = std::max(0, std::min(m_board_row_max - 1, new_r));
                    new_c = std::max(0, std::min(m_board_row_max - 1, new_c));
                    return {new_r, new_c};
                } else if (dist < 4) {
                    // Move away
                    int new_r = my_r - dr;
                    int new_c = my_c - dc;
                    new_r = std::max(0, std::min(m_board_row_max - 1, new_r));
                    new_c = std::max(0, std::min(m_board_row_max - 1, new_c));
                    return {new_r, new_c};
                } else {
                    // Good range, move laterally for better angle
                    int new_r = my_r;
                    int new_c = my_c;
                    if (abs(dr) > abs(dc)) {
                        new_c += (rand() % 2) ? 1 : -1;
                    } else {
                        new_r += (rand() % 2) ? 1 : -1;
                    }
                    new_r = std::max(0, std::min(m_board_row_max - 1, new_r));
                    new_c = std::max(0, std::min(m_board_row_max - 1, new_c));
                    return {new_r, new_c};
                }
            }
        }
        
        // Default: random exploration
        int new_r = my_r + (rand() % 3) - 1;
        int new_c = my_c + (rand() % 3) - 1;
        new_r = std::max(0, std::min(m_board_row_max - 1, new_r));
        new_c = std::max(0, std::min(m_board_row_max - 1, new_c));
        
        return {new_r, new_c};
    }

public:
    // DIFFERENT WEAPON CHOICES - TRY EACH TO SEE WHAT WORKS BEST
    
    // Option 1: HAMMER BOT (most kills in testing)
    //Hunter_Killer() : RobotBase(5, 2, hammer)  // Max speed, low armor
    //{
    //    srand(time(NULL));
    //}
    
    // Option 2: RAILGUN BOT (best range control)
    Hunter_Killer() : RobotBase(4, 3, railgun)  // Balanced
    {
        srand(time(NULL));
        grenade_count = 0;
    }
    
    // Option 3: FLAMETHROWER BOT (area denial)
    //Hunter_Killer() : RobotBase(3, 4, flamethrower)  // Tanky
    //{
    //    srand(time(NULL));
    //}
    
    // Option 4: GRENADE BOT (crowd control)
    //Hunter_Killer() : RobotBase(4, 3, grenade)  // Balanced
    //{
    //    srand(time(NULL));
    //    grenade_count = 15;
    //}
    
    virtual void get_radar_direction(int& radar_direction_out) override 
    {
        // SIMPLE: Always scan toward target if we have one
        if (has_target) {
            int my_r, my_c;
            get_current_location(my_r, my_c);
            
            int dr = target_row - my_r;
            int dc = target_col - my_c;
            
            // Convert to direction 1-8
            if (dr < 0 && dc == 0) radar_direction_out = 1;
            else if (dr < 0 && dc > 0) radar_direction_out = 2;
            else if (dr == 0 && dc > 0) radar_direction_out = 3;
            else if (dr > 0 && dc > 0) radar_direction_out = 4;
            else if (dr > 0 && dc == 0) radar_direction_out = 5;
            else if (dr > 0 && dc < 0) radar_direction_out = 6;
            else if (dr == 0 && dc < 0) radar_direction_out = 7;
            else if (dr < 0 && dc < 0) radar_direction_out = 8;
            else radar_direction_out = 1; // Default up
        } else {
            // Systematic scanning pattern
            static int scan_dir = 1;
            radar_direction_out = scan_dir;
            scan_dir = (scan_dir % 8) + 1;
        }
    }
    
    virtual void process_radar_results(const std::vector<RadarObj>& radar_results) override 
    {
        // Update grenade count
        if (get_weapon() == grenade) {
            grenade_count = get_grenades();
        }
        
        // Track obstacles
        for (const auto& obj : radar_results) {
            if (obj.m_type == 'M' || obj.m_type == 'P' || obj.m_type == 'F') {
                obstacles.insert({obj.m_row, obj.m_col});
            }
        }
        
        // Find target
        find_best_target(radar_results);
        
        // Track health changes
        int current_health = get_health();
        if (current_health < last_health) {
            danger_count++;
        } else {
            danger_count = std::max(0, danger_count - 1);
        }
        last_health = current_health;
    }
    
    virtual bool get_shot_location(int& shot_row, int& shot_col) override 
    {
        // ALWAYS SHOOT IF YOU CAN!
        if (!has_target) return false;
        
        int my_r, my_c;
        get_current_location(my_r, my_c);
        
        // Check if we're in range
        int dist = distance(my_r, my_c, target_row, target_col);
        
        // Different weapons have different range preferences
        WeaponType my_weapon = get_weapon();
        
        if (my_weapon == hammer) {
            // Hammer needs to be adjacent (distance 1)
            if (dist == 1) {
                shot_row = target_row;
                shot_col = target_col;
                return true;
            }
        }
        else if (my_weapon == railgun) {
            // Railgun can shoot any distance, but prefer not too close
            if (dist >= 2) {  // Don't shoot if adjacent
                shot_row = target_row;
                shot_col = target_col;
                
                // Add slight lead if target is moving
                if (dist > 3) {
                    // Predict movement toward us
                    int dr = my_r - target_row;
                    int dc = my_c - target_col;
                    if (abs(dr) > abs(dc)) {
                        shot_row += (dr > 0) ? 1 : -1;
                    } else {
                        shot_col += (dc > 0) ? 1 : -1;
                    }
                }
                return true;
            }
        }
        else if (my_weapon == flamethrower) {
            // Flamethrower has range 4
            if (dist <= 4 && dist >= 1) {
                shot_row = target_row;
                shot_col = target_col;
                return true;
            }
        }
        else if (my_weapon == grenade) {
            // Grenades have 3x3 area, need ammo
            if (grenade_count > 0 && dist <= 6) {
                shot_row = target_row;
                shot_col = target_col;
                return true;
            }
        }
        
        return false;
    }
    
    virtual void get_move_direction(int& move_direction, int& move_distance) override 
    {
        int my_r, my_c;
        get_current_location(my_r, my_c);
        
        // Calculate where we want to go
        auto [target_r, target_c] = calculate_move();
        
        // If we're already there, don't move
        if (target_r == my_r && target_c == my_c) {
            move_direction = 0;
            move_distance = 0;
            return;
        }
        
        // Calculate direction vector
        int dr = target_r - my_r;
        int dc = target_c - my_c;
        
        // Normalize to -1, 0, 1 for direction calculation
        int dir_r = (dr > 0) ? 1 : (dr < 0) ? -1 : 0;
        int dir_c = (dc > 0) ? 1 : (dc < 0) ? -1 : 0;
        
        // Convert to arena direction (1-8)
        if (dir_r < 0 && dir_c == 0) move_direction = 1; // Up
        else if (dir_r < 0 && dir_c > 0) move_direction = 2; // Up-Right
        else if (dir_r == 0 && dir_c > 0) move_direction = 3; // Right
        else if (dir_r > 0 && dir_c > 0) move_direction = 4; // Down-Right
        else if (dir_r > 0 && dir_c == 0) move_direction = 5; // Down
        else if (dir_r > 0 && dir_c < 0) move_direction = 6; // Down-Left
        else if (dir_r == 0 && dir_c < 0) move_direction = 7; // Left
        else if (dir_r < 0 && dir_c < 0) move_direction = 8; // Up-Left
        
        // Calculate distance (cap at move speed)
        int move_speed = get_move_speed();
        move_distance = std::min(move_speed, std::max(abs(dr), abs(dc)));
        
        // Adjust for obstacles
        for (int d = 1; d <= move_distance; d++) {
            int test_r = my_r + (dir_r * d);
            int test_c = my_c + (dir_c * d);
            
            // Check if cell is blocked by obstacle
            if (obstacles.find({test_r, test_c}) != obstacles.end()) {
                move_distance = d - 1;
                break;
            }
            
            // Check bounds
            if (!is_valid(test_r, test_c)) {
                move_distance = d - 1;
                break;
            }
        }
        
        // If we can't move in desired direction, try orthogonal
        if (move_distance == 0 && move_speed > 0) {
            // Try right
            if (is_valid(my_r, my_c + 1) && obstacles.find({my_r, my_c + 1}) == obstacles.end()) {
                move_direction = 3;
                move_distance = 1;
            }
            // Try down
            else if (is_valid(my_r + 1, my_c) && obstacles.find({my_r + 1, my_c}) == obstacles.end()) {
                move_direction = 5;
                move_distance = 1;
            }
            // Try left
            else if (is_valid(my_r, my_c - 1) && obstacles.find({my_r, my_c - 1}) == obstacles.end()) {
                move_direction = 7;
                move_distance = 1;
            }
            // Try up
            else if (is_valid(my_r - 1, my_c) && obstacles.find({my_r - 1, my_c}) == obstacles.end()) {
                move_direction = 1;
                move_distance = 1;
            }
        }
        
        // If in serious danger and can't move, at least try to change facing
        if (danger_count > 2 && move_distance == 0) {
            move_direction = (rand() % 8) + 1;
            move_distance = 0;
        }
    }
};

// Factory function
extern "C" RobotBase* create_robot() 
{
    return new Hunter_Killer();
}