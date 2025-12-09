#include "RobotBase.h"
#include <cstdlib>
#include <ctime>
#include <set>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <algorithm>
#include <queue>

class DylanBot : public RobotBase 
{
private:
    // Enemy tracking
    struct Enemy {
        int row, col;
        int last_seen_round;
        int estimated_health;  // Track estimated enemy health
        bool operator<(const Enemy& other) const {
            return std::make_pair(row, col) < std::make_pair(other.row, other.col);
        }
    };
    
    // Arena state
    std::set<std::pair<int, int>> obstacles;  // M, P, F
    std::set<std::pair<int, int>> visited_cells;
    std::set<Enemy> known_enemies;
    
    // Combat state
    int current_round;
    int last_fire_round;
    int consecutive_misses;
    bool has_target;
    int target_row, target_col;
    
    // Movement strategy
    enum ZoneType { SAFE, DANGEROUS, COMBAT, LOOT };
    std::vector<std::pair<int, int>> safe_zones;
    std::vector<std::pair<int, int>> combat_zones;
    
    // Weapon-specific constants (based on arena mechanics)
    const int RAILGUN_DAMAGE = 15;  // Average 15 damage
    const int RAILGUN_RANGE = 100;  // Effectively unlimited range in arena
    const int MIN_RAILGUN_DISTANCE = 4;  // Minimum safe distance
    const int PREFERRED_RAILGUN_DISTANCE = 6;  // Optimal engagement distance
    
    // Grenade robot detection
    bool suspected_grenade_bot;
    int grenade_avoidance_timer;
    
    // Helper functions
    int manhattan_distance(int r1, int c1, int r2, int c2) const {
        return std::abs(r1 - r2) + std::abs(c1 - c2);
    }
    
    bool is_valid_cell(int row, int col) const {
        return row >= 0 && row < m_board_row_max && col >= 0 && col < m_board_col_max;
    }
    
    bool is_passable(int row, int col) const {
        // Check if cell contains an obstacle
        auto it = obstacles.find({row, col});
        return it == obstacles.end();
    }
    
    bool is_safe_cell(int row, int col) const {
        if (!is_passable(row, col)) return false;
        
        // Check for nearby dangerous obstacles
        for (int dr = -2; dr <= 2; dr++) {
            for (int dc = -2; dc <= 2; dc++) {
                int nr = row + dr;
                int nc = col + dc;
                if (obstacles.find({nr, nc}) != obstacles.end()) {
                    // If it's a pit (P) or flamethrower (F), it's dangerous
                    // Actually, all obstacles are dangerous in different ways
                    if (std::abs(dr) <= 1 && std::abs(dc) <= 1) {
                        return false;  // Too close to obstacle
                    }
                }
            }
        }
        return true;
    }
    
    // Get current location (const-safe version)
    void get_my_location(int& row, int& col) const {
        // We need to use the non-const version by casting away const
        // This is safe because we're not modifying the object
        const_cast<DylanBot*>(this)->RobotBase::get_current_location(row, col);
    }
    
    // Predict enemy movement (simple linear prediction)
    void predict_enemy_position(int current_row, int current_col, 
                                int& predicted_row, int& predicted_col,
                                int steps_ahead = 1) const {
        predicted_row = current_row;
        predicted_col = current_col;
        
        // Simple: assume enemy moves toward us or away
        int my_row, my_col;
        get_my_location(my_row, my_col);
        
        int dr = my_row - current_row;
        int dc = my_col - current_col;
        
        if (std::abs(dr) > std::abs(dc)) {
            // Move vertically
            predicted_row += (dr > 0) ? 1 : -1;
        } else {
            // Move horizontally
            predicted_col += (dc > 0) ? 1 : -1;
        }
    }
    
    // Find safest move direction
    std::pair<int, int> find_best_move(int current_row, int current_col) const {
        int best_row = current_row;
        int best_col = current_col;
        double best_score = -std::numeric_limits<double>::max();
        
        // Get move speed (need non-const access)
        int move_speed = const_cast<DylanBot*>(this)->get_move_speed();
        if (move_speed <= 0) return {current_row, current_col};
        
        // Evaluate all possible moves within range
        for (int dr = -move_speed; dr <= move_speed; dr++) {
            for (int dc = -(move_speed - std::abs(dr)); dc <= (move_speed - std::abs(dr)); dc++) {
                int new_row = current_row + dr;
                int new_col = current_col + dc;
                
                if (!is_valid_cell(new_row, new_col)) continue;
                if (!is_passable(new_row, new_col)) continue;
                
                // Calculate score for this position
                double score = 0.0;
                
                // 1. Distance to closest enemy (prefer medium range for railgun)
                int min_enemy_dist = std::numeric_limits<int>::max();
                for (const auto& enemy : known_enemies) {
                    int dist = manhattan_distance(new_row, new_col, enemy.row, enemy.col);
                    min_enemy_dist = std::min(min_enemy_dist, dist);
                    
                    // Ideal: stay at railgun range (6-8 tiles)
                    if (dist >= PREFERRED_RAILGUN_DISTANCE - 1 && dist <= PREFERRED_RAILGUN_DISTANCE + 1) {
                        score += 50.0;
                    }
                    // Too close: dangerous
                    else if (dist < MIN_RAILGUN_DISTANCE) {
                        score -= 100.0 / (dist + 1);
                    }
                    // Too far: can't shoot
                    else if (dist > 10) {
                        score -= 20.0;
                    }
                }
                
                // 2. Safety from obstacles
                if (is_safe_cell(new_row, new_col)) {
                    score += 30.0;
                }
                
                // 3. Avoid recently visited cells (exploration bonus)
                if (visited_cells.find({new_row, new_col}) == visited_cells.end()) {
                    score += 10.0;
                }
                
                // 4. Center positioning (better radar coverage)
                int center_row = m_board_row_max / 2;
                int center_col = m_board_col_max / 2;
                int center_dist = manhattan_distance(new_row, new_col, center_row, center_col);
                score += (20.0 / (center_dist + 1));
                
                // 5. Avoid edges (can get trapped)
                int edge_dist = std::min(std::min(new_row, m_board_row_max - 1 - new_row),
                                         std::min(new_col, m_board_col_max - 1 - new_col));
                score += edge_dist * 2.0;
                
                if (score > best_score) {
                    best_score = score;
                    best_row = new_row;
                    best_col = new_col;
                }
            }
        }
        
        return {best_row, best_col};
    }
    
    // Update enemy tracking
    void update_enemy_info(const std::vector<RadarObj>& radar_results) {
        current_round++;
        
        // Age known enemies
        std::set<Enemy> aged_enemies;
        for (const auto& enemy : known_enemies) {
            Enemy aged = enemy;
            aged.last_seen_round++;
            if (aged.last_seen_round < 5) {  // Forget enemies not seen for 5 rounds
                aged_enemies.insert(aged);
            }
        }
        known_enemies = aged_enemies;
        
        // Process new radar data
        for (const auto& obj : radar_results) {
            if (obj.m_type == 'R') {  // Live enemy robot
                Enemy new_enemy{obj.m_row, obj.m_col, 0, 100};  // Assume full health initially
                
                // Check if we already know this enemy
                auto it = known_enemies.find(new_enemy);
                if (it != known_enemies.end()) {
                    // Update position
                    known_enemies.erase(it);
                }
                known_enemies.insert(new_enemy);
                
                // Set as target if no current target
                if (!has_target) {
                    has_target = true;
                    target_row = obj.m_row;
                    target_col = obj.m_col;
                }
            }
            else if (obj.m_type == 'X') {  // Dead robot
                // Remove from tracking
                Enemy dead{obj.m_row, obj.m_col, 0, 0};
                known_enemies.erase(dead);
                
                // Clear target if this was our target
                if (has_target && target_row == obj.m_row && target_col == obj.m_col) {
                    has_target = false;
                }
            }
            else if (obj.m_type == 'M' || obj.m_type == 'P' || obj.m_type == 'F') {
                obstacles.insert({obj.m_row, obj.m_col});
            }
        }
        
        // Update visited cells
        int my_row, my_col;
        get_my_location(my_row, my_col);
        visited_cells.insert({my_row, my_col});
        
        // Update grenade avoidance
        if (grenade_avoidance_timer > 0) grenade_avoidance_timer--;
    }

public:
    DylanBot() : RobotBase(4, 3, railgun)  // Fast with moderate armor
    {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        current_round = 0;
        last_fire_round = -5;
        consecutive_misses = 0;
        has_target = false;
        target_row = target_col = -1;
        suspected_grenade_bot = false;
        grenade_avoidance_timer = 0;
    }
    
    virtual void get_radar_direction(int& radar_direction_out) override 
    {
        int my_row, my_col;
        get_my_location(my_row, my_col);
        
        // Smart radar scanning strategy
        if (has_target) {
            // Track current target
            int dr = target_row - my_row;
            int dc = target_col - my_col;
            
            // Determine direction (1-8) based on vector
            if (dr < 0 && dc == 0) radar_direction_out = 1; // Up
            else if (dr < 0 && dc > 0) radar_direction_out = 2; // Up-Right
            else if (dr == 0 && dc > 0) radar_direction_out = 3; // Right
            else if (dr > 0 && dc > 0) radar_direction_out = 4; // Down-Right
            else if (dr > 0 && dc == 0) radar_direction_out = 5; // Down
            else if (dr > 0 && dc < 0) radar_direction_out = 6; // Down-Left
            else if (dr == 0 && dc < 0) radar_direction_out = 7; // Left
            else radar_direction_out = 8; // Up-Left
        } 
        else if (!known_enemies.empty()) {
            // Scan toward last known enemy position
            const Enemy& last_enemy = *known_enemies.begin();
            int dr = last_enemy.row - my_row;
            int dc = last_enemy.col - my_col;
            
            if (dr < 0 && dc == 0) radar_direction_out = 1;
            else if (dr < 0 && dc > 0) radar_direction_out = 2;
            else if (dr == 0 && dc > 0) radar_direction_out = 3;
            else if (dr > 0 && dc > 0) radar_direction_out = 4;
            else if (dr > 0 && dc == 0) radar_direction_out = 5;
            else if (dr > 0 && dc < 0) radar_direction_out = 6;
            else if (dr == 0 && dc < 0) radar_direction_out = 7;
            else radar_direction_out = 8;
        }
        else {
            // Systematic exploration pattern
            static int scan_pattern = 1;
            radar_direction_out = scan_pattern;
            scan_pattern = (scan_pattern % 8) + 1;  // Cycle through 1-8
        }
    }
    
    virtual void process_radar_results(const std::vector<RadarObj>& radar_results) override 
    {
        // Update internal state with radar data
        update_enemy_info(radar_results);
        
        // Check for potential grenade bots (multiple enemies close together)
        if (radar_results.size() >= 3) {
            int robot_count = 0;
            for (const auto& obj : radar_results) {
                if (obj.m_type == 'R') robot_count++;
            }
            if (robot_count >= 2) {
                suspected_grenade_bot = true;
                grenade_avoidance_timer = 3;
            }
        }
        
        // Update target selection
        if (!known_enemies.empty()) {
            // Choose closest enemy as target
            int my_row, my_col;
            get_my_location(my_row, my_col);
            
            int min_dist = std::numeric_limits<int>::max();
            for (const auto& enemy : known_enemies) {
                int dist = manhattan_distance(my_row, my_col, enemy.row, enemy.col);
                if (dist < min_dist) {
                    min_dist = dist;
                    target_row = enemy.row;
                    target_col = enemy.col;
                    has_target = true;
                }
            }
        } else {
            has_target = false;
        }
    }
    
    virtual bool get_shot_location(int& shot_row, int& shot_col) override 
    {
        // Check if we should shoot
        if (!has_target) return false;
        
        int my_row, my_col;
        get_my_location(my_row, my_col);
        
        // Check distance to target
        int distance = manhattan_distance(my_row, my_col, target_row, target_col);
        
        // Railgun specific logic
        if (get_weapon() == railgun) {
            // Railgun can shoot any distance, but we prefer optimal range
            if (distance < MIN_RAILGUN_DISTANCE) {
                // Too close - don't shoot, focus on moving away
                return false;
            }
            
            // Don't shoot too frequently (cooldown)
            if (current_round - last_fire_round < 2) {
                return false;
            }
            
            // Predict target movement
            int predicted_row, predicted_col;
            predict_enemy_position(target_row, target_col, predicted_row, predicted_col);
            
            // Adjust prediction based on consecutive misses
            if (consecutive_misses > 0) {
                // Try different prediction
                predicted_row = target_row + ((std::rand() % 3) - 1);
                predicted_col = target_col + ((std::rand() % 3) - 1);
            }
            
            // Ensure valid cell
            if (is_valid_cell(predicted_row, predicted_col)) {
                shot_row = predicted_row;
                shot_col = predicted_col;
                last_fire_round = current_round;
                return true;
            }
        }
        
        return false;
    }
    
    virtual void get_move_direction(int& move_direction, int& move_distance) override 
    {
        int my_row, my_col;
        get_my_location(my_row, my_col);
        
        // Check if movement is disabled (fell in pit)
        int move_speed = get_move_speed();
        if (move_speed <= 0) {
            move_direction = 0;
            move_distance = 0;
            return;
        }
        
        // Find best position to move to
        auto [best_row, best_col] = find_best_move(my_row, my_col);
        
        // If we're staying put
        if (best_row == my_row && best_col == my_col) {
            move_direction = 0;
            move_distance = 0;
            return;
        }
        
        // Calculate direction vector
        int dr = best_row - my_row;
        int dc = best_col - my_col;
        
        // Normalize to single step for direction calculation
        int step_dr = (dr > 0) ? 1 : (dr < 0) ? -1 : 0;
        int step_dc = (dc > 0) ? 1 : (dc < 0) ? -1 : 0;
        
        // Convert to direction (1-8)
        if (step_dr < 0 && step_dc == 0) move_direction = 1; // Up
        else if (step_dr < 0 && step_dc > 0) move_direction = 2; // Up-Right
        else if (step_dr == 0 && step_dc > 0) move_direction = 3; // Right
        else if (step_dr > 0 && step_dc > 0) move_direction = 4; // Down-Right
        else if (step_dr > 0 && step_dc == 0) move_direction = 5; // Down
        else if (step_dr > 0 && step_dc < 0) move_direction = 6; // Down-Left
        else if (step_dr == 0 && step_dc < 0) move_direction = 7; // Left
        else move_direction = 8; // Up-Left
        
        // Calculate distance (cap at move speed)
        move_distance = std::min(move_speed, std::max(std::abs(dr), std::abs(dc)));
        
        // Adjust for obstacles in path
        int actual_distance = 0;
        for (int d = 1; d <= move_distance; d++) {
            int test_row = my_row + (step_dr * d);
            int test_col = my_col + (step_dc * d);
            
            if (!is_valid_cell(test_row, test_col) || !is_passable(test_row, test_col)) {
                break;
            }
            actual_distance = d;
        }
        
        move_distance = actual_distance;
        
        // Special case: avoid suspected grenade bots
        if (suspected_grenade_bot && grenade_avoidance_timer > 0) {
            // Move away from cluster
            if (known_enemies.size() >= 2) {
                int avg_enemy_row = 0, avg_enemy_col = 0;
                for (const auto& enemy : known_enemies) {
                    avg_enemy_row += enemy.row;
                    avg_enemy_col += enemy.col;
                }
                avg_enemy_row /= known_enemies.size();
                avg_enemy_col /= known_enemies.size();
                
                // Move opposite direction
                int flee_dr = my_row - avg_enemy_row;
                int flee_dc = my_col - avg_enemy_col;
                
                if (flee_dr != 0 || flee_dc != 0) {
                    flee_dr = (flee_dr > 0) ? 1 : (flee_dr < 0) ? -1 : 0;
                    flee_dc = (flee_dc > 0) ? 1 : (flee_dc < 0) ? -1 : 0;
                    
                    // Convert to direction
                    if (flee_dr < 0 && flee_dc == 0) move_direction = 1;
                    else if (flee_dr < 0 && flee_dc > 0) move_direction = 2;
                    else if (flee_dr == 0 && flee_dc > 0) move_direction = 3;
                    else if (flee_dr > 0 && flee_dc > 0) move_direction = 4;
                    else if (flee_dr > 0 && flee_dc == 0) move_direction = 5;
                    else if (flee_dr > 0 && flee_dc < 0) move_direction = 6;
                    else if (flee_dr == 0 && flee_dc < 0) move_direction = 7;
                    else move_direction = 8;
                    
                    move_distance = std::min(2, move_speed); // Quick escape
                }
            }
        }
    }
};

// Factory function
extern "C" RobotBase* create_robot() 
{
    return new DylanBot();
}