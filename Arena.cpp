#include "Arena.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <dlfcn.h>      // For dlopen, dlsym, dlclose
#include <dirent.h>     // For reading directory
#include <sys/stat.h>   // For file stat
#include <cstring>      // For strstr
#include <algorithm>
#include <set>
#include <utility>
#include <limits>
#include <functional>

std::string weaponToString(WeaponType w) {
    switch(w) {
        case flamethrower: return "flamethrower";
        case railgun:      return "railgun";
        case grenade:      return "grenade";
        case hammer:       return "hammer";
        default:           return "unknown";
    }
}

Arena:: Arena(): rows(20), cols(20), current_round(1){
    std::srand(std::time(nullptr));
    std::cout << "Arena created: " << rows << "x" << cols << "\n";

    board.resize(rows, std::vector<char>(cols, '.'));
}

Arena::~Arena(){
    printGameSummary();
    for (auto robot:robots){
        delete robot;
    }
    for (void* handle : library_handles){
        if (handle){
            dlclose(handle);
        }
    }
    std::cout << "Arena destroyed\n";
    
    
}


void Arena::run_sim(bool live){
    std::cout << "\n============ RobotWarz ============\n";
    is_live = live;
    if (live){
        std::cout << "LIVE MODE: Press ENTER after each round\n";
    }
    loadRobots();
    if (robots.empty()){
        std::cout << "No robots! Exiting...\n";
        return;
    }
    placeRobots();
        
    placeObstacles();

    clearScreen();

        while(!checkWinner() && current_round < 10000){

            int alive_count = 0;
            for (auto robot : robots) {
                if (robot->get_health() > 0) {
                    alive_count++;
                }
            }
            
            // ========== CHANGED: Three end conditions ==========
            // 1. Only 0 or 1 robot alive (game ends naturally)
            // 2. Max rounds reached with multiple alive (DRAW)
            if (alive_count <= 1 || current_round >= 9999) {
                std::cout << "\n=========== FINAL ROUND " << current_round << " ===========\n";}
        
        for (auto robot: robots){
            if (robot->get_health() <= 0 && death_registry.getDeathRound(robot) == -1){
                death_registry.recordDeath(robot, current_round);
                death_registry.deaths_for_summmary.push_back(robot);
            }

            if (robot->get_health() <= 0){
                int death_round = death_registry.getDeathRound(robot);
                std::cout << robot->m_name << " " << robot->m_character << " is dead (Died round: " 
                << death_round << "). Skipping turn.\n";
                continue;
            }
            
            std::cout << "\n" << robot->m_name << " " << robot->m_character 
                      << " begins turn.\n";
            int r_row, r_col;
            robot->get_current_location(r_row, r_col);
            std::cout << "  Position: (" << r_row << ", " << r_col << ")\n";
            std::cout << "  Health: " << robot->get_health() 
                      << ", Armor: " << robot->get_armor() << "\n";

            // STEP 1: Radar
            int radar_direction;
            robot->get_radar_direction(radar_direction);
            //std::cout << "  Radar direction: " << radar_direction << "\n";
            
            std::vector<RadarObj> radar_results = scanRadar(robot, radar_direction);

            //std::cout << "  Found " << radar_results.size() << " object(s):\n";
            //for (const auto& obj : radar_results) {
            //    std::cout << "    - Type '" << obj.m_type << "' at (" 
            //              << obj.m_row << ", " << obj.m_col << ")\n";
            //}

            robot->process_radar_results(radar_results);
            
            // STEP 2: Ask if robot wants to shoot
            int shot_row, shot_col;
            bool wants_to_shoot = robot->get_shot_location(shot_row, shot_col);
            
            if (wants_to_shoot) {
                std::cout << "  DECIDED TO SHOOT at (" << shot_row << "," << shot_col << ")\n";
                handleShooting(robot, shot_row, shot_col);
            } else {
                std::cout << "  DECIDED NOT TO SHOOT\n";
                // STEP 3: Ask for movement
                int move_direction, move_distance;
                robot->get_move_direction(move_direction, move_distance);
                std::cout << "  Wants to move: direction " << move_direction 
                          << ", distance " << move_distance << "\n";
                // TODO: Process movement later
                handleMovement(robot, move_direction, move_distance);
            }
        }
        printBoard();

        if (is_live){
            std::cout << "Press ENTER to continue to the next round...\n";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            clearScreen();
        }

        current_round++;
    }
}

void Arena::loadRobots() {
    std::cout << "Loading robots...\n";
    
    // List of robots to try loading
    std::vector<std::string> robot_files = {"Robot_Flame_e_o.cpp", "Robot_Testbot.cpp", "Robot_Flame_e_o.cpp", "Robot_Testbot.cpp"};
    
    int loaded_count = 0;
    for (const auto& filename : robot_files) {
        if (loadSingleRobot(filename)) {
            loaded_count++;
        }
    }
    
    std::cout << "Successfully loaded " << loaded_count << " robot(s).\n";
    
    if (robots.empty()) {
        std::cout << "Warning: No robots loaded!\n";
        return;
    }
    
    // Assign display characters
    char display_chars[] = {'!', '@', '#', '$', '%', '^', '&', '*'};
    for (size_t i = 0; i < robots.size() && i < sizeof(display_chars); i++) {
        robots[i]->m_character = display_chars[i];
        std::cout << "  " << robots[i]->m_name << " assigned character '" 
                  << display_chars[i] << "'\n";
    }
}

bool Arena::loadSingleRobot(const std::string& filename){
    std::cout << "Attempting to load: " << filename << "\n";

    struct stat buffer;
    if (stat(filename.c_str(), &buffer) != 0){
        std::cout << "FILE NOT FOUND: " << filename << "\n";
        return false;
    }

    std::string robot_name = filename;

    if (robot_name.find("Robot_") == 0){
        robot_name = robot_name.substr(6);
    }
    size_t dot_pos = robot_name.find(".cpp");
    if (dot_pos != std::string::npos){
        robot_name = robot_name.substr(0, dot_pos);
    }

    std::string shared_lib = "lib" + robot_name + ".so";
    std::string compile_cmd = "g++ -shared -fPIC -o " + shared_lib + 
                              " " + filename + " RobotBase.o -I. -std=c++20";
    std::cout << "  Compiling: " << compile_cmd << "\n";
    
    int compile_result = std::system(compile_cmd.c_str());
    if (compile_result != 0) {
        std::cerr << "  Failed to compile " << filename << "\n";
        return false;
    }
    
    // Step 2: Load the shared library
    void* handle = dlopen(("./" + shared_lib).c_str(), RTLD_LAZY);
    if (!handle) {
        std::cerr << "  Failed to load " << shared_lib << ": " << dlerror() << "\n";
        return false;
    }
    
    // Step 3: Find the factory function
    RobotFactory create_robot = (RobotFactory)dlsym(handle, "create_robot");
    if (!create_robot) {
        std::cerr << "  Failed to find create_robot in " << shared_lib << ": " << dlerror() << "\n";
        dlclose(handle);
        return false;
    }
    
    // Step 4: Create the robot instance
    RobotBase* robot = create_robot();
    if (!robot) {
        std::cerr << "  Factory function returned null for " << robot_name << "\n";
        dlclose(handle);
        return false;
    }
    
    // Step 5: Set up the robot
    robot->m_name = robot_name;
    robot->set_boundaries(rows, cols);
    
    // Store the robot and library handle
    robots.push_back(robot);
    library_handles.push_back(handle);
    
    std::cout << "  Successfully loaded robot: " << robot_name << "\n";
    std::cout << "    Health: " << robot->get_health() 
              << ", Weapon: " << robot->get_weapon()
              << ", Armor: " << robot->get_armor()
              << ", Move: " << robot->get_move_speed() << "\n";
    
    return true;
}

void Arena::placeRobots() {
    std::cout << "Placing robots on board...\n";
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            board[i][j] = '.';
        }
    }
    
    for (size_t i = 0; i < robots.size(); i++) {
        int max_attempts = rows * cols * 2;  // Try many times
        bool placed = false;
        
        for (int attempt = 0; attempt < max_attempts && !placed; attempt++) {
            int row = rand() % rows;
            int col = rand() % cols;
            
            // Check if cell is empty
            if (board[row][col] == '.') {
                // Place robot
                robots[i]->move_to(row, col);
                board[row][col] = robots[i]->m_character;  // Use robot's display char
                placed = true;
                
                std::cout << "  " << robots[i]->m_name << " (" 
                          << robots[i]->m_character << " at (" 
                          << row << ", " << col << ")\n";
            }
        }
        
        if (!placed) {
            std::cerr << "  ERROR: Could not place " << robots[i]->m_name << "!\n";
            // Try to find any empty spot
            for (int r = 0; r < rows && !placed; r++) {
                for (int c = 0; c < cols && !placed; c++) {
                    if (board[r][c] == '.') {
                        robots[i]->move_to(r, c);
                        board[r][c] = robots[i]->m_character;
                        placed = true;
                        std::cout << "  " << robots[i]->m_name << " at (" 
                                  << r << ", " << c << ") (fallback)\n";
                    }
                }
            }
            
            if (!placed) {
                std::cerr << "  FATAL: No empty cells on board!\n";
                exit(1);
            }
        }
    }
    
}

void Arena::placeObstacles(){
    for (int r = 0; r < rows; r++){
        for (int c = 0; c < cols; c++){
            char cell = board[r][c];
            if (cell == 'M' || cell == 'P' || cell == 'F'){
                board[r][c] = '.';
            }
        }
    }

    int cells = rows*cols;
    int min_obstacles = cells / 10;
    int max_obstacles = cells / 5;
    int total_obstacles = min_obstacles + (rand() % (max_obstacles - min_obstacles + 1));

    std::cout << "Adding " << total_obstacles << " random obstacles\n";

    std::vector<std::pair<char,int>> obstacle_chance = {{'M',20},{'F',10},{'P',5}};

    int obstacles_placed = 0;
    int attempts = 0;
    const int max = 1000;
    while (obstacles_placed < total_obstacles && attempts < max){
        int row = rand() % rows;
        int col = rand() % cols;

        if (board[row][col] == '.'){
            int total_weight = 0;
            for (auto& [type, weight] : obstacle_chance){
                total_weight += weight;
            }
            int random_weight = rand() % total_weight;
            char chosen_obstacle = 'M';

            for (auto& [type, weight] : obstacle_chance){
                if (random_weight < weight){
                    chosen_obstacle = type;
                    break;
                }
                random_weight -= weight;
            }

            board[row][col] = chosen_obstacle;
            obstacles_placed++;
            attempts++;

        }


    } 

}

// ============ RADAR IMPLEMENTATION ============

// Helper: Convert board display character to radar type
char Arena::convertToRadarType(char display_char) const {
    // Robot characters (live robots)
    if (display_char == '!' || display_char == '@' || 
        display_char == '#' || display_char == '$' ||
        display_char == '%' || display_char == '^' ||
        display_char == '&' || display_char == '*') {
        return 'R'; // Live robot
    }
    
    // Dead robot
    if (display_char == 'X') {
        return 'X'; // Dead robot
    }
    
    // Obstacles or empty - return the char itself
    return display_char; // 'M', 'P', 'F', or '.'
}

// Helper: Scan a single cell and add to results if not empty
void Arena::scanCell(int row, int col, std::vector<RadarObj>& results) const {
    // Check bounds
    if (row < 0 || row >= rows || col < 0 || col >= cols) {
        return;
    }
    
    char cell_type = board[row][col];
    if (cell_type != '.') {
        char radar_type = convertToRadarType(cell_type);
        results.push_back(RadarObj(radar_type, row, col));
    }
}

// Simple radar scanning (1-cell wide for now)
std::vector<RadarObj> Arena::scanRadar(RobotBase* robot, int direction) {
    std::vector<RadarObj> results;
    
    // Get robot's current position
    int robot_row, robot_col;
    robot->get_current_location(robot_row, robot_col);

    //std::cout << "  [RADAR DEBUG] Scanning from (" << robot_row << "," << robot_col 
    //         << ") direction " << direction << "\n";
    
    
    // Direction 0: Check 8 surrounding cells (3x3 minus center)
    if (direction == 0) {
        for (int dr = -1; dr <= 1; dr++) {
            for (int dc = -1; dc <= 1; dc++) {
                if (dr == 0 && dc == 0) continue; // Skip robot itself
                scanCell(robot_row + dr, robot_col + dc, results);
            }
        }
        return results;
    }
    
    // Directions 1-8: 3-cell wide scanning
    if (direction < 1 || direction > 8) {
        return results; // Invalid direction
    }
    
    // Get direction vector
    int dir_row = directions[direction].first;
    int dir_col = directions[direction].second;
    
    // Determine side vectors for 3-wide scanning
    // For orthogonal directions (1,3,5,7): side vectors are perpendicular
    // For diagonal directions (2,4,6,8): side vectors are at 45 degrees
    
    int side1_row, side1_col, side2_row, side2_col;
    
    if (direction == 1 || direction == 5) { // Up or Down (vertical)
        side1_row = 0; side1_col = -1;  // Left
        side2_row = 0; side2_col = 1;   // Right
    }
    else if (direction == 3 || direction == 7) { // Right or Left (horizontal)
        side1_row = -1; side1_col = 0;  // Up
        side2_row = 1; side2_col = 0;   // Down
    }
    else if (direction == 2) { // Up-right
        side1_row = -1; side1_col = 0;  // Up
        side2_row = 0; side2_col = 1;   // Right
    }
    else if (direction == 4) { // Down-right
        side1_row = 0; side1_col = 1;   // Right
        side2_row = 1; side2_col = 0;   // Down
    }
    else if (direction == 6) { // Down-left
        side1_row = 1; side1_col = 0;   // Down
        side2_row = 0; side2_col = -1;  // Left
    }
    else if (direction == 8) { // Up-left
        side1_row = 0; side1_col = -1;  // Left
        side2_row = -1; side2_col = 0;  // Up
    }
    
    // Scan along the line to edge of board
    int distance = 0;
    bool scanning = true;
    
    while (scanning) {
        distance++;
        
        // Check 3 cells across at this distance
        for (int offset = -1; offset <= 1; offset++) {
            int scan_row, scan_col;
            
            if (offset == -1) {
                // Left/up side cell
                scan_row = robot_row + (dir_row * distance) + (side1_row * distance);
                scan_col = robot_col + (dir_col * distance) + (side1_col * distance);
            }
            else if (offset == 0) {
                // Center cell
                scan_row = robot_row + (dir_row * distance);
                scan_col = robot_col + (dir_col * distance);
            }
            else { // offset == 1
                // Right/down side cell
                scan_row = robot_row + (dir_row * distance) + (side2_row * distance);
                scan_col = robot_col + (dir_col * distance) + (side2_col * distance);
            }
            
            // Check bounds
            if (scan_row >= 0 && scan_row < rows && scan_col >= 0 && scan_col < cols) {
                scanCell(scan_row, scan_col, results);
            } else {
                // If center cell is out of bounds, stop scanning
                if (offset == 0) {
                    scanning = false;
                }
            }
        }
        
        // Stop if we've gone far enough (center cell out of bounds)
        int center_row = robot_row + (dir_row * distance);
        int center_col = robot_col + (dir_col * distance);
        if (center_row < 0 || center_row >= rows || center_col < 0 || center_col >= cols) {
            scanning = false;
        }
    }
    //std::cout << "  [RADAR DEBUG] Total objects found: " << results.size() << "\n";
    return results;
}

void Arena::handleShooting(RobotBase* shooter, int target_row, int target_col) {
    std::cout << "  [SHOOTING] ";
    int shooter_row, shooter_col;
    shooter->get_current_location(shooter_row, shooter_col);
    std::cout << shooter->m_name << " at (" << shooter_row << "," << shooter_col << ") ";
    std::cout << "shooting at (" << target_row << "," << target_col << ")\n";
    
    // Get shooter's weapon type
    WeaponType weapon = shooter->get_weapon();
    std::cout << "  [WEAPON] " << weaponToString(weapon) << "\n";
    
    std::vector<std::pair<int, int>> affected_cells;
    
    // Calculate affected cells based on weapon type
    switch(weapon) {
        case railgun: {
            //std::cout << "  [RAILGUN] Firing through entire line\n";
            // Railgun goes through everything to edge of arena
            // Calculate direction vector from shooter to target
            int dr = target_row - shooter_row;
            int dc = target_col - shooter_col;
            
            // Normalize direction (make dr/dc -1, 0, or 1)
            if (dr != 0) dr = dr > 0 ? 1 : -1;
            if (dc != 0) dc = dc > 0 ? 1 : -1;
            
            // Start from shooter position, move in direction to edge
            int current_row = shooter_row + dr;
            int current_col = shooter_col + dc;
            
            while (current_row >= 0 && current_row < rows && 
                   current_col >= 0 && current_col < cols) {
                affected_cells.push_back({current_row, current_col});
                current_row += dr;
                current_col += dc;
            }
            break;
        }
        
        case flamethrower: {
            //std::cout << "  [FLAMETHROWER] 3x4 area\n";
            // Flamethrower: 3 cells wide, 4 cells long from shooter
            // Determine direction from shooter to target
            int dr = target_row - shooter_row;
            int dc = target_col - shooter_col;
            
            // Normalize direction
            if (dr != 0) dr = dr > 0 ? 1 : -1;
            if (dc != 0) dc = dc > 0 ? 1 : -1;
            
            // For 4 cells in length
            for (int length = 1; length <= 4; length++) {
                int center_row = shooter_row + (dr * length);
                int center_col = shooter_col + (dc * length);
                
                // Check bounds
                if (center_row < 0 || center_row >= rows || 
                    center_col < 0 || center_col >= cols) {
                    continue;
                }
                
                // For 3 cells wide
                if (dr == 0) { // Horizontal shot
                    for (int offset = -1; offset <= 1; offset++) {
                        affected_cells.push_back({center_row, center_col + offset});
                    }
                } else if (dc == 0) { // Vertical shot
                    for (int offset = -1; offset <= 1; offset++) {
                        affected_cells.push_back({center_row + offset, center_col});
                    }
                } else { // Diagonal shot
                    affected_cells.push_back({center_row, center_col});
                    affected_cells.push_back({center_row - dr, center_col});
                    affected_cells.push_back({center_row, center_col - dc});
                }
            }
            break;
        }
        
        case hammer: {
            //std::cout << "  [HAMMER] Adjacent cell only\n";
            // Hammer: just the target cell (must be adjacent)
            affected_cells.push_back({target_row, target_col});
            break;
        }
        
        case grenade: {
            //std::cout << "  [GRENADE] 3x3 area at target\n";
            // Grenade: 3x3 area at target location
            for (int dr = -1; dr <= 1; dr++) {
                for (int dc = -1; dc <= 1; dc++) {
                    affected_cells.push_back({target_row + dr, target_col + dc});
                }
            }
            // Check grenade ammo
            if (shooter->get_grenades() <= 0) {
                //std::cout << "  [AMMO] No grenades left!\n";
                return;
            }
            break;
        }
    }
    
    // Remove duplicates and out-of-bounds cells
    std::sort(affected_cells.begin(), affected_cells.end());
    affected_cells.erase(std::unique(affected_cells.begin(), affected_cells.end()), 
                         affected_cells.end());
    
    // Apply damage to all affected robots
    std::set<RobotBase*> hit_robots; // Track robots already hit
    
    for (const auto& cell : affected_cells) {
        int row = cell.first;
        int col = cell.second;
        
        // Check bounds
        if (row < 0 || row >= rows || col < 0 || col >= cols) {
            continue;
        }
        
        // Find robot at this cell
        for (auto robot : robots) {
            if (robot == shooter) continue; // Skip shooter
            
            int robot_row, robot_col;
            robot->get_current_location(robot_row, robot_col);
            
            if (robot_row == row && robot_col == col) {
                // Check if we already hit this robot (for area weapons)
                if (hit_robots.find(robot) != hit_robots.end()) {
                    continue;
                }
                
                hit_robots.insert(robot);
                
                std::cout << "  [HIT] " << robot->m_name << " " << robot->m_character 
                          << " at (" << row << ", " << col << ")!\n";
                
                // Calculate damage based on weapon
                int damage = 0;
                switch(weapon) {
                    case flamethrower: damage = 30 + (rand() % 21); break;  // 30-50
                    case railgun:      damage = 10 + (rand() % 11); break;  // 10-20  
                    case hammer:       damage = 50 + (rand() % 11); break;  // 50-60
                    case grenade:      damage = 10 + (rand() % 31); break;  // 10-40
                }
                
                // Apply armor reduction (10% per armor level)
                int armor = robot->get_armor();
                double reduction = 1.0 - (armor * 0.1);
                int final_damage = static_cast<int>(damage * reduction);
                
                std::cout << "  [DAMAGE] Base: " << damage 
                          << ", Armor: " << armor << " (reduces by " << (armor * 10) << "%)"
                          << ", Final damage: " << final_damage << "\n";
                
                // Apply damage
                int remaining_health = robot->take_damage(final_damage);
                
                // Reduce armor by 1 (as per spec)
                robot->reduce_armor(1);
                
                std::cout << "  [RESULT] " << robot->m_name 
                          << " now has " << remaining_health << " health and " 
                          << robot->get_armor() << " armor\n";
                
                // Check if robot died
                if (remaining_health <= 0) {
                    std::cout << "  ☠️ " << robot->m_name << " HAS BEEN DESTROYED!\n";
                    // Mark as dead on board (change character to 'X')
                    board[row][col] = 'X';
                    death_registry.recordDeath(robot, current_round);
                    death_registry.deaths_for_summmary.push_back(robot);
                }
            }
        }
    }
    
    // Handle grenade ammo
    if (weapon == grenade) {
        shooter->decrement_grenades();
        std::cout << "  [AMMO] " << shooter->m_name << " has " 
                  << shooter->get_grenades() << " grenades remaining\n";
    }
    
    if (hit_robots.empty()) {
        std::cout << "  [MISS] No robots hit\n";
    }
}

void Arena::handleMovement(RobotBase* robot, int direction, int distance) {
    if (distance <= 0 || direction < 0 || direction > 8) {
        //std::cout << "  [MOVEMENT] Invalid movement parameters\n";
        return;
    }
    
    // Get current position
    int current_row, current_col;
    robot->get_current_location(current_row, current_col);
    
    //std::cout << "  [MOVEMENT] Robot at (" << current_row << "," << current_col 
    //          << ") wants to move direction " << direction << " for " << distance << " cells\n";
    
    // Cap distance at robot's move speed
    int max_move = robot->get_move_speed();
    if (distance > max_move) {
        distance = max_move;
        //std::cout << "  [MOVEMENT] Capped distance to " << max_move << " (robot's max speed)\n";
    }
    
    // Get direction vector
    if (direction == 0 || direction > 8) {
        //std::cout << "  [MOVEMENT] Invalid direction\n";
        return;
    }
    
    int dir_row = directions[direction].first;
    int dir_col = directions[direction].second;
    
    int new_row = current_row;
    int new_col = current_col;
    int steps_taken = 0;
    
    // Move step by step
    for (int step = 0; step < distance; step++) {
        int next_row = new_row + dir_row;
        int next_col = new_col + dir_col;
        
        // Check bounds
        if (next_row < 0 || next_row >= rows || next_col < 0 || next_col >= cols) {
            //std::cout << "  [MOVEMENT] Hit boundary at step " << (step + 1) << "\n";
            break;
        }
        
        // Check what's in the next cell
        char next_cell = board[next_row][next_col];
        
        // Check for collisions
        if (next_cell != '.') {
            // Something is in the way
            //std::cout << "  [MOVEMENT] Collision at (" << next_row << "," << next_col 
            //          << ") with '" << next_cell << "'\n";
            
            // Apply obstacle effect
            if (next_cell == 'M' || next_cell == 'X' || 
                (next_cell >= '!' && next_cell <= '&' && next_cell != robot->m_character)) {
                // Mound, Dead robot, or other live robot: stop movement
                //std::cout << "  [MOVEMENT] Stopped by obstacle/robot\n";
                break;
            }
            else if (next_cell == 'P') {
                // Pit: move into it, then disable movement
                std::cout << "  [MOVEMENT] Fell into pit!\n";
                new_row = next_row;
                new_col = next_col;
                robot->disable_movement();
                steps_taken++;
                break;
            }
            else if (next_cell == 'F') {
                // Flamethrower: move through but take damage
                std::cout << "  [MOVEMENT] Moved through flamethrower, taking damage!\n";
                new_row = next_row;
                new_col = next_col;
                
                // Apply flamethrower damage (30-50)
                int damage = 30 + (rand() % 21); // 30-50
                robot->take_damage(damage);
                std::cout << "  [DAMAGE] Took " << damage << " damage from flamethrower\n";

                if (robot->get_health() <= 0){
                    board[new_row][new_col] = 'X';

                    death_registry.recordDeath(robot, current_round);
                    std::cout << "  ☠️ " << robot->m_name << " DIED FROM FLAMETHROWER!\n";
                }
                
                steps_taken++;
                // Continue moving (flamethrowers don't block)
                continue;
            }
        }
        
        // Clear path, move to next cell
        new_row = next_row;
        new_col = next_col;
        steps_taken++;
    }
    
    // Update board if position changed
    if (new_row != current_row || new_col != current_col) {
        // Clear old position
        board[current_row][current_col] = '.';
        
        // Set new position
        board[new_row][new_col] = robot->m_character;
        
        // Update robot's position
        robot->move_to(new_row, new_col);
        
        //std::cout << "  [MOVEMENT] Moved " << steps_taken << " steps to (" 
        //          << new_row << "," << new_col << ")\n";
    } else {
        std::cout << "  Did not move\n";
    }
}
void Arena::printBoard() const {
    std::cout << "\nCurrent Board (" << rows << "x" << cols << "):\n";
    
    // Print column numbers
    std::cout << "     ";
    for (int col = 0; col < cols; col++) {
        if (col < 10) std::cout << " ";
        std::cout << col << " ";
    }
    std::cout << "\n\n";
    
    // Print each row
    for (int row = 0; row < rows; row++) {
        // Print row number
        if (row < 10) std::cout << " ";
        std::cout << row << "   ";
        
        // Print row contents
        for (int col = 0; col < cols; col++) {
            char cell = board[row][col];
            if (cell == '.' || cell == 'M' || cell == 'P' || cell == 'F') {
                // Obstacles or empty: show with spaces
                std::cout << " " << cell << " ";
            } else if (cell == 'X') {
                // Dead robot
                std::cout << " X" << cell << " ";
            } else {
                // Live robot: show as "R" + character (no space between)
                std::cout << " R" << cell;
            }
        }
        std::cout << "\n";
    }
    
    // Print robot status
    std::cout << "\nRobot Status:\n";
    for (RobotBase* robot : robots) {
        int row, col;
        robot->get_current_location(row, col);
        std::cout << "  " << robot->m_name << " " << robot->m_character 
                  << ": Health=" << robot->get_health() 
                  << ", Pos=(" << row << "," << col << ")\n";
    }
    
}

void Arena:: clearScreen(){
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif
}

bool Arena::checkWinner() const{
    int alive = 0;
    for (auto robot:robots){
        if (robot->get_health() > 0){
            alive++;
        }
    }
    return alive == 1;
}

void Arena::printGameSummary(){
    std::vector<RobotBase*> alive_robots;
    for (auto robot : robots) {
        if (robot->get_health() > 0) {
            alive_robots.push_back(robot);
        }
    }
    
    std::cout << "\n--------------------\n";
    
    // Determine if it's a win or draw
    if (alive_robots.size() == 1) {
        // WIN: One survivor
        std::cout << "WINNER: " << alive_robots[0]->m_name << alive_robots[0]->m_character << "\n";
    } else if (alive_robots.size() > 1) {
        // DRAW: Multiple survivors (max rounds reached)
        std::cout << "DRAW: " << alive_robots.size() << " survivors after " << current_round << " rounds\n";
    } else {
        // DRAW: All died simultaneously (rare)
        std::cout << "DRAW: All robots destroyed!\n";
    }
    
    std::cout << "--------------------\n";
    
    // Get all robots sorted by status
    std::vector<RobotBase*> all_robots = robots;
    
    // Sort: alive first (by health descending), then dead (by death round descending)
    std::sort(all_robots.begin(), all_robots.end(),
        [&](RobotBase* a, RobotBase* b) {
            bool a_alive = a->get_health() > 0;
            bool b_alive = b->get_health() > 0;
            
            // Alive before dead
            if (a_alive && !b_alive) return true;
            if (!a_alive && b_alive) return false;
            
            // Both alive: sort by health (highest first)
            if (a_alive && b_alive) {
                return a->get_health() > b->get_health();
            }
            
            // Both dead: sort by death round (latest death first = better placement)
            int death_a = death_registry.getDeathRound(a);
            int death_b = death_registry.getDeathRound(b);
            return death_a > death_b;
        });
    
    // Print all robots with their status
    int place = 1;
    for (auto robot : all_robots) {
        if (robot->get_health() > 0) {
            // Alive robot
            std::cout << place << ". " << robot->m_name << robot->m_character 
                      << " (survived with " << robot->get_health() << " health)\n";
        } else {
            // Dead robot
            int death_round = death_registry.getDeathRound(robot);
            if (death_round != -1) {
                std::cout << place << ". " << robot->m_name << robot->m_character 
                          << " (died round " << death_round << ")\n";
            } else {
                // Shouldn't happen, but just in case
                std::cout << place << ". " << robot->m_name << robot->m_character 
                          << " (destroyed)\n";
            }
        }
        place++;
    }
}
