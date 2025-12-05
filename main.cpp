#include "Arena.h"
#include <iostream>

int main(int argc, char* argv[]) {
    std::cout << "Starting RobotWarz!\n";
    
    Arena the_game;

    bool live_mode = false;

    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "-l" || arg == "--live") {
            live_mode = true;
        }
    }

    the_game.run_sim(live_mode);
    
    return 0;
}