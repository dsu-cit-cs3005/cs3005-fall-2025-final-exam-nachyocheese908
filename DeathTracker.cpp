#include "DeathTracker.h"

DeathTracker::DeathTracker(){};

void DeathTracker::recordDeath(RobotBase* robot, int round){
    if (death_rounds.find(robot) == death_rounds.end()){
        death_rounds[robot] = round;
    }
}

int DeathTracker::getDeathRound(RobotBase* robot) const {
    auto it = death_rounds.find(robot);
        return it != death_rounds.end() ? it-> second: -1;
}
