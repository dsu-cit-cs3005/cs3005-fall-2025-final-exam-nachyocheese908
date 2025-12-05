#include <unordered_map>
#include "RobotBase.h"
class DeathTracker{
    private:
        std::unordered_map<RobotBase*, int> death_rounds;
    
    public:
        DeathTracker();
        ~DeathTracker() = default;
        void recordDeath(RobotBase* robot, int round);
        int getDeathRound(RobotBase* robot) const;
};