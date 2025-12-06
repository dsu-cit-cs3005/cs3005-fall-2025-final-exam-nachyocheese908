#include <unordered_map>
#include "RobotBase.h"
class DeathTracker{
    private:
        std::unordered_map<RobotBase*, int> death_rounds;
    
    public:
        std::vector<RobotBase*> deaths_for_summmary;
        DeathTracker();
        ~DeathTracker() = default;
        void recordDeath(RobotBase* robot, int round);
        int getDeathRound(RobotBase* robot) const;
};