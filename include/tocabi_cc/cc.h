#include <tocabi_data/robot_data.h>

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    
    RobotData &rd_;
    //WholebodyController &wbc_;
    //TaskCommand tc;

private:
    Eigen::VectorQd ControlVal_;
};
