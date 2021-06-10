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
    void copyRobotData(RobotData &rd_l);

    RobotData &rd_;
    RobotData rd_cc_;

    //WholebodyController &wbc_;
    //TaskCommand tc;

private:
    Eigen::VectorQd ControlVal_;
};
