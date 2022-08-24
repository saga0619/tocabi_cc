#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>


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
    void PublishHapticData();

    void HapticPoseCallback(const geometry_msgs::PoseConstPtr &msg);
    Eigen::Matrix3d Quat2rotmatrix(double q0, double q1, double q2, double q3);
    float PositionMapping( float haptic_pos, int i);

    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;
    ros::Subscriber haptic_pose_sub_;
    ros::Publisher haptic_force_pub_;
    
    Eigen::Vector3d haptic_pos_;
    Eigen::Vector4d haptic_ori_;
    Eigen::Matrix3d haptic_orientation_;

    //WholebodyController &wbc_;
    //TaskCommand tc;

    double haptic_force_[3];

private:
    Eigen::VectorQd ControlVal_;
};
