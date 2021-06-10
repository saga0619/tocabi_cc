#include "tocabi_cc/cc.h"

using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    ControlVal_.setZero();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

// void CustomController::taskCommandToCC(TaskCommand tc_)
// {
//     tc = tc_;
// }

void CustomController::computeSlow()
{
    copyRobotData(rd_);

    


    // {
    //     if (tc.mode == 10)
    //     {
    //         //rd_.control_time_; current time
    //         //rd_.link_[Right_Foot].Jac : current rightfoot jac
    //         //rd_.q_dot_ : current q velocity

    //         //rd_.link_[Right_Foot]

    //         //ControlVal_=

    //         wbc_.set_contact(rd_, 1, 1);

    //         int task_number = 6;
    //         rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
    //         rd_.f_star.setZero(task_number);

    //         rd_.J_task = rd_.link_[Pelvis].Jac;

    //         if (tc.custom_taskgain)
    //         {
    //             rd_.link_[Pelvis].pos_p_gain = Vector3d::Ones() * tc.pos_p;
    //             rd_.link_[Pelvis].pos_d_gain = Vector3d::Ones() * tc.pos_d;
    //             rd_.link_[Pelvis].rot_p_gain = Vector3d::Ones() * tc.ang_p;
    //             rd_.link_[Pelvis].rot_d_gain = Vector3d::Ones() * tc.ang_d;
    //         }

    //         rd_.link_[Pelvis].x_desired = tc.ratio * rd_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos;
    //         rd_.link_[Pelvis].x_desired(2) = tc.height + tc.ratio * rd_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos(2);
    //         rd_.link_[Pelvis].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

    //         rd_.f_star = wbc_.getfstar6d(rd_, Pelvis);
    //         ControlVal_ = wbc_.task_control_torque_QP(rd_, rd_.J_task, rd_.f_star);
    //     }
    //     else if (tc.mode == 11)
    //     {
    //         wbc_.set_contact(rd_, 1, 1);

    //         int task_number = 6;
    //         rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
    //         rd_.f_star.setZero(task_number);

    //         rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;

    //         if (tc.custom_taskgain)
    //         {
    //             rd_.link_[COM_id].pos_p_gain = Vector3d::Ones() * tc.pos_p;
    //             rd_.link_[COM_id].pos_d_gain = Vector3d::Ones() * tc.pos_d;
    //             rd_.link_[COM_id].rot_p_gain = Vector3d::Ones() * tc.ang_p;
    //             rd_.link_[COM_id].rot_d_gain = Vector3d::Ones() * tc.ang_d;
    //         }

    //         rd_.link_[COM_id].x_desired = tc.ratio * rd_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos;
    //         rd_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * rd_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos(2);
    //         rd_.link_[COM_id].rot_desired = Matrix3d::Identity();
    //         rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
    //         rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, true);
    //         //rd_.link_[COM_id].Set_T
    //         rd_.f_star = wbc_.getfstar6d(rd_, COM_id);

    //         ControlVal_ = wbc_.task_control_torque_QP(rd_, rd_.J_task, rd_.f_star);
    //         //task controller for mode 11 ....
    //     }
    //     else if (tc.mode == 12)
    //     {
    //         wbc_.set_contact(rd_, 1, 1);

    //         int task_number = 9;
    //         rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
    //         rd_.f_star.setZero(task_number);

    //         rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;
    //         rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;

    //         if (tc.custom_taskgain)
    //         {
    //             rd_.link_[COM_id].pos_p_gain = Vector3d::Ones() * tc.pos_p;
    //             rd_.link_[COM_id].pos_d_gain = Vector3d::Ones() * tc.pos_d;
    //             rd_.link_[COM_id].rot_p_gain = Vector3d::Ones() * tc.ang_p;
    //             rd_.link_[COM_id].rot_d_gain = Vector3d::Ones() * tc.ang_d;
    //             rd_.link_[Upper_Body].rot_p_gain = Vector3d::Ones() * tc.ang_p;
    //             rd_.link_[Upper_Body].rot_d_gain = Vector3d::Ones() * tc.ang_d;
    //         }

    //         rd_.link_[COM_id].x_desired = tc.ratio * rd_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos;
    //         rd_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * rd_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos(2);

    //         rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
    //         rd_.link_[COM_id].rot_desired = Matrix3d::Identity();
    //         rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
    //         rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
    //         rd_.link_[Upper_Body].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
    //         //rd_.link_[COM_id].Set_T

    //         rd_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, COM_id);
    //         rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body);

    //         ControlVal_ = wbc_.task_control_torque_QP(rd_, rd_.J_task, rd_.f_star);
    //         //task controller for mode 11 ....
    //     }
}

void CustomController::computeFast()
{
    // if (tc.mode == 10)
    // {
    // }
    // else if (tc.mode == 11)
    // {
    // }
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}