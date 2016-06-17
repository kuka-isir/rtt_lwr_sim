// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/rtt_lwr_sim.hpp>
#include <rtt_ros_kdl_tools/mqueue_connpolicy.hpp>
#include <Eigen/Dense>
#include <ros/service.h>

using namespace lwr;
using namespace KDL;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void LWRSim::WorldUpdateBegin()
{
    if(!is_configured && !isRunning()) return;

    for(unsigned j=0; j<joints_idx_.size(); j++) {
        jnt_pos_[j] = gazebo_joints_[joints_idx_[j]]->GetAngle(0).Radian();
        jnt_vel_[j] = gazebo_joints_[joints_idx_[j]]->GetVelocity(0);
        jnt_trq_[j] = gazebo_joints_[joints_idx_[j]]->GetForce(0u);
    }



    // Do set pos if asked
    if(set_joint_pos_no_dynamics_)
    {
        for(unsigned j=0; j<joints_idx_.size(); j++)
#ifdef GAZEBO_GREATER_6
            gazebo_joints_[joints_idx_[j]]->SetPosition(0,jnt_pos_no_dyn_[j]);
#else
            gazebo_joints_[joints_idx_[j]]->SetAngle(0,jnt_pos_no_dyn_[j]);
#endif
        // Set jnt pos
        jnt_pos_cmd_ = jnt_pos_ = jnt_pos_no_dyn_;
        set_joint_pos_no_dynamics_ = false;
    }

    stepInternalModel();

}

void LWRSim::WorldUpdateEnd()
{
    if(!is_configured && !isRunning()) return;

    // Read From gazebo simulation
    for(unsigned j=0; j<joints_idx_.size(); j++) {
        jnt_pos_[j] = gazebo_joints_[joints_idx_[j]]->GetAngle(0).Radian();
        jnt_vel_[j] = gazebo_joints_[joints_idx_[j]]->GetVelocity(0);
        jnt_trq_[j] = gazebo_joints_[joints_idx_[j]]->GetForce(0u);
    }

    // If user is connected, let's write command to gazebo
    if(this->hasReceivedAtLeastOneCommand())
    {

        for(unsigned j=0; j<joints_idx_.size(); j++)
            gazebo_joints_[joints_idx_[j]]->SetForce(0,jnt_trq_gazebo_cmd_[j]);
    }
    else
    {
        // If not, try to keep the same position TODO: find out why it drifts
        for(auto joint : gazebo_joints_)
#ifdef GAZEBO_GREATER_6
              joint->SetPosition(0,joint->GetAngle(0).Radian());
#else
              joint->SetAngle(0,joint->GetAngle(0).Radian());
#endif
    }
}
