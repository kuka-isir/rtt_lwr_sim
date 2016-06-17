// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/rtt_lwr_sim_remote.hpp>
#include <Eigen/Dense>
#include <ros/service.h>

using namespace lwr;
using namespace KDL;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;
LWRSimRemote::LWRSimRemote(const string& name): LWRCommon(name)
{

}
bool LWRSimRemote::configureHook()
{
    gazebo::client::printVersion();
    gazebo::client::setup(argv);

    gz_node.reset(new gazebo::transport::Node());
    gz_node->Init();
    // Listen to Gazebo world_stats topic
    gz_state_sub = gz_node->Subscribe("~/" + this->getName() + "/joint_states", &LWRSimRemote::gazeboStateCallback,this/*,true*/);
    gz_state_pub = gz_node->Advertise<joint_state_msgs::msgs::JointState>("~/" + this->getName() + "/joint_states_command");
    return true;
}
void LWRSimRemote::gazeboStateCallback(ConstJointStatePtr& _msg)
{
    static bool rec_one = false;

    // Dump the message contents to stdout.
    // std::cout << _msg->time().sec()<<" " <<_msg->time().nsec()<<std::endl;

    size_t pos_size = _msg->position().size();
    size_t vel_size = _msg->velocity().size();
    size_t eff_size = _msg->effort().size();

    if(!rec_one)
    {
        rec_one = true;

        bool is_configured = LWRCommon::configureHook();
        if(this->configure())
            this->start();
    }

    joint_position_gazebo_in = Map<const VectorXd>(_msg->position().data(),pos_size);
    joint_velocity_gazebo_in = Map<const VectorXd>(_msg->velocity().data(),vel_size);
    joint_effort_gazebo_in   = Map<const VectorXd>(_msg->effort().data(),eff_size);

    stepInternalModel(joint_position_gazebo_in,joint_velocity_gazebo_in,joint_effort_gazebo_in);
    const Eigen::VectorXd& jnt_trq_out = LWRCommon::getComputedCommand();

    joint_state_msgs::msgs::JointState msg_out;
    for(int i=0;i<jnt_trq_out.size();++i)
        msg_out.add_effort(jnt_trq_out[i]);

    gz_state_pub->Publish(msg_out/*,true*/);
}

ORO_CREATE_COMPONENT(lwr::LWRSimRemote)
