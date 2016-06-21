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
    Logger::In in(getName());
    static bool rec_one = false;

    // Dump the message contents to stdout.
    // std::cout << _msg->time().sec()<<" " <<_msg->time().nsec()<<std::endl;

    if(!rec_one)
    {
        rec_one = true;

        if(LWRCommon::configureHook())
        {
            const int ndof = LWRCommon::getNrOfJoints();
            // Resize to the actual number of joints commanded
            
            joint_position_gazebo_in.setZero(ndof);
            joint_velocity_gazebo_in.setZero(ndof);
            joint_effort_gazebo_in.setZero(ndof);
            
            for(int i=0;i<_msg->name_size();i++)
            {
                joint_idx_map[_msg->name(i)] = i;
                std::cout << _msg->name(i)<<" --> " <<i<<std::endl;
            }
            for(int i=0;i<getJointNames().size();i++)
                std::cout << getJointNames().at(i)<<" --> " <<joint_idx_map[getJointNames().at(i)]<<std::endl;
            this->start();
            
        }else{
            log(Error) << "Could not configure LWRCommon" << endlog();
        }
    }

    const std::vector<std::string>& joint_names = getJointNames();
    
    for(int i=0;i<joint_names.size();++i)
    {
        joint_position_gazebo_in[i] = _msg->position(joint_idx_map[joint_names[i]]);
        joint_velocity_gazebo_in[i] = _msg->velocity(joint_idx_map[joint_names[i]]);
        joint_effort_gazebo_in[i]   = _msg->effort(joint_idx_map[joint_names[i]]);
    }

    //log(Debug) << "updating internal model + writing to ports" << endlog();
    stepInternalModel(joint_position_gazebo_in,joint_velocity_gazebo_in,joint_effort_gazebo_in);

    const Eigen::VectorXd& jnt_trq_out = LWRCommon::getComputedCommand();

    joint_state_msgs::msgs::JointState msg_out;
    
    for(int i=0;i<joint_names.size();++i)
    {
        msg_out.add_effort(jnt_trq_out[i]);
        msg_out.add_name(joint_names[i]);
    }
    

    gz_state_pub->Publish(msg_out/*,true*/);
}

ORO_CREATE_COMPONENT(lwr::LWRSimRemote)
