// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#ifndef __LWR_SIM_REMOTE_HPP__
#define __LWR_SIM_REMOTE_HPP__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rtt_lwr_sim/lwr_common.hpp>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <model_joint_plugin/joint_state.pb.h>

typedef const boost::shared_ptr<
    const joint_state_msgs::msgs::JointState>
ConstJointStatePtr;

namespace lwr{

    class LWRSimRemote : public LWRCommon{
    public:
        LWRSimRemote(std::string const& name);
        bool configureHook();
        virtual ~LWRSimRemote(){};
    protected:
        void gazeboStateCallback(ConstJointStatePtr& _msg);
        gazebo::transport::PublisherPtr gz_state_pub;
        std::vector<std::string> argv;
        gazebo::transport::NodePtr gz_node;
        gazebo::transport::SubscriberPtr gz_state_sub;

        // Current state
        Eigen::VectorXd joint_position_gazebo_in;
        Eigen::VectorXd joint_velocity_gazebo_in;
        Eigen::VectorXd joint_effort_gazebo_in;
        
        std::map<std::string,int> joint_idx_map;

    };
}

#endif
