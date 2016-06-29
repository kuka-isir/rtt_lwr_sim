// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#ifndef LWR_SIM_HPP
#define LWR_SIM_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rtt_lwr_sim/lwr_common.hpp>


namespace lwr{

    class LWRSim : public LWRCommon{
    public:
        LWRSim(std::string const& name);
        bool configureHook();
        void WorldUpdateEnd();
        bool getModel(const std::string& gazebo_comp_name,
            const std::string& model_name,
            double timeout_s = 20.0);

        virtual ~LWRSim(){};
    protected:
        gazebo::physics::ModelPtr model;
        gazebo::event::ConnectionPtr world_end;

        gazebo::physics::Joint_V gazebo_joints_;
        gazebo::physics::Link_V model_links_;

        // Current state
        Eigen::VectorXd jnt_pos_;
        Eigen::VectorXd jnt_vel_;
        Eigen::VectorXd jnt_trq_;

    };
}

#endif
