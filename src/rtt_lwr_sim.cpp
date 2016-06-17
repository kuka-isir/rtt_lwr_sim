// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/rtt_lwr_sim.hpp>
#include <Eigen/Dense>
#include <ros/service.h>

using namespace lwr;
using namespace KDL;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;
LWRSim::LWRSim(const string& name): LWRCommon(name)
{
    this->addOperation("getModel",&LWRSim::getModel,this,ClientThread);
    world_begin =  gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&LWRSim::WorldUpdateBegin,this));
    world_end = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&LWRSim::WorldUpdateEnd,this));
}

bool LWRSim::getModel(const std::string& gazebo_comp_name,
                      const std::string& model_name,
                      double timeout_s)
{
    if(model)
    {
        log(Warning) << "Model ["<<model_name<<"] already loaded !"<< endlog();
        return true;
    }
    gazebo::printVersion();
    if(! gazebo::physics::get_world())
    {
        log(Error) << "getWorldPtr does not seem to exists" << endlog();
        return false;
    }
    model = gazebo::physics::get_world()->GetModel(getName());
    if(model)
    {
        log(Info) << "Model ["<<model_name<<"] successfully loaded !"<< endlog();
        return true;
    }

    return bool(model);
}

bool LWRSim::setLinkGravityMode(const std::string& link_name,bool gravity_mode)
{
    // HACK: I want to remove gravity for ati_link (force torque sensor), but
    // <gravity> tag does not work for some reason, so I'm doin' it here.
    // FIXME

    for(gazebo::physics::Link_V::iterator it = this->model_links_.begin();
        it != this->model_links_.end();++it)
        {
            if((*it)->GetName() == link_name)
            {
                gravity_mode_.insert(std::make_pair((*it),gravity_mode));
                RTT::log(RTT::Warning)<<"Setting link "<<link_name<<" to "<<((*it)->GetGravityMode()? "true":"false")<<RTT::endlog();
                return true;
            }
        }
    RTT::log(RTT::Error)<<"["<<getName()<<"] setLinkGravityMode() -> "<<link_name<<" does not exists !"<<RTT::endlog();
    return false;
}

bool LWRSim::configureHook()
{
    if(model.get() == NULL) {
        RTT::log(RTT::Error)<<"No model could be loaded"<<RTT::endlog();
        return false;
    }
    // Get the joints
    gazebo_joints_ = model->GetJoints();
    model_links_ = model->GetLinks();

    RTT::log(RTT::Info)<<"Model has "<<gazebo_joints_.size()<<" joints"<<RTT::endlog();
    RTT::log(RTT::Info)<<"Model has "<<model_links_.size()<<" links"<<RTT::endlog();

    //NOTE: Get the joint names and store their indices
    // Because we have base_joint (fixed), j0...j6, ati_joint (fixed)
    int idx = 0;
    joints_idx_.clear();
    for(gazebo::physics::Joint_V::iterator jit=gazebo_joints_.begin();
        jit != gazebo_joints_.end();++jit,++idx)
    {

        const std::string name = (*jit)->GetName();
        // NOTE: Remove fake fixed joints (revolute with upper==lower==0
        // NOTE: This is not used anymore thanks to <disableFixedJointLumping>
        // Gazebo option (ati_joint is fixed but gazebo can use it )

        if((*jit)->GetLowerLimit(0u) == (*jit)->GetUpperLimit(0u))
        {
            RTT::log(RTT::Info)<<"Not adding (fake) fixed joint ["<<name<<"] idx:"<<idx<<RTT::endlog();
            continue;
        }
        joints_idx_.push_back(idx);
        joint_names_.push_back(name);
        RTT::log(RTT::Info)<<"Adding joint ["<<name<<"] idx:"<<idx<<RTT::endlog();
    }
    is_configured = LWRCommon::configureHook();

    return is_configured;
}

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

ORO_CREATE_COMPONENT(lwr::LWRSim)
