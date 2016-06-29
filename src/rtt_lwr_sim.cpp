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
    this->addOperation("getModel",&LWRSim::getModel,this,OwnThread);
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

bool LWRSim::configureHook()
{
    if(model.get() == NULL) {
        RTT::log(RTT::Error)<<"No model could be loaded"<<RTT::endlog();
        return false;
    }
    
    if(!LWRCommon::configureHook())
        return false;
    // Get the joints
    gazebo_joints_ = model->GetJoints();
    model_links_ = model->GetLinks();
    
    std::vector<std::string> jn;
    for(int i=0;i<model->GetJoints().size();i++)
    {
        jn.push_back(model->GetJoints()[i]->GetName());
    }
    
    for(int i=0; i<jn.size(); i++)
        log(RTT::Debug) << "- gz_joint ["<<jn[i]<<"] --> idx "<<i << endlog();
        
    buildJointIndexMap(jn);
    
    const auto& jidx = getJointMapIndex();
    
    for(unsigned i=0; i<jidx.size(); i++)
        log(RTT::Debug) << "- actuated_joint ["<<gazebo_joints_[jidx[i]]->GetName()<<"] --> idx "<<jidx[i] << endlog();
        
    const int ndof = getNrOfJoints();

    jnt_pos_.setZero(ndof);
    jnt_vel_.setZero(ndof);
    jnt_trq_.setZero(ndof);

    return true;
}

void LWRSim::WorldUpdateEnd()
{
    if(!isRunning())
    {
        if(model)
            model->SetEnabled(false);
        return;
    }

    const auto& jidx = getJointMapIndex();
    // Read From gazebo simulation
    for(unsigned i=0; i<jidx.size(); i++) {
        jnt_pos_[i] = gazebo_joints_[jidx[i]]->GetAngle(0).Radian();
        jnt_vel_[i] = gazebo_joints_[jidx[i]]->GetVelocity(0);
        jnt_trq_[i] = gazebo_joints_[jidx[i]]->GetForce(0u);
    }

    stepInternalModel(jnt_pos_,jnt_vel_,jnt_trq_);
    
    const Eigen::VectorXd& jnt_trq_out = LWRCommon::getComputedCommand();
    // If user is connected, let's write command to gazebo
    if(this->hasReceivedAtLeastOneCommand())
    {
        static bool once = false;
        if(!once)
        {
            once = true;
            log(RTT::Info) << "["<<getName()<<"] Getting first command Tau = "<<jnt_trq_out.transpose() << endlog();
            for(int i=0;i<jidx.size();i++)
            {
                log(RTT::Info) << "  T["<<i<<"] (idx:"<<jidx[i]<<") --> "<<jnt_trq_out[i]<<endlog();
            }
        }
        model->SetEnabled(true);
        for(unsigned i=0; i<jidx.size(); i++)
            gazebo_joints_[jidx[i]]->SetForce(0,jnt_trq_out[i]);
    }
    else
    {
       model->SetEnabled(false);
        // If not, try to keep the same position TODO: find out why it drifts
//         for(auto joint : gazebo_joints_)
// #ifdef GAZEBO_GREATER_6
//               joint->SetPosition(0,joint->GetAngle(0).Radian());
// #else
//               joint->SetAngle(0,joint->GetAngle(0).Radian());
// #endif
    }
}

ORO_CREATE_COMPONENT(lwr::LWRSim)
