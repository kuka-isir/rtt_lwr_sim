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

    if(!LWRCommon::configureHook())
        return false;
    
    std::vector<std::string> jn;
    for(int i=0;i<model->GetJoints().size();i++)
        jn.push_back(model->GetJoints()[i]->GetName());
    
    buildJointIndexMap(jn);
    
    const int ndof = getNrOfJoints();

    jnt_pos_.setZero(ndof);
    jnt_vel_.setZero(ndof);
    jnt_trq_.setZero(ndof);

    return true;
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

    stepInternalModel(jnt_pos_,jnt_vel_,jnt_trq_);
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
    const Eigen::VectorXd& jnt_trq_out = LWRCommon::getComputedCommand();
    // If user is connected, let's write command to gazebo
    if(this->hasReceivedAtLeastOneCommand())
    {
        for(unsigned j=0; j<joints_idx_.size(); j++)
            gazebo_joints_[joints_idx_[j]]->SetForce(0,jnt_trq_out[j]);
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
