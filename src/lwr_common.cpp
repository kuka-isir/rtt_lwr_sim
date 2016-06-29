
// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/lwr_common.hpp>

using namespace lwr;
using namespace KDL;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

LWRCommon::LWRCommon(std::string const& name):TaskContext(name),
urdf_str_(""),
robot_name_(""),
root_link_(""),
tip_link_(""),
robot_ns_(""),
rtt_sem_(0),
use_sim_clock(true),
safety_checks_(false),
set_joint_pos_no_dynamics_(false),
set_brakes_(false),
sync_with_cmds_(false),
nb_loops_(0),
timeout_s(2.0),
is_configured(false),
verbose(false),
gravity_vector(0.,0.,-9.81289)
{
    this->addOperation("ready",&LWRCommon  ::readyService,this,RTT::ClientThread);
    this->addProperty("synchronize_with_commands", sync_with_cmds_).doc("");
    this->addProperty("synchronize_commands_timeout_s", timeout_s).doc("");
    this->addProperty("kp_default", kp_default_).doc("");
    this->addProperty("kd_default", kd_default_).doc("");
    this->addProperty("kcp_default", kcp_default_).doc("");
    this->addProperty("kcd_default", kcd_default_).doc("");
    //this->addAttribute("toKRL", m_toKRL);

    this->addProperty("root_link", root_link_).doc("");
    this->addProperty("set_brakes", set_brakes_).doc("");
    this->addProperty("tip_link", tip_link_).doc("");
    this->addProperty("robot_description",urdf_str_).doc("");
    this->addProperty("robot_ns",robot_ns_).doc("");
    this->addProperty("tf_prefix",tf_prefix_).doc("");
    this->addProperty("gravity_vector",gravity_vector).doc("");
    this->addProperty("use_sim_clock",use_sim_clock).doc("");
    this->addProperty("safety_checks",safety_checks_).doc("");
    this->addProperty("robot_name",robot_name_).doc("The name of the robot lwr/lwr_sim");

    this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");
    this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
    this->ports()->addPort("CartesianPositionCommand", port_CartesianPositionCommand).doc("");
    this->ports()->addPort("JointImpedanceCommand", port_JointImpedanceCommand).doc("");
    this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
    this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");

    this->ports()->addPort("toKRL",port_ToKRL).doc("");
    this->ports()->addPort("fromKRL",port_FromKRL).doc("");

    this->ports()->addPort("CartesianWrench", port_CartesianWrench).doc("");
    this->ports()->addPort("CartesianWrenchStamped", port_CartesianWrenchStamped).doc("");

    this->ports()->addPort("RobotState", port_RobotState).doc("");
    this->ports()->addPort("FRIState", port_FRIState).doc("");

    this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
    this->ports()->addPort("CartesianVelocity", port_CartesianVelocity).doc("");
    this->ports()->addPort("CartesianPosition", port_CartesianPosition).doc("");
    this->ports()->addPort("MassMatrix", port_MassMatrix).doc("");
    this->ports()->addPort("Jacobian", port_Jacobian).doc("");
    this->ports()->addPort("JointTorque", port_JointTorque).doc("");
    this->ports()->addPort("GravityTorque", port_GravityTorque).doc("");
    this->ports()->addPort("JointPosition", port_JointPosition).doc("");
    //this->ports()->addPort("JointTorqueRaw", port_JointTorqueRaw).doc("");
    //this->ports()->addPort("JointPositionFRIOffset", port_JointPositionFRIOffset).doc("");

    this->ports()->addPort("JointStates",port_JointStates).doc("");
    this->ports()->addPort("JointStatesCommand",port_JointStatesCommand).doc("");
    this->ports()->addPort("JointStatesDynamicsDecomposition",port_JointStatesDynamics).doc("");

    this->addProperty("kp",kp_);
    this->addProperty("kd",kd_);
    this->addProperty("kg",kg_);
    this->addProperty("kcp",kcp_);
    this->addProperty("kcd",kcd_);

    this->addProperty("verbose",verbose);

    this->addOperation("setJointImpedance",&LWRCommon::setJointImpedance,this,OwnThread);
    this->addOperation("setCartesianImpedance",&LWRCommon::setCartesianImpedance,this,OwnThread);
    this->addOperation("setGravityMode",&LWRCommon::setGravityMode,this,OwnThread);
    this->addOperation("resetJointImpedanceGains",&LWRCommon::resetJointImpedanceGains,this,OwnThread);

    this->addOperation("setInitialJointPosition",&LWRCommon::setInitialJointPosition,this,OwnThread);

}

void LWRCommon::setJointTorqueControlMode()
{
    if(!isConfigured())
    {
        log(Error) << "Please configure first, doing nothing." << endlog();
    }
    setJointImpedanceControlMode();
    Eigen::VectorXd kp(kdl_chain_.getNrOfJoints()),kd(kdl_chain_.getNrOfJoints());
    kp.setConstant(0.0);
    kd.setConstant(0.0);
    setJointImpedance(kp,kd);
}
bool LWRCommon::readyService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& res)
{
    return true;
}
const vector< string >& LWRCommon::getJointNames()
{
    return joint_names_;
}

bool LWRCommon::configureHook()
{
    // Get the rosparam service requester
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        log(Error) << "Could not load rosparam service." <<endlog();
        return false;
    }

    // We are in the /gazebo namespace,
    // So in the params we have (default) :
    // /robot_description
    // /robot_name
    // /root_link
    // /tip_link
    bool ret = true;
    rosparam->getRelative("robot_name");
    rosparam->getRelative("root_link");
    rosparam->getRelative("tip_link");
    rosparam->getRelative("tf_prefix");
    rosparam->getRelative("robot_description");

    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(this,kdl_tree_,kdl_chain_))
    {
        log(Error) << "Error while loading the URDF with params :"
        <<"\n -- root_link  : "<<root_link_
        <<"\n -- tip_link   : "<<tip_link_
        /*<<"\n -- tf_prefix   : "<<tf_prefix_*/
        <<endlog();
        return false;
    }
    
    const int ndof = kdl_chain_.getNrOfJoints();

    if(ndof == 0)
    {
        RTT::log(RTT::Error) << "No Joints could be added, exiting" << RTT::endlog();
        return false;
    }

    RTT::log(RTT::Info)<<"Gazebo model found "<<ndof<<" joints "<<RTT::endlog();
    
    id_dyn_solver.reset(new ChainDynParam(kdl_chain_,gravity_vector));
    id_rne_solver.reset(new ChainIdSolver_RNE(kdl_chain_,gravity_vector));
    fk_vel_solver.reset(new ChainFkSolverVel_recursive(kdl_chain_));
    jnt_to_jac_solver.reset(new ChainJntToJacSolver(kdl_chain_));

    // Fake LWR
    jnt_pos_brakes_.resize(ndof);
    jnt_pos_no_dyn_.resize(ndof);
    jnt_pos_fri_offset_.setZero(ndof);
    jnt_pos_old_.setZero(ndof);
    jnt_trq_raw_.setZero(ndof);
    grav_trq_.setZero(ndof);
    jnt_pos_cmd_.setZero(ndof);
    jnt_trq_cmd_.setZero(ndof);
    jnt_trq_gazebo_cmd_.setZero(ndof);

    kp_.setZero(ndof);
    kd_.setZero(ndof);
    kg_.setZero(ndof);
    kg_.setConstant(1.0);
    kp_default_.setZero(ndof);
    kd_default_.setZero(ndof);
    kcp_.setZero(6);
    kcd_.setZero(6);
    kcp_default_.setZero(6);
    kcd_default_.setZero(6);

    ret &= rosparam->getParam(getName() + "/kp_default","kp_default"); // searches in lwr_sim/<param-name>
    ret &= rosparam->getParam(getName() + "/kd_default","kd_default");
    ret &= rosparam->getParam(getName() + "/kcp_default","kcp_default");
    ret &= rosparam->getParam(getName() + "/kcd_default","kcd_default");

    for(int i=0;i<FRI_USER_SIZE;i++)
    {
        fri_from_krl.realData[i]=0.0;
        fri_from_krl.intData[i]=0;
    }
    fri_from_krl.boolData=0;
    fri_from_krl.fill=0;

    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_states_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_states_cmd_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_states_dyn_);
    
    joint_names_.clear();
    for(int i=0;i<ndof;i++)
        joint_names_.push_back(joint_states_.name[i]);

    jnt_pos_kdl_.resize(ndof);
    f_ext_.resize(kdl_chain_.getNrOfSegments());
    jnt_vel_kdl_.resize(ndof);
    jnt_acc_kdl_.resize(ndof);
    jnt_trq_kdl_.resize(ndof);
    jnt_trq_grav_kdl_.resize(ndof);
    jnt_trq_coriolis_kdl_.resize(ndof);
    jac_.resize(ndof);
    jac_.data.setZero();
    mass_kdl_.resize(ndof);
    mass_kdl_.data.setZero();

    robot_state.control = static_cast<fri_uint16_t>(FRI_CTRL_POSITION);
    fri_state.quality = static_cast<fri_uint16_t>(FRI_QUALITY_PERFECT);
    fri_to_krl.intData[0] = FRI_STATE_MON;
    fri_from_krl.intData[0] = FRI_STATE_MON;

    pos_limits_.resize(ndof);
    vel_limits_.resize(ndof);
    trq_limits_.resize(ndof);

    // Limits frmo Kuka manual
    pos_limits_ << 170*TORAD,120*TORAD,170*TORAD,120*TORAD,170*TORAD,120*TORAD,170*TORAD;
    vel_limits_ << 112.5*TORAD,112.5*TORAD,112.5*TORAD,112.5*TORAD,180*TORAD,112.5*TORAD,112.5*TORAD;
    trq_limits_ << 200,200,100,100,100,30,30;

    port_JointStates.setDataSample(joint_states_);
    port_JointStatesCommand.setDataSample(joint_states_cmd_);
    port_JointStatesDynamics.setDataSample(joint_states_dyn_);

    port_JointPosition.setDataSample(jnt_pos_cmd_);
    port_JointVelocity.setDataSample(jnt_vel_kdl_.data);
    port_JointTorque.setDataSample(jnt_trq_cmd_);
    port_GravityTorque.setDataSample(jnt_trq_grav_kdl_.data);
    port_Jacobian.setDataSample(jac_);
    port_MassMatrix.setDataSample(mass_kdl_.data);
    port_FromKRL.setDataSample(fri_from_krl);

    resetJointImpedanceGains();
    resetCartesianImpedanceGains();
    
    return true;
}

void LWRCommon::buildJointIndexMap(const std::vector<std::string>& 
                                    jnt_names_from_gz)
{
    if(!LWRCommon::isConfigured() || !joint_names_.size())
    {
        log(Error) << "Not configured, can't build joint map index" << endlog();
        return;
    }
    joints_idx_.clear();

    for(auto name : joint_names_)
    {
        auto it = std::find(jnt_names_from_gz.begin(), 
                jnt_names_from_gz.end(), name);
        if (it != jnt_names_from_gz.end())
            joints_idx_.push_back(std::distance(jnt_names_from_gz.begin(), it));
    }
}
const vector< int >& LWRCommon::getJointMapIndex()
{
    return joints_idx_;
}

void LWRCommon::resetJointImpedanceGains()
{
    this->setJointImpedance(kp_default_,kd_default_);
}
void LWRCommon::resetCartesianImpedanceGains()
{
    this->setCartesianImpedance(kcp_default_,kcd_default_);
}
void LWRCommon::setInitialJointPosition(const std::vector<double>& jnt_pos_cmd)
{
    if(!jnt_pos_cmd.size() == kdl_chain_.getNrOfJoints()){
        log(Error) << "Invalid size ( found "<<jnt_pos_cmd.size() << " should be " << kdl_chain_.getNrOfJoints() << ")" << endlog();
        return;
    }

    jnt_pos_no_dyn_ = VectorXd::Map(jnt_pos_cmd.data(),jnt_pos_cmd.size());
    set_joint_pos_no_dynamics_ = true;
}

bool LWRCommon::safetyChecks(const VectorXd& position,
                          const VectorXd& velocity,
                          const VectorXd& torque)
{
    return safetyCheck(position,pos_limits_,"Position") &&
    safetyCheck(velocity,vel_limits_,"Velocity") &&
    safetyCheck(torque,trq_limits_,"Torque");
}

int LWRCommon::getNrOfJoints()
{
    return kdl_chain_.getNrOfJoints();
}

bool LWRCommon::safetyCheck(const VectorXd& v,
                         const VectorXd& limits,
                         const std::string& name)
{
    if(v.size() != kdl_chain_.getNrOfJoints())
    {
        log(Error) << name<<" vector size error "<<v.size()<<"!="<<kdl_chain_.getNrOfJoints()<<endlog();
        return false;
    }

    bool ret=true;
    for(unsigned i=0;i<v.size();i++)
    {
        if(std::abs(v[i]) > limits[i])
        {
           log(Error) << name<<" limit exceded at J"<<i<<" : "<<v[i]<<" / limit "<<limits[i]<<endlog();
           ret = false;
        }
    }
    return ret;
}

bool LWRCommon::setJointImpedance(const VectorXd& stiffness, const VectorXd& damping)
{
    if(! (stiffness.size() == kdl_chain_.getNrOfJoints() && damping.size() == kdl_chain_.getNrOfJoints()))
    {
        log(Error) << "Size error, not setting impedance (stiff size "
                            <<stiffness.size()<<" damp size "<<damping.size()
                            <<", should be "<<kdl_chain_.getNrOfJoints()<<")"<<endlog();
        return false;
    }

    for(unsigned j=0;j<kdl_chain_.getNrOfJoints();++j)
    {
        jnt_imp_.stiffness[j] = stiffness[j];
        jnt_imp_.damping[j] = damping[j];
    }

    kp_ = stiffness;
    kd_ = damping;

    return true;
}
bool LWRCommon::setGravityMode()
{
    VectorXd s(kdl_chain_.getNrOfJoints());
    s.setZero();
    VectorXd d(kdl_chain_.getNrOfJoints());
    d.setZero();
    return this->setJointImpedance(s,d);
}

bool LWRCommon::setCartesianImpedance(const VectorXd& cart_stiffness, const VectorXd& cart_damping)
{
    if(cart_damping.size() != 6 || cart_damping.size() != 6)
    {
        log(Error) << "Size error, not setting cartesian impedance (stiff size "
                            <<cart_stiffness.size()<<" damp size "<<cart_damping.size()
                            <<", should be 6 and 6)"<<endlog();
        return false;
    }

    cart_imp_.stiffness.linear.x = cart_stiffness[0];
    cart_imp_.stiffness.linear.y = cart_stiffness[1];
    cart_imp_.stiffness.linear.z = cart_stiffness[2];

    cart_imp_.stiffness.angular.x = cart_stiffness[3];
    cart_imp_.stiffness.angular.y = cart_stiffness[4];
    cart_imp_.stiffness.angular.z = cart_stiffness[5];

    cart_imp_.damping.linear.x = cart_damping[0];
    cart_imp_.damping.linear.y = cart_damping[1];
    cart_imp_.damping.linear.z = cart_damping[2];

    cart_imp_.damping.angular.x = cart_damping[3];
    cart_imp_.damping.angular.y = cart_damping[4];
    cart_imp_.damping.angular.z = cart_damping[5];

    kcp_ = cart_stiffness;
    kcd_ = cart_damping;

    return true;
}

void LWRCommon::updateJointImpedance(const lwr_fri::FriJointImpedance& impedance)
{
    for(unsigned int i=0;i<kdl_chain_.getNrOfJoints()
        && i<impedance.damping.size()
        && i<impedance.stiffness.size();i++)
    {
        if(impedance.stiffness[i] >=0)
            kp_[i] = impedance.stiffness[i];
        if(impedance.damping[i] >= 0)
            kd_[i] = impedance.damping[i];
    }
}

void LWRCommon::updateCartesianImpedance(const lwr_fri::CartesianImpedance& cart_impedance)
{
    kcp_[0] = cart_impedance.stiffness.linear.x;
    kcp_[1] = cart_impedance.stiffness.linear.y;
    kcp_[2] = cart_impedance.stiffness.linear.z;

    kcp_[3] = cart_impedance.stiffness.angular.x;
    kcp_[4] = cart_impedance.stiffness.angular.y;
    kcp_[5] = cart_impedance.stiffness.angular.z;

    kcd_[0] = cart_impedance.damping.linear.x;
    kcd_[1] = cart_impedance.damping.linear.y;
    kcd_[2] = cart_impedance.damping.linear.z;

    kcd_[3] = cart_impedance.damping.angular.x;
    kcd_[4] = cart_impedance.damping.angular.y;
    kcd_[5] = cart_impedance.damping.angular.z;

    cart_imp_ = cart_impedance;
}


void LWRCommon::setJointImpedanceControlMode()
{
    fri_to_krl.intData[1] = 10*FRI_CTRL_JNT_IMP;
}
void LWRCommon::setCartesianImpedanceControlMode()
{
    fri_to_krl.intData[1] = 10*FRI_CTRL_CART_IMP;
}

bool LWRCommon::hasReceivedAtLeastOneCommand()
{
    static bool has_one_cmd = false;
    if(has_one_cmd) return true;

    switch(static_cast<FRI_CTRL>(robot_state.control)){
        case FRI_CTRL_JNT_IMP:
            if(jnt_trq_cmd_fs == NewData || jnt_pos_cmd_fs == NewData)
                has_one_cmd = true;
            break;

        case FRI_CTRL_POSITION:
            if(jnt_pos_cmd_fs == NewData)
                has_one_cmd = true;
            break;
        case FRI_CTRL_CART_IMP:
            if(cart_pos_cmd_fs == NewData)
                has_one_cmd = true;
            break;
        default:
            // log(Error) << "hasReceivedAtLeastOneCommand() Wrong control mode "
            // << "robot_state.control="<<robot_state.control<<endlog();
            break;
    }
    return has_one_cmd;
}

void LWRCommon::stepInternalModel(
    const Eigen::VectorXd& jnt_pos,
    const Eigen::VectorXd& jnt_vel,
    const Eigen::VectorXd& jnt_trq)
{
    const int ndof = kdl_chain_.getNrOfJoints();

    TimeService::nsecs tstart,tstart_wait;
    tstart = TimeService::Instance()->getNSecs();
    // Reset commands from users
    jnt_trq_cmd_.setZero();
    // Reset joint command to current in case we are missing a command
    if(set_brakes_ == false)
        jnt_pos_cmd_ = jnt_pos;

    Xd_cmd_.setZero();
    F_cmd_.setZero();

    fri_to_krl_cmd_fs = port_ToKRL.readNewest(fri_to_krl);

    ros::Time time_now = rtt_rosclock::host_now();
    fri_state.timestamp = time_now.toNSec();

    // Update Robot Internal State to mimic KRC
    // Update cmds to FRI
    if(fri_to_krl_cmd_fs == NewData)
    {
        for(int i=0;i<FRI_USER_SIZE;i++)
        {
            fri_from_krl.realData[i]=fri_to_krl.realData[i];
            fri_from_krl.intData[i]=fri_to_krl.intData[i];
        }
        fri_from_krl.boolData=fri_to_krl.boolData;
    }
    // Update robot status
    switch(fri_to_krl.intData[1])
    {
        case 10:
            robot_state.control = FRI_CTRL_POSITION;
            break;
        case 30:
            robot_state.control = FRI_CTRL_JNT_IMP;
            break;
        case 20:
            robot_state.control = FRI_CTRL_CART_IMP;
            break;
        default:
            // log(Error) << "fri_to_krl.intData[1] is wrong : "<<fri_to_krl.intData[1]<<endlog();
            robot_state.control = FRI_CTRL_OTHER;
            break;
    }
    // Update robot power state + fri state
    switch(static_cast<FRI_CTRL>(robot_state.control))
    {
        case FRI_CTRL_POSITION:
        case FRI_CTRL_JNT_IMP:
        case FRI_CTRL_CART_IMP:
            fri_state.state = fri_from_krl.intData[0] = FRI_STATE_CMD;
            robot_state.power = 1;
            break;
        case FRI_CTRL_OTHER:
            fri_state.state = fri_from_krl.intData[0] = FRI_STATE_MON;
            robot_state.power = 0;
            break;
        default:
            fri_state.state = fri_from_krl.intData[0] = FRI_STATE_OFF;
            robot_state.power = 0;
            break;
    }

    if(safety_checks_)
        safetyChecks(jnt_pos,jnt_vel,jnt_trq);

    bool exit_loop = false;
    int n_wait=0;
    tstart_wait = TimeService::Instance()->getNSecs();
    do
    {
        jnt_trq_cmd_fs = port_JointTorqueCommand.readNewest(jnt_trq_cmd_);
        jnt_pos_cmd_fs = port_JointPositionCommand.readNewest(jnt_pos_cmd_);
        cart_pos_cmd_fs = port_CartesianPositionCommand.readNewest(cart_pos_cmd_);

        if(jnt_pos_cmd_fs == NoData && jnt_trq_cmd_fs == NoData && cart_pos_cmd_fs == NoData)
            break;

        switch(static_cast<FRI_CTRL>(robot_state.control)){
            case FRI_CTRL_JNT_IMP:
                if(jnt_trq_cmd_fs == NewData || jnt_pos_cmd_fs == NewData)
                {
                    exit_loop = true;
                    nb_loops_++;
                }
                break;
            case FRI_CTRL_OTHER:
                exit_loop = true;
                break;
            case FRI_CTRL_POSITION:
                if(jnt_pos_cmd_fs == NewData || jnt_pos_cmd_fs == NoData)
                {
                    exit_loop = true;
                    nb_loops_++;
                }
                break;
            case FRI_CTRL_CART_IMP:
                if(cart_pos_cmd_fs == NewData || cart_pos_cmd_fs == NoData)
                {
                    exit_loop = true;
                    nb_loops_++;
                }
                break;
            default:
                break;
        }
        if(!sync_with_cmds_)
            break;

        if(verbose && n_wait % 10 == 0 && !exit_loop)
        {
            log(RTT::Debug) << getName()
            << " WorldUpdateBegin() : waiting "
            << TimeService::Instance()->getNSecs()
            <<" - pos:"<<jnt_pos_cmd_fs
            <<" - trq:"<<jnt_trq_cmd_fs
            <<" - crt:"<<cart_pos_cmd_fs
            << endlog();
        }
        if(!exit_loop){
            n_wait++;
            usleep(250);
            if(n_wait * 250. > timeout_s*1e6)
            {
                log(RTT::Debug) << getName() << " Timeout at "<< TimeService::Instance()->getNSecs() << endlog();
                break;
            }
        }
    }while(!exit_loop);

    // This is to fix a strange bug on 2.9
    // When scripting connects ports, jnt_pos goes back to a zero size
    // even though there's no data
    if(jnt_trq_cmd_.size() != ndof)
    {
        jnt_trq_cmd_.setZero(ndof);
    }
    if(jnt_pos_cmd_.size() != ndof)
    {
        jnt_pos_cmd_.resize(ndof);
        jnt_pos_cmd_ = jnt_pos;
    }

    TimeService::nsecs tduration_wait = TimeService::Instance()->getNSecs(tstart_wait);

    cart_wrench_cmd_fs = port_CartesianWrenchCommand.readNewest(cart_wrench_cmd_);
    jnt_imp_cmd_fs = port_JointImpedanceCommand.readNewest(jnt_imp_cmd_);
    cart_imp_cmd_fs = port_CartesianImpedanceCommand.readNewest(cart_imp_cmd_);

    // Update joint impedance gains
    if(jnt_imp_cmd_fs == NewData)
        updateJointImpedance(jnt_imp_cmd_);

    // Update cartesian impedance gains
    if(cart_imp_cmd_fs == NewData)
        updateCartesianImpedance(cart_imp_cmd_);

    // Copy State to KDL
    jnt_pos_kdl_.data = jnt_pos;
    jnt_vel_kdl_.data = jnt_vel;
    jnt_acc_kdl_.data.setConstant(0.0);

    // Compute MassMatrix
    id_dyn_solver->JntToMass(jnt_pos_kdl_,mass_kdl_);

    // Compute Inverse dynamics [q,qdot] => T
    std::fill(f_ext_.begin(),f_ext_.end(),Wrench::Zero());
    id_rne_solver->CartToJnt(jnt_pos_kdl_,jnt_vel_kdl_,jnt_acc_kdl_,f_ext_,jnt_trq_kdl_);

    // Compute Gravity terms G(q)
    id_dyn_solver->JntToGravity(jnt_pos_kdl_,jnt_trq_grav_kdl_);

    // Compute Coriolis terms b(q,qdot)
    id_dyn_solver->JntToCoriolis(jnt_pos_kdl_,jnt_vel_kdl_,jnt_trq_coriolis_kdl_);

    Map<VectorXd>(joint_states_dyn_.position.data(),ndof) = jnt_trq_grav_kdl_.data;
    Map<VectorXd>(joint_states_dyn_.velocity.data(),ndof) = jnt_trq_coriolis_kdl_.data;
    Map<VectorXd>(joint_states_dyn_.effort.data(),ndof) = jnt_trq_kdl_.data;

    jnt_to_jac_solver->JntToJac(jnt_pos_kdl_,jac_,kdl_chain_.getNrOfSegments());
    fk_vel_solver->JntToCart(JntArrayVel(jnt_pos_kdl_,jnt_vel_kdl_),ee_framevel_kdl_,kdl_chain_.getNrOfSegments());
    ee_twist_kdl_ = ee_framevel_kdl_.GetTwist();
    ee_frame_kdl_ = ee_framevel_kdl_.GetFrame();

    tf::poseKDLToMsg(ee_frame_kdl_,cart_pos_);
    tf::twistKDLToMsg(ee_twist_kdl_,cart_twist_);

    cart_pos_stamped_.header.frame_id = root_link_;
    cart_pos_stamped_.pose = cart_pos_;


    //ee_twist_des_kdl_.SetToZero();
    SetToZero(ee_twist_des_kdl_);
    ee_twist_diff_kdl_ = diff(ee_twist_kdl_,ee_twist_des_kdl_);
    tf::twistKDLToEigen(ee_twist_diff_kdl_,Xd_err_);

    if(cart_pos_cmd_fs == NewData){
        tf::poseMsgToKDL(cart_pos_cmd_,ee_frame_des_kdl_);
        ee_frame_diff_kdl_ = diff(ee_frame_kdl_,ee_frame_des_kdl_);
        tf::twistKDLToEigen(ee_frame_diff_kdl_,X_err_);
    }


    tf::wrenchMsgToEigen(cart_wrench_cmd_,F_cmd_);

    cart_wrench_stamped_.header.frame_id = tip_link_;
    cart_wrench_stamped_.wrench = cart_wrench_;

    jnt_trq_gazebo_cmd_ = kg_.asDiagonal() * jnt_trq_grav_kdl_.data;

    // COMPUTE COMMANDS
    switch(static_cast<FRI_CTRL>(robot_state.control)){
        case FRI_CTRL_JNT_IMP:
            // Joint Impedance part
            jnt_trq_gazebo_cmd_ += kp_.asDiagonal()*(jnt_pos_cmd_ - jnt_pos) - kd_.asDiagonal()*jnt_vel ;
            // Additional torque
            jnt_trq_gazebo_cmd_ += jnt_trq_cmd_;

            set_brakes_ = false == (jnt_trq_cmd_fs == NewData || jnt_pos_cmd_fs == NewData);

            break;
        case FRI_CTRL_OTHER:
        case FRI_CTRL_POSITION:
            // Joint Position (Joint Imp with kd fixed)
            jnt_trq_gazebo_cmd_ += kp_default_.asDiagonal()*(jnt_pos_cmd_-jnt_pos) - kd_default_.asDiagonal()*jnt_vel ;

            set_brakes_ = !(jnt_pos_cmd_fs == NewData);
            break;
        case FRI_CTRL_CART_IMP:
            //Cartesian Impedance Control
            jnt_trq_gazebo_cmd_ += jac_.data.transpose()*(kcp_.asDiagonal()*(X_err_ + F_cmd_) + kcd_.asDiagonal()*(Xd_err_));

            set_brakes_ = !(cart_pos_cmd_fs == NewData);

            break;
        default:
            break;
    }

    //Update status
    cart_pos_stamped_.header.stamp =
    cart_wrench_stamped_.header.stamp =
    joint_states_.header.stamp =
    joint_states_cmd_.header.stamp =
    joint_states_dyn_.header.stamp = time_now;

    Map<VectorXd>(joint_states_.position.data(),ndof) = jnt_pos;
    Map<VectorXd>(joint_states_.velocity.data(),ndof) = jnt_vel;
    Map<VectorXd>(joint_states_.effort.data(),ndof) = jnt_trq;

    Map<VectorXd>(joint_states_cmd_.position.data(),ndof) = jnt_pos_cmd_;
    Map<VectorXd>(joint_states_cmd_.effort.data(),ndof) = jnt_trq_gazebo_cmd_ - jnt_trq_grav_kdl_.data;

    port_JointStates.write(joint_states_);
    port_JointStatesCommand.write(joint_states_cmd_);
    port_JointStatesDynamics.write(joint_states_dyn_);


    port_GravityTorque.write(jnt_trq_grav_kdl_.data);

    port_CartesianPosition.write(cart_pos_);
    port_CartesianPositionStamped.write(cart_pos_stamped_);
    port_CartesianVelocity.write(cart_twist_);
    port_CartesianWrench.write(cart_wrench_);
    port_CartesianWrenchStamped.write(cart_wrench_stamped_);

    port_Jacobian.write(jac_);
    port_MassMatrix.write(mass_kdl_.data);

    port_FromKRL.write(fri_from_krl);

    port_RobotState.write(robot_state);
    port_FRIState.write(fri_state);

    port_JointPosition.write(jnt_pos);
    port_JointVelocity.write(jnt_vel);
    port_JointTorque.write(jnt_trq);
    
    TimeService::nsecs tduration = TimeService::Instance()->getNSecs(tstart);

    if(verbose)
    {
        log(RTT::Debug) << getName() << " WorldUpdateBegin()  END "
        <<tstart
        <<"\n - ["<<jnt_trq_cmd_fs<<"]\tjnt_trq_cmd:"<<jnt_trq_cmd_.transpose()
        <<"\n - ["<<jnt_pos_cmd_fs<<"]\tjnt_pos_cmd:"<<jnt_pos_cmd_.transpose()
        <<"\n - sync_with_cmds:"<<sync_with_cmds_
        <<"\n - Waited for cmds (loops):"<<n_wait
        <<"\n - Waited for cmds ns :"<<tduration_wait
        <<"\n - set_brakes:"<<set_brakes_
        <<"\n - robot_state.control:"<<robot_state.control
        <<"\n -- kp: "<<kp_.transpose()
        <<"\n -- kd: "<<kd_.transpose()
        <<"\n -- duration: "<<tduration<< endlog();
    }
}
const Eigen::VectorXd& LWRCommon::getComputedCommand()
{
    return jnt_trq_gazebo_cmd_;
}
