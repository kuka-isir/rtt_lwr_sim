// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2016
#ifndef __LWRCOMMON_HPP__
#define __LWRCOMMON_HPP__
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <kuka_lwr_fri/friComm.h>

#include <lwr_fri/CartesianImpedance.h>
#include <lwr_fri/FriJointImpedance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

#include <tf_conversions/tf_kdl.h>

#include <Eigen/Dense>

#include <vector>

#include <rtt_rosclock/rtt_rosclock.h>
#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>
#include <rtt_roscomm/rtt_rostopic.h>

#include <rtt_rosparam/rosparam.h>

#include <rtt_ros_kdl_tools/tools.hpp>
#include <rtt_ros_kdl_tools/chain_utils.hpp>

#include <urdf/model.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/Empty.h>
#include <thread>
#include <memory>

#include <Eigen/Dense>
#include <ros/service.h>

namespace lwr{

    class LWRCommon : public RTT::TaskContext{
    public:
        LWRCommon(std::string const& name);
        bool configureHook();
        void updateHook();

        virtual ~LWRCommon(){};
    protected:
        void stepInternalModel(const Eigen::VectorXd& jnt_pos,
            const Eigen::VectorXd& jnt_vel,
            const Eigen::VectorXd& jnt_trq); // once jnt_pos,jnt_vel,jnt_trq, does all the calculations and write to ports

        bool waitForROSService(std::string service_name);
        const Eigen::VectorXd& getComputedCommand();
        void setJointImpedanceControlMode();
        void setJointTorqueControlMode();
        void setCartesianImpedanceControlMode();
        void resetJointImpedanceGains();
        void setInitialJointPosition(const std::vector<double>& joint_position_cmd);
        void resetCartesianImpedanceGains();
        bool readyService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& res);
        bool setGravityMode();
        bool setJointImpedance(const Eigen::VectorXd& stiffness, const Eigen::VectorXd& damping);
        bool setCartesianImpedance(const Eigen::VectorXd& cart_stiffness, const Eigen::VectorXd& cart_damping);

        RTT::OutputPort<bool> port_sync_status;
        RTT::InputPort<bool> port_sync_cmd;

        RTT::InputPort<lwr_fri::CartesianImpedance > port_CartesianImpedanceCommand;
        RTT::InputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
        RTT::InputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
        RTT::InputPort<lwr_fri::FriJointImpedance > port_JointImpedanceCommand;
        RTT::InputPort<Eigen::VectorXd > port_JointPositionCommand;
        RTT::InputPort<Eigen::VectorXd > port_JointTorqueCommand;
        //RTT::InputPort<std_msgs::Int32 > port_KRL_CMD;
        RTT::InputPort<tFriKrlData > port_ToKRL;
        //RTT::InputPort<std_msgs::Int32 > port_KRL_CMD;

        RTT::OutputPort<tFriKrlData > port_FromKRL;
        RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrench;
        RTT::OutputPort<geometry_msgs::WrenchStamped > port_CartesianWrenchStamped;
        RTT::OutputPort<tFriRobotState > port_RobotState;
        RTT::OutputPort<tFriIntfState > port_FRIState;
        RTT::OutputPort<Eigen::VectorXd > port_JointVelocity;
        RTT::OutputPort<geometry_msgs::Twist > port_CartesianVelocity;
        RTT::OutputPort<geometry_msgs::Pose > port_CartesianPosition;
        RTT::OutputPort<geometry_msgs::PoseStamped > port_CartesianPositionStamped;
        RTT::OutputPort<Eigen::MatrixXd > port_MassMatrix;
        RTT::OutputPort<KDL::Jacobian > port_Jacobian;
        RTT::OutputPort<Eigen::VectorXd > port_JointTorque,
                                          port_GravityTorque,
                                          port_JointPosition,
                                          port_JointTorqueRaw,
                                          port_JointPositionFRIOffset;
        int prop_fri_port;
        double dr_max_;

        bool set_joint_pos_no_dynamics_;
        bool set_brakes_;

        RTT::InputPort<sensor_msgs::JointState > port_JointStateGazebo;

        Eigen::VectorXd pos_limits_,
                        vel_limits_,
                        trq_limits_;

        Eigen::VectorXd jnt_pos_fri_offset_,
                        jnt_pos_old_,
                        jnt_trq_raw_,
                        grav_trq_,
                        jnt_vel_,
                        jnt_pos_cmd_,
                        jnt_trq_cmd_,
                        jnt_trq_gazebo_cmd_,
                        jnt_pos_brakes_,
                        jnt_pos_no_dyn_;

        Eigen::Matrix<double,6,1>  X_,
                                   X_cmd_,
                                   Xd_,
                                   Xd_cmd_,
                                   X_err_,
                                   Xd_err_,
                                   F_cmd_;
        KDL::Twist ee_twist_kdl_,
                   ee_twist_des_kdl_,
                   ee_twist_diff_kdl_,
                   ee_frame_diff_kdl_;

        KDL::Frame ee_frame_kdl_,
                   ee_frame_des_kdl_;

        KDL::FrameVel ee_framevel_kdl_;
        tf::Matrix3x3 quat_m;
        tf::Quaternion quat_tf;
        KDL::Jacobian jac_;

        geometry_msgs::PoseStamped cart_pos_stamped_;
        geometry_msgs::Pose cart_pos_, cart_pos_cmd_;
        geometry_msgs::Wrench cart_wrench_, cart_wrench_cmd_;
        geometry_msgs::WrenchStamped cart_wrench_stamped_;
        geometry_msgs::Twist cart_twist_;
        lwr_fri::FriJointImpedance jnt_imp_cmd_,jnt_imp_;
        lwr_fri::CartesianImpedance cart_imp_cmd_,cart_imp_;

        tFriIntfState fri_state;
        tFriRobotState robot_state;

        tFriKrlData fri_from_krl;
        tFriKrlData fri_to_krl;

        std::string urdf_str_;
        urdf::Model urdf_model_;
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;

        std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
        std::unique_ptr<KDL::ChainDynParam> id_dyn_solver;
        std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
        std::unique_ptr<KDL::ChainIdSolver_RNE> id_rne_solver;

        bool safety_checks_;

        KDL::Vector gravity_vector;

        //! Control gains
        Eigen::VectorXd kp_,
                        kd_,
                        kg_,
                        kp_default_,
                        kd_default_;

        Eigen::VectorXd             kcp_,
                                    kcd_,
                                    kcp_default_,
                                    kcd_default_;
        std::string robot_name_;

        sensor_msgs::JointState joint_states_,
                                joint_states_cmd_,
                                joint_states_dyn_;

        RTT::OutputPort<sensor_msgs::JointState> port_JointStates,
                                                 port_JointStatesCommand,
                                                 port_JointStatesDynamics;

        std::string root_link_,tip_link_,robot_ns_,tf_prefix_;

        bool use_sim_clock;

        bool safetyChecks(const Eigen::VectorXd& position,const Eigen::VectorXd& velocity,const Eigen::VectorXd& torque);
        bool safetyCheck(const Eigen::VectorXd& v, const Eigen::VectorXd& limits,const std::string& name="");
        void updateJointImpedance(const lwr_fri::FriJointImpedance& impedance);
        void updateCartesianImpedance(const lwr_fri::CartesianImpedance& cart_impedance);
        bool hasReceivedAtLeastOneCommand();
        //KDL Stuff
        KDL::Wrenches f_ext_;

        KDL::JntArray jnt_pos_kdl_,
                      jnt_trq_grav_kdl_,
                      jnt_vel_kdl_,
                      jnt_acc_kdl_,
                      jnt_trq_kdl_,
                      jnt_trq_coriolis_kdl_;

        RTT::FlowStatus jnt_trq_cmd_fs
                      ,jnt_pos_cmd_fs
                      ,cart_pos_cmd_fs
                      ,cart_wrench_cmd_fs
                      ,fri_to_krl_cmd_fs
                      ,jnt_imp_cmd_fs
                      ,cart_imp_cmd_fs;

        KDL::Wrench cart_wrench_kdl_;
        KDL::JntSpaceInertiaMatrix mass_kdl_;
        bool sync_with_cmds_;
        double period_sim_;
        double service_timeout_s;
        std::vector<int> joints_idx_;

        std::vector<std::string> joint_names_;
        unsigned int nb_no_data_,nb_loops_;
        // RTT::os::MutexRecursive gazebo_mutex_,rtt_mutex_;
        RTT::os::Semaphore rtt_sem_;
        double timeout_s;
        bool verbose = true;
        bool is_configured;
    };
}

#endif
