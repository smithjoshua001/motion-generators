#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp> // needed for the macro at the end of this file
#include <string>
#include <Eigen/Dense>

#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/kinematics/JointAccelerations.hpp>

#include <rst-rt/geometry/Pose.hpp>
#include <rst-rt/kinematics/Twist.hpp>

class TaskDynamicAttractor : public RTT::TaskContext {
public:
    TaskDynamicAttractor(std::string const &name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook() {start_time = 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()); return true;}
    void updateHook();
    void stopHook() {}
    void cleanupHook() {}

    void setGain(float gain, float orientation_gain);
    void compute();
    void preparePorts();
    void setDesiredState(Eigen::VectorXf &desiredState);

    template<typename Derived, typename Derived2> static void skewMatrix(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> &skewMat) {
        //util::logger->debug("File: UTIL Message: skewmat zero2 {}" );
        skewMat.setZero();
        skewMat(2, 1) = vector(0);
        skewMat(1, 2) = -vector(0);
        skewMat(0, 2) = vector(1);
        skewMat(2, 0) = -vector(1);
        skewMat(0, 1) = -vector(2);
        skewMat(1, 0) = vector(2);
    }

private:
    Eigen::MatrixXf gainMatrix;
    Eigen::MatrixXf skewGainMatrix;
    Eigen::Matrix3f skewMat;
    Eigen::VectorXf state, state_dot;
    Eigen::VectorXf des_state, des_state_dot;

    Eigen::VectorXf state_error;
    Eigen::MatrixXf quat_left;

    int DOF;

    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::geometry::Pose> in_pose_port;
    RTT::InputPort<rstrt::kinematics::Twist> in_twist_port;

    // Data flow:
    RTT::FlowStatus in_pose_flow;
    RTT::FlowStatus in_twist_flow;

    rstrt::geometry::Pose in_pose_var;
    rstrt::kinematics::Twist in_twist_var;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::geometry::Pose> out_pose_port;
    RTT::OutputPort<rstrt::kinematics::Twist> out_twist_port;
    RTT::OutputPort<Eigen::VectorXf> out_acceleration_port;

    rstrt::geometry::Pose out_pose_var;
    rstrt::kinematics::Twist out_twist_var;
    Eigen::VectorXf out_acceleration_var;

    bool portsArePrepared, first_config, go;

    double start_time;
};
