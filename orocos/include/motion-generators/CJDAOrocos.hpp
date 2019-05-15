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

#include "motion-generators/JointDynamicAttractor.hpp"
#include "motion-generators/JointTrajectoryOrocos.hpp"

class CJDAOrocos : public RTT::TaskContext, public JointTrajectoryOrocos {
public:
    CJDAOrocos(std::string const &name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook() {
    stop = false;
    return JointTrajectoryOrocos::startHook();}
    void updateHook();
    void stopHook() {JointTrajectoryOrocos::stopHook();}
    void cleanupHook() {JointTrajectoryOrocos::cleanupHook();}

    void goToStop();

private:

    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    RTT::FlowStatus in_robotstatus_flow;
    rstrt::robot::JointState in_robotstatus_var;

RTT::InputPort<rstrt::kinematics::JointAngles> in_despos_port;
    RTT::FlowStatus in_despos_flow;
    rstrt::kinematics::JointAngles in_despos_var;

RTT::InputPort<rstrt::kinematics::JointVelocities> in_desvel_port;
    RTT::FlowStatus in_desvel_flow;
    rstrt::kinematics::JointVelocities in_desvel_var;

RTT::InputPort<rstrt::kinematics::JointAccelerations> in_desacc_port;
    RTT::FlowStatus in_desacc_flow;
    rstrt::kinematics::JointAccelerations in_desacc_var;

    std::shared_ptr<JointDynamicAttractor<float> > jda;
    bool first_config, stop;
    Eigen::VectorXf state_temp;
};
