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

class JointDynamicAttractorOrocos : public RTT::TaskContext, public JointTrajectoryOrocos {
public:
    JointDynamicAttractorOrocos(std::string const &name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook() {return JointTrajectoryOrocos::startHook();}
    void updateHook();
    void stopHook() {JointTrajectoryOrocos::stopHook();}
    void cleanupHook() {JointTrajectoryOrocos::cleanupHook();}

private:

    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    RTT::FlowStatus in_robotstatus_flow;
    rstrt::robot::JointState in_robotstatus_var;

    std::shared_ptr<JointDynamicAttractor<float> > jda;
    bool first_config;
    Eigen::VectorXf state_temp;
};
