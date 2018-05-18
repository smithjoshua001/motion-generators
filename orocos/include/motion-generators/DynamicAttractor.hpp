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

class DynamicAttractor : public RTT::TaskContext {
public:
    DynamicAttractor(std::string const &name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook() {return true;}
    void updateHook();
    void stopHook() {}
    void cleanupHook() {}

    void setRobotDOF(int dof);
    void setStateDOF1(int state_dof);
    void setStateDOF(int state_dof, int state_dot_dof);
    void setGain(float gain);
    void compute();
    void preparePorts();
    void setDesiredState(Eigen::VectorXf &desiredState);

private:
    Eigen::MatrixXf gainMatrix;
    Eigen::VectorXf state, state_dot;
    Eigen::VectorXf des_state, des_state_dot;

    int DOF;

    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;

    rstrt::robot::JointState in_robotstatus_var;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::robot::JointState> out_robotstatus_port;

    rstrt::robot::JointState out_robotstatus_var;

    bool portsArePrepared, first_config;
};
