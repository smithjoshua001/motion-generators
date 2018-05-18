#include "motion-generators/DynamicAttractor.hpp"

DynamicAttractor::DynamicAttractor(std::string const &name) : RTT::TaskContext(name) {
    addOperation("setRobotDOF", &DynamicAttractor::setRobotDOF, this).doc("set robot dof");
    addOperation("setStateDOF", &DynamicAttractor::setStateDOF1, this).doc("set DOF size");
    addOperation("setGain", &DynamicAttractor::setGain, this).doc("set Gains");
    addOperation("setDesiredState", &DynamicAttractor::setDesiredState, this).doc("setDesiredState");
    portsArePrepared = false;
    first_config = false;
}

void DynamicAttractor::setRobotDOF(int dof) {
    DOF = dof;
    preparePorts();
}

void DynamicAttractor::setStateDOF1(int state_dof) {
    setStateDOF(state_dof, state_dof);
}

void DynamicAttractor::setStateDOF(int state_dof, int state_dot_dof) {
    state.resize(state_dof);
    state.setZero();
    state_dot.resize(state_dot_dof);
    state_dot.setZero();

    des_state.resize(state_dof);
    des_state.setZero();
    des_state_dot.resize(state_dot_dof);
    des_state_dot.setZero();

    gainMatrix.resize(state_dof, state_dof);
    gainMatrix.setZero();
}

void DynamicAttractor::preparePorts() {
    if (portsArePrepared) {
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("out_robotstatus_port");
    }

    //prepare input
    in_robotstatus_var = rstrt::robot::JointState(DOF);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    //prepare output
    out_robotstatus_var = rstrt::robot::JointState(DOF);
    out_robotstatus_var.angles.setZero();
    out_robotstatus_var.velocities.setZero();
    out_robotstatus_var.torques.setZero();
    out_robotstatus_port.setName("out_robotstatus_port");
    out_robotstatus_port.doc("Output port for sending robotstatus values");
    out_robotstatus_port.setDataSample(out_robotstatus_var);
    ports()->addPort(out_robotstatus_port);

    portsArePrepared = true;
}

bool DynamicAttractor::configureHook() {
    return portsArePrepared;
}
void DynamicAttractor::updateHook() {
    if (in_robotstatus_port.connected() && !first_config) {
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    } else {
        // return;
    }
    if (in_robotstatus_flow != RTT::NoData && !first_config) {
        state.head(DOF) = in_robotstatus_var.angles;
        state.tail(DOF) = in_robotstatus_var.velocities;
        first_config = true;
    } else if (in_robotstatus_flow == RTT::NoData) {
        return;
    }
    compute();
    out_robotstatus_var.angles = state.head(DOF);
    out_robotstatus_var.velocities = state.tail(DOF);
    out_robotstatus_port.write(out_robotstatus_var);
}

void DynamicAttractor::setGain(float gain) {
    gainMatrix.block(0, DOF, DOF, DOF).setIdentity();
    gainMatrix.block(DOF, 0, DOF, DOF).setIdentity();
    gainMatrix.block(DOF, 0, DOF, DOF) *= -gain * gain;
    gainMatrix.block(DOF, DOF, DOF, DOF).setIdentity();
    gainMatrix.block(DOF, DOF, DOF, DOF) *= -2 * gain;
}
void DynamicAttractor::compute() {
    state_dot = gainMatrix * (state - des_state);
    state = state + state_dot * 0.001;
    // RTT::log(RTT::Error) << state.transpose() << RTT::endlog();
    // RTT::log(RTT::Error) << des_state.transpose() << RTT::endlog();
    // RTT::log(RTT::Error) << state_dot.transpose() << RTT::endlog();
}

void DynamicAttractor::setDesiredState(Eigen::VectorXf &desiredState) {
    des_state = desiredState;
}

ORO_CREATE_COMPONENT_LIBRARY() ORO_LIST_COMPONENT_TYPE(DynamicAttractor)
