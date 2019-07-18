#include "motion-generators/JointDynamicAttractorOrocos.hpp"

JointDynamicAttractorOrocos::JointDynamicAttractorOrocos(std::string const &name) :
    JointTrajectoryOrocos(name, std::static_pointer_cast<JointTrajectory<float> >(std::make_shared<JointDynamicAttractor<float> >())) {
    first_config = false;
    jda = std::static_pointer_cast<JointDynamicAttractor<float> >(this->getTrajectory());
    addOperation("setGain", &JointDynamicAttractor<float>::setGain, jda.get());
    addOperation("setDesiredState", &JointDynamicAttractor<float>::setDesiredState, jda.get());
    addOperation("setLimit", &JointDynamicAttractor<float>::setLimit, jda.get());
}

bool JointDynamicAttractorOrocos::configureHook() {
    bool temp = JointTrajectoryOrocos::configureHook();
    if (temp) {
        in_robotstatus_var = rstrt::robot::JointState(this->dof);
        in_robotstatus_port.setName("in_robotstatus_port");
        in_robotstatus_port.doc("Input port for reading the robotstatuss for all chains");
        ports()->addPort(in_robotstatus_port);
        in_robotstatus_flow = RTT::NoData;
        state_temp.resize(this->dof * 2);
        state_temp.setZero();
    }
    return temp;
}
void JointDynamicAttractorOrocos::updateHook() {
    // std::cout<<"UPDATE HOOK!"<<std::endl;
    if (this->in_robotstatus_port.connected() && !first_config) {
        this->in_robotstatus_flow = this->in_robotstatus_port.read(this->in_robotstatus_var);
    } else {
        // return;
    }
    // std::cout<<"Read in_robotstatus!"<<std::endl;
    if (this->in_robotstatus_flow != RTT::NoData && !first_config) {
        state_temp.head(this->dof) = this->in_robotstatus_var.angles;
        state_temp.tail(this->dof).setZero();// = this->in_robotstatus_var.velocities;
        std::cout << "Velocities:" << this->in_robotstatus_var.velocities.transpose() << std::endl;
        std::cout << "SET INITIAL STATE!" << std::endl;
        std::cout << state_temp.transpose() << std::endl;
        // std::cout<<"SET INITIAL STATE!"<<std::endl;

        std::cout << "PREINITIAL STATE: " << jda->getInitialState().transpose() << std::endl;
        jda->setInitialState(state_temp);
        std::cout << "INITIAL STATE: " << jda->getInitialState().transpose() << std::endl;
        first_config = true;

        JointTrajectoryOrocos::updateHook();
        std::cout << "INITIAL STATE UPDATE: " << jda->getInitialState().transpose() << std::endl;
        return;
    } else if (this->in_robotstatus_flow == RTT::NoData) {
        return;
    }
    JointTrajectoryOrocos::updateHook();
    // if (jda->finished) {
    //     first_config = false;
    //     this->stop();
    // }
}
ORO_CREATE_COMPONENT_LIBRARY() ORO_LIST_COMPONENT_TYPE(JointDynamicAttractorOrocos)
