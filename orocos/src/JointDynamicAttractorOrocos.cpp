#include "motion-generators/JointDynamicAttractorOrocos.hpp"

JointDynamicAttractorOrocos::JointDynamicAttractorOrocos(std::string const &name) : RTT::TaskContext(name),
    // jda(),
    JointTrajectoryOrocos(this, std::static_pointer_cast<JointTrajectory<float> >(std::make_shared<JointDynamicAttractor<float> >())) {
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
        task->ports()->addPort(in_robotstatus_port);
        in_robotstatus_flow = RTT::NoData;
        state_temp.resize(this->dof*2);
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
        state_temp.tail(this->dof) = this->in_robotstatus_var.velocities;
        // std::cout<<"SET INITIAL STATE!"<<std::endl;
        jda->setInitialState(state_temp);
        first_config = true;
    } else if (this->in_robotstatus_flow == RTT::NoData) {
        return;
    }
    JointTrajectoryOrocos::updateHook();
    if (jda->finished) {
        first_config = false;
        this->stop();
    }
}
ORO_CREATE_COMPONENT_LIBRARY() ORO_LIST_COMPONENT_TYPE(JointDynamicAttractorOrocos)
