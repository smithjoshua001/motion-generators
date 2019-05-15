#include "motion-generators/CJDAOrocos.hpp"

CJDAOrocos::CJDAOrocos(std::string const &name) : RTT::TaskContext(name),
    // jda(),
    JointTrajectoryOrocos(this, std::static_pointer_cast<JointTrajectory<float> >(std::make_shared<JointDynamicAttractor<float> >())) {
    first_config = false;
    jda = std::static_pointer_cast<JointDynamicAttractor<float> >(this->getTrajectory());
    addOperation("setGain", &JointDynamicAttractor<float>::setGain, jda.get());
    addOperation("setLimit", &JointDynamicAttractor<float>::setLimit, jda.get());
    addOperation("goToStop", &CJDAOrocos::goToStop,this);
    stop = false;
}

bool CJDAOrocos::configureHook() {
    bool temp = JointTrajectoryOrocos::configureHook();
    if (temp) {
        in_robotstatus_var = rstrt::robot::JointState(this->dof);
        in_robotstatus_port.setName("in_robotstatus_port");
        in_robotstatus_port.doc("Input port for reading the robotstatuss for all chains");
        task->ports()->addPort(in_robotstatus_port);
        in_robotstatus_flow = RTT::NoData;

	in_despos_var = rstrt::kinematics::JointAngles(this->dof);
        in_despos_port.setName("in_despos_port");
        in_despos_port.doc("Input port for reading the robotstatuss for all chains");
        task->ports()->addPort(in_despos_port);
        in_despos_flow = RTT::NoData;

	in_desvel_var = rstrt::kinematics::JointVelocities(this->dof);
        in_desvel_port.setName("in_desvel_port");
        in_desvel_port.doc("Input port for reading the desvels for all chains");
        task->ports()->addPort(in_desvel_port);
        in_desvel_flow = RTT::NoData;

	in_desacc_var = rstrt::kinematics::JointAccelerations(this->dof);
        in_desacc_port.setName("in_desacc_port");
        in_desacc_port.doc("Input port for reading the desaccs for all chains");
        task->ports()->addPort(in_desacc_port);
        in_desacc_flow = RTT::NoData;
	
        state_temp.resize(this->dof*2);
        state_temp.setZero();
        stop = false;
    }
    return temp;
}
void CJDAOrocos::updateHook() {
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
         std::cout<<"SET INITIAL STATE!"<<std::endl;
         std::cout<<this->in_robotstatus_var.angles.transpose()<<std::endl;
        jda->setInitialState(state_temp);
        first_config = true;
    } else if (this->in_robotstatus_flow == RTT::NoData) {
        return;
    }
    if(this->in_despos_port.connected() && this->in_desvel_port.connected() && this->in_desacc_port.connected() && !stop){
        this->in_despos_flow = this->in_despos_port.read(this->in_despos_var);
        this->in_desvel_flow = this->in_desvel_port.read(this->in_desvel_var);
        this->in_desacc_flow = this->in_desacc_port.read(this->in_desacc_var);
        if(in_despos_flow != RTT::NoData){
            jda->getDesiredState().head(dof) = in_despos_var.angles;
            jda->getDesiredState().tail(dof) = in_desvel_var.velocities;
            jda->setAccelerationOffset(in_desacc_var.accelerations);
        }
    }
    JointTrajectoryOrocos::updateHook();
    if (jda->finished && stop) {
        first_config = false;
        this->RTT::TaskContext::stop();
    }
}

void CJDAOrocos::goToStop() {
    state_temp.setZero();
    jda->getDesiredState()=state_temp;
    jda->setAccelerationOffset(state_temp.head(dof));
    stop=true;
    jda->finished = false;
}
ORO_LIST_COMPONENT_TYPE(CJDAOrocos)
