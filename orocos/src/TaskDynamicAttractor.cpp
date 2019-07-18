#include "motion-generators/TaskDynamicAttractor.hpp"
using namespace motion_generators::taskspace;
TaskDynamicAttractor::TaskDynamicAttractor(std::string const &name) : RTT::TaskContext(name) {
    addOperation("setGain", &TaskDynamicAttractor::setGain, this).doc("set Gains");
    addOperation("setDesiredState", &TaskDynamicAttractor::setDesiredState, this).doc("setDesiredState");
    addProperty("go", go);
    portsArePrepared = false;
    first_config = false;
    go = false;

    state.resize(13);
    state.setZero();
    state_dot.resize(13);
    state_dot.setZero();

    des_state.resize(13);
    des_state.setZero();
    des_state_dot.resize(13);
    des_state_dot.setZero();

    gainMatrix.resize(6, 13);
    gainMatrix.setZero();

    state_error.resize(13);
    state_error.setZero();
    quat_left.resize(4, 4);
    quat_left.setZero();
    preparePorts();
}

void TaskDynamicAttractor::preparePorts() {
    if (portsArePrepared) {
        ports()->removePort("in_pose_port");
        ports()->removePort("in_twist_port");
        ports()->removePort("out_pose_port");
        ports()->removePort("out_twist_port");
    }

    //prepare input
    in_pose_var = rstrt::geometry::Pose();
    in_pose_port.setName("in_pose_port");
    in_pose_port.doc("Input port for reading pose values");
    ports()->addPort(in_pose_port);
    in_pose_flow = RTT::NoData;

    in_twist_var = rstrt::kinematics::Twist();
    in_twist_port.setName("in_twist_port");
    in_twist_port.doc("Input port for reading twist values");
    ports()->addPort(in_twist_port);
    in_twist_flow = RTT::NoData;

    //prepare output
    out_pose_var = rstrt::geometry::Pose();
    out_pose_port.setName("out_pose_port");
    out_pose_port.doc("Output port for sending pose values");
    out_pose_port.setDataSample(out_pose_var);
    ports()->addPort(out_pose_port);

    out_twist_var = rstrt::kinematics::Twist();
    out_twist_port.setName("out_twist_port");
    out_twist_port.doc("Output port for sending twist values");
    out_twist_port.setDataSample(out_twist_var);
    ports()->addPort(out_twist_port);

    out_acceleration_var = Eigen::VectorXf(6);
    out_acceleration_var.setZero();
    out_acceleration_port.setName("out_acceleration_port");
    out_acceleration_port.doc("Output port for sending acceleration values");
    out_acceleration_port.setDataSample(out_acceleration_var);
    ports()->addPort(out_acceleration_port);

    portsArePrepared = true;
}

bool TaskDynamicAttractor::configureHook() {
    return portsArePrepared;
}
void TaskDynamicAttractor::updateHook() {
    if (in_pose_port.connected() && in_twist_port.connected() && (!first_config || !go)) {
        in_pose_flow = in_pose_port.read(in_pose_var);
        in_twist_flow = in_twist_port.read(in_twist_var);
    } else {
        // return;
    }
    if (in_pose_flow != RTT::NoData && in_twist_flow != RTT::NoData && (!first_config || !go)) {
        state.head<3>() = in_pose_var.translation.translation;
        state.segment<4>(3) = in_pose_var.rotation.rotation;
        state.segment<3>(7) = in_twist_var.linear;
        state.tail(3) = in_twist_var.angular;
        first_config = true;
    } else if (in_pose_flow == RTT::NoData || in_twist_flow == RTT::NoData) {
        return;
    }
    if (first_config && go) {
        compute();
    }
    out_pose_var.translation.translation = state.head<3>();
    out_pose_var.rotation.rotation = state.segment<4>(3);
    out_twist_var.linear = state.segment<3>(7);
    out_twist_var.angular = state.segment<3>(10);
    out_acceleration_var = state_dot.segment<6>(7);
    out_pose_port.write(out_pose_var);
    out_twist_port.write(out_twist_var);
    out_acceleration_port.write(out_acceleration_var);
}

void TaskDynamicAttractor::setGain(float gain, float orientation_gain) {
    gainMatrix.block(0, 0, 3, 3).setIdentity();
    gainMatrix.block(3, 4, 3, 3).setIdentity();
    gainMatrix.block(3, 4, 3, 3) *= -orientation_gain * orientation_gain;

    gainMatrix.block(0, 0, 3, 3) *= -gain * gain;
    gainMatrix.block(0, 7, 6, 6).setIdentity();
    gainMatrix.block(0, 7, 3, 3) *= -2 * gain;
    gainMatrix.block(3, 10, 3, 3) *= -2 * orientation_gain;
}
void TaskDynamicAttractor::compute() {
    if (state.segment<4>(3).transpose() * des_state.segment<4>(3) < 0) {
        des_state.segment<4>(3) = -des_state.segment<4>(3);
    }
    skewMatrix(des_state.segment<3>(4), skewMat);
    quat_left.block<3, 3>(1, 1) = -skewMat + des_state(3) * Eigen::Matrix3f::Identity();
    quat_left.block<3, 1>(1, 0) = -des_state.segment<3>(4);
    quat_left.block<1, 3>(0, 1) = des_state.segment<3>(4).transpose();
    quat_left(0, 0) = des_state(3);
    state_error = state - des_state;
    state_error.segment<4>(3) = quat_left * state.segment<4>(3);

    Eigen::Quaternionf temp(des_state(3), des_state(4), des_state(5), des_state(6));
    Eigen::Quaternionf temp2(state(3), state(4), state(5), state(6));
    Eigen::Quaternionf temp3 = temp2 * temp.inverse();//temp.inverse()*temp2;
    temp3.normalize();
    //std::cout<<state_error.segment<4>(3).transpose()<<" : "<<temp3.w()<<", "<<temp3.x()<<", "<<temp3.y()<<", "<<temp3.z()<<"\n";

    state_error(3) = temp3.w();
    state_error(4) = temp3.x();
    state_error(5) = temp3.y();
    state_error(6) = temp3.z();

    quat_left.block<3, 3>(1, 1) = skewMat + des_state(3) * Eigen::Matrix3f::Identity();
    quat_left.block<3, 1>(1, 0) = des_state.segment<3>(4);
    quat_left.block<1, 3>(0, 1) = -des_state.segment<3>(4).transpose();
    quat_left(0, 0) = des_state(3);

    Eigen::MatrixXf omega_mat(4, 4);
    omega_mat.setZero();
    skewMatrix(state.segment<3>(10), skewMat);
    omega_mat.block<3, 3>(1, 1) = -skewMat;
    omega_mat.block<3, 1>(1, 0) = state.segment<3>(10);
    omega_mat.block<1, 3>(0, 1) = -state.segment<3>(10).transpose();
    state_dot.segment<4>(3) = (0.5 * (quat_left * omega_mat) * (state_error.segment<4>(3)));
    state_dot.segment<3>(0) = state_error.segment<3>(7);

    Eigen::Quaternionf temp4(0, state(10), state(11), state(12));
    Eigen::Quaternionf temp5(state_error(3), state_error(4), state_error(5), state_error(6));
    Eigen::Quaternionf temp6 = (temp4) * (temp5 * temp);
    //state_dot.segment<4>(3) = 0.5*;
    state_dot(3) = 0.5 * temp6.w();
    state_dot(4) = 0.5 * temp6.x();
    state_dot(5) = 0.5 * temp6.y();
    state_dot(6) = 0.5 * temp6.z();

    state_dot.tail<6>() = gainMatrix * state_error;
    state = state + state_dot * 0.001;
    // RTT::log(RTT::Error) << state.transpose() << RTT::endlog();
    // RTT::log(RTT::Error) << des_state.transpose() << RTT::endlog();
    // RTT::log(RTT::Error) << state_dot.transpose() << RTT::endlog();
}

void TaskDynamicAttractor::setDesiredState(Eigen::VectorXf &desiredState) {
    des_state = desiredState;
}

ORO_LIST_COMPONENT_TYPE(TaskDynamicAttractor)
