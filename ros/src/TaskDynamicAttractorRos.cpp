#include <motion-generators/TaskDynamicAttractorRos.hpp>

using namespace motion_generators::taskspace;

TaskDynamicAttractorRos::TaskDynamicAttractorRos(std::string const &name, ros::NodeHandle &nh, loco::WholeBody *model) : TaskDynamicAttractor(),
    tfListener(tfBuffer) {
    this->name = name;
    std::cout << "SETTING MODEL!!!" << std::endl;
    this->model = model;
    if (!this->model) {
        std::cout << "NO MODEL??" << std::endl;
        exit(-1);
    }
    prepareRos(nh);
    setGains(1, 1);
    gain.setValue(1);
    orientation_gain.setValue(1);
}

void TaskDynamicAttractorRos::prepareRos(ros::NodeHandle &nh) {
    desiredPose_sub = nh.subscribe("/TDA/" + name + "/target_pose", 1, &TaskDynamicAttractorRos::setDesiredPose, this);
    outputPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/TDA/" + name + "/output_pose", 1);
}

void TaskDynamicAttractorRos::update(double dt) {
    // std::cout << "UPDATE!!" << std::endl;
    if (first_config && go && started_contact && time_elapsed < 0.5) {
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setShouldBeGrounded(true);
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setStancePhase(1.0);
        time_elapsed += dt;
        return;
    }
    TaskDynamicAttractor::update(dt);
    if (first_config && go) {
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setShouldBeGrounded(false);
        // if (started_contact) {
        //     std::cout << state_error.head<7>().norm() - 1 << std::endl;
        //     // model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->getSwingPhase() + ((((state_dot.head<7>()).norm()) * dt) / maxError));
        //     model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(((state_error.head<7>().norm() - 1) / maxError));
        // } else {
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(0.0);
        // }
        // model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(1 - ((des_state - state).norm() / maxError));
        // model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(1.0);
        std::cout << "SWING PHASE! :" << model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->getSwingPhase() << std::endl;

        model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(kindr::Position3D(state.head<3>()));

        geometry_msgs::PoseStamped desiredPose;
        desiredPose.header.frame_id = "map";
        desiredPose.header.stamp = ros::Time::now();
        desiredPose.pose.position.x = state[0];
        desiredPose.pose.position.y = state[1];
        desiredPose.pose.position.z = state[2];
        desiredPose.pose.orientation.w = state[3];
        desiredPose.pose.orientation.x = state[4];
        desiredPose.pose.orientation.y = state[5];
        desiredPose.pose.orientation.z = state[6];
        outputPose_pub.publish(desiredPose);
    }
}
void TaskDynamicAttractorRos::reset() {
    TaskDynamicAttractor::reset();
    model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(0.0);
}
using QD = quadruped_model::QuadrupedModel::QuadrupedDescription;
void TaskDynamicAttractorRos::setDesiredPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::cout << "RECEIVED DESIRED POSE!!" << std::endl;
    geometry_msgs::PoseStamped desiredPose;
    try{
        std::cout << "TRANSFORM DESIRED POSE TO MAP!!" << std::endl;
        desiredPose = tfBuffer.transform(*msg, "map", ros::Time(0), "map");
        std::cout << "SETTING DESIRED STATE!!" << std::endl;
        des_state.setZero();
        des_state[0] = desiredPose.pose.position.x;
        des_state[1] = desiredPose.pose.position.y;
        des_state[2] = desiredPose.pose.position.z;
        des_state[3] = desiredPose.pose.orientation.w;
        des_state[4] = desiredPose.pose.orientation.x;
        des_state[5] = desiredPose.pose.orientation.y;
        des_state[6] = desiredPose.pose.orientation.z;
        std::cout << "SET DESIRED STATE!!" << std::endl;

        std::cout << "SETTING CURRENT STATE!!" << std::endl;
        if (!this->model) {
            std::cout << "NO MODEL FAILURE!!!" << std::endl;
        }
        std::cout << model->getLimbsPtr()->size() << std::endl;
        state.head<3>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getPositionWorldToEndEffectorInWorldFrame().vector();
        std::cout << "SET CURRENT STATE POS!!" << std::endl;
        state.segment<4>(3) = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getOrientationWorldToEndEffector().vector();
        std::cout << "SET CURRENT STATE ORI!!" << std::endl;

        state.segment<3>(7) = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getLinearVelocityEndEffectorInWorldFrame().vector();
        std::cout << "SET CURRENT STATE LVEL!!" << std::endl;
        //QD::mapKeyNameToKeyId<QD::LimbEnum>(swing_leg.getValue())
        state.segment<3>(10) = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getAngularVelocityEndEffectorInWorldFrame().vector();
        std::cout << "SET CURRENT STATE!!" << std::endl;

        started_contact = model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->isGrounded();
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(0.0);
        // model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setStancePhase(1.0);
        std::cout << "SETTING GAINS!!" << std::endl;
        TaskDynamicAttractor::setGains(gain.getValue(), orientation_gain.getValue());
        time_elapsed = 0;
        std::cout << "PASS DES AND STATE!!" << std::endl;

        TaskDynamicAttractor::setInitialState(state);
        TaskDynamicAttractor::setDesiredPose(des_state);
    }catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
}

bool TaskDynamicAttractorRos::addParametersToHandler(const std::string &ns) {
    if (!parameter_handler::handler->addParam(ns + "/TDA/" + name + "/Leg", swing_leg) || !parameter_handler::handler->addParam(ns + "/TDA/" + name + "/LinearGain", gain) || !parameter_handler::handler->addParam(ns + "/TDA/" + name + "/OrientationGain", orientation_gain)) {
        return false;
    }
    return true;
}
