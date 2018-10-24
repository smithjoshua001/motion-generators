#include <motion-generators/HumanInputTrajectoryRos.hpp>

using namespace motion_generators::taskspace;

HumanInputTrajectoryRos::HumanInputTrajectoryRos(std::string const &name, ros::NodeHandle &nh, loco::WholeBody *model) : HumanInputTrajectory<double>(),
    tfListener(tfBuffer) {
    this->name = name;
    std::cout << "SETTING MODEL!!!" << std::endl;
    this->model = model;
    if (!this->model) {
        std::cout << "NO MODEL??" << std::endl;
        exit(-1);
    }
    prepareRos(nh);
    maxVel.setValue(0.001);
    maxDistance.setValue(0.1);
    transitionTime.setValue(5);
    enable = false;
}

void HumanInputTrajectoryRos::prepareRos(ros::NodeHandle &nh) {
    joystick_sub = nh.subscribe("/HIT/" + name + "/joy_update", 1, &HumanInputTrajectoryRos::updateJoystick, this);
    outputPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/HIT/" + name + "/output_pose", 1);
}

void HumanInputTrajectoryRos::update(double dt) {
    currentPose.head<3>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getPositionWorldToEndEffectorInWorldFrame().vector();
    HumanInputTrajectory<double>::maxDistance = maxDistance.getValue();
    if (enable && timeAccum <= transitionTime.getValue()) {
        timeAccum += dt;
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setShouldBeGrounded(true);
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setStancePhase(timeAccum / (timeAccum + (((transitionTime.getValue() * 0.80) - ((transitionTime.getValue() * 0.80) * 0.99)) / 0.99)));
        desiredPose.head<3>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getPositionWorldToEndEffectorInWorldFrame().vector();
    }
    if (enable && timeAccum > transitionTime.getValue()) {
        std::lock_guard<std::mutex> guard(desLock);
        HumanInputTrajectory<double>::update(dt);

        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setShouldBeGrounded(false);
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(0.0);

        model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(kindr::Position3D(desiredPose.head<3>()));

        geometry_msgs::PoseStamped desiredPose;
        desiredPose.header.frame_id = "map";
        desiredPose.header.stamp = ros::Time::now();
        desiredPose.pose.position.x = this->desiredPose[0];
        desiredPose.pose.position.y = this->desiredPose[1];
        desiredPose.pose.position.z = this->desiredPose[2];
        desiredPose.pose.orientation.w = 1;
        desiredPose.pose.orientation.x = 0;
        desiredPose.pose.orientation.y = 0;
        desiredPose.pose.orientation.z = 0;
        outputPose_pub.publish(desiredPose);
    }
}
void HumanInputTrajectoryRos::reset() {
    HumanInputTrajectory<double>::reset();
    std::lock_guard<std::mutex> guard(desLock);
    currentPose.head<3>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getPositionWorldToEndEffectorInWorldFrame().vector();
    model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(0.0);
}
using QD = quadruped_model::QuadrupedModel::QuadrupedDescription;
void HumanInputTrajectoryRos::updateJoystick(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    if (joy_msg->buttons[0]) {
        reset();
        if (model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->isGrounded()) {
            timeAccum = 0;
        }
        enable = true;
    }
    if (joy_msg->buttons[1]) {
        enable = false;
    }

    if (enable) {
        std::lock_guard<std::mutex> guard(desLock);
        desiredPose[0] += joy_msg->axes[0] * maxVel.getValue();
        desiredPose[1] += joy_msg->axes[1] * maxVel.getValue();
        desiredPose[2] += joy_msg->axes[2] * maxVel.getValue();
    }
}

bool HumanInputTrajectoryRos::addParametersToHandler(const std::string &ns) {
    if (!parameter_handler::handler->addParam(ns + "/HIT/" + name + "/Leg", swing_leg) || !parameter_handler::handler->addParam(ns + "/HIT/" + name + "/maxVel", maxVel) || !parameter_handler::handler->addParam(ns + "/HIT/" + name + "/maxDistance", maxDistance) || !parameter_handler::handler->addParam(ns + "/HIT/" + name + "/transitionTime", transitionTime)) {
        return false;
    }
    return true;
}
