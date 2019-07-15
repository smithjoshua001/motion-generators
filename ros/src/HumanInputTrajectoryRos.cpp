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
    maxVel.setValue(0.01);
    maxDistance.setValue(0.1);
    transitionTime.setValue(5);
    enable = false;
    this->addParametersToHandler("");
    timeAccum =0;
}

void HumanInputTrajectoryRos::prepareRos(ros::NodeHandle &nh) {
    joystick_sub = nh.subscribe("/HIT/" + name + "/joy_update", 1, &HumanInputTrajectoryRos::updateJoystick, this);
    outputPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/HIT/" + name + "/output_pose", 1);
}

void HumanInputTrajectoryRos::update(double dt) {
    currentPose.head<3>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getPositionWorldToEndEffectorInWorldFrame().vector();
    currentPose.tail<4>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getOrientationWorldToEndEffector().vector();

    HumanInputTrajectory<double>::maxDistance = maxDistance.getValue();
    if (enable && timeAccum <= transitionTime.getValue()) {
        timeAccum += dt;
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setShouldBeGrounded(true);
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setStancePhase(timeAccum / (timeAccum + (((transitionTime.getValue() * 0.80) - ((transitionTime.getValue() * 0.80) * 0.99)) / 0.99)));
        desiredPose.head<3>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getPositionWorldToEndEffectorInWorldFrame().vector();
        desiredPose.tail<4>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getOrientationWorldToEndEffector().vector();
	desiredPose.tail<3>() = -desiredPose.tail<3>();
    }
    if (enable && timeAccum > transitionTime.getValue()) {
        std::lock_guard<std::mutex> guard(desLock);
        HumanInputTrajectory<double>::update(dt);

        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setShouldBeGrounded(false);
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(0.0);

    }
    if(enable){
        model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(kindr::Position3D(desiredPose.head<3>()));
        std::cout<<"MOTION GENERATOR OUTPUT:"<<std::endl<<model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateDesiredPtr()->getPositionWorldToEndEffectorInWorldFrame().vector().transpose()<<std::endl<<std::endl;

        loco::RotationQuaternion rotation;
        rotation.w() = this->desiredPose[3];
        rotation.x() = -this->desiredPose[4];
        rotation.y() = -this->desiredPose[5];
        rotation.z() = -this->desiredPose[6];
        model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateDesiredPtr()->setOrientationWorldToEndEffector(rotation);
        loco::RotationQuaternion rotInvert = rotation.inverted();
        geometry_msgs::PoseStamped desiredPose;
        desiredPose.header.frame_id = "odom";
        desiredPose.header.stamp = ros::Time::now();
        desiredPose.pose.position.x = this->desiredPose[0];
        desiredPose.pose.position.y = this->desiredPose[1];
        desiredPose.pose.position.z = this->desiredPose[2];
        desiredPose.pose.orientation.w = rotation.w();
        desiredPose.pose.orientation.x = -rotation.x();
        desiredPose.pose.orientation.y = -rotation.y();
        desiredPose.pose.orientation.z = -rotation.z();
        outputPose_pub.publish(desiredPose);
        
    }
    
}
void HumanInputTrajectoryRos::reset() {
    HumanInputTrajectory<double>::reset();
    std::lock_guard<std::mutex> guard(desLock);
    currentPose.head<3>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getPositionWorldToEndEffectorInWorldFrame().vector();
    currentPose.tail<4>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getOrientationWorldToEndEffector().vector();
    currentPose.tail<3>()=-currentPose.tail<3>();
    desiredPose = currentPose;

    // model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(0.0);
}
using QD = quadruped_model::QuadrupedModel::QuadrupedDescription;
void HumanInputTrajectoryRos::updateJoystick(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    
    if (joy_msg->buttons.size()>0 && joy_msg->buttons[0]) {
        reset();
        /*if (model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->isGrounded()) {
            timeAccum = 0;
        }*/
        enable = true;
    }
    if (joy_msg->buttons.size()>1 && joy_msg->buttons[1]) {
        enable = false;
    }
    
    if (enable && joy_msg->axes.size()>=6) {
        std::lock_guard<std::mutex> guard(desLock);
        desiredPose[0] += joy_msg->axes[0] * maxVel.getValue();
        desiredPose[1] += joy_msg->axes[1] * maxVel.getValue();
        desiredPose[2] += joy_msg->axes[2] * maxVel.getValue();
        if(joy_msg->axes.size()>3){
            Eigen::Vector3d axes;
	    axes[0] = joy_msg->axes[3];axes[1] = joy_msg->axes[4];axes[2] = joy_msg->axes[5];

            Eigen::Quaterniond quat(desiredPose[3],desiredPose[4],desiredPose[5],desiredPose[6]);
            //axes=quat._transformVector(axes);

            //Eigen::Quaterniond quat_dot(0,joy_msg->axes[3] * maxVel.getValue(),joy_msg->axes[4] * maxVel.getValue(),joy_msg->axes[5]  * maxVel.getValue());
Eigen::Quaterniond quat_dot(0,axes[0] * maxVel.getValue(),axes[1] * maxVel.getValue(),axes[2]  * maxVel.getValue());
            // std::cout<<"QUAT DOT:: "<<quat_dot.coeffs().transpose()<<std::endl;
            quat_dot = (quat*quat_dot);
            quat_dot.coeffs()*=0.5;
            // std::cout<<quat_dot.coeffs().transpose()<<", "<<quat_dot.w()<<std::endl;
            desiredPose[3] += quat_dot.w();
            desiredPose[4] += quat_dot.x();
            desiredPose[5] += quat_dot.y();
            desiredPose[6] += quat_dot.z();
        }
        // std::cout<<desiredPose.transpose()<<std::endl;
    }
}

bool HumanInputTrajectoryRos::addParametersToHandler(const std::string &ns) {
    if (!parameter_handler::handler->addParam(ns + "/HIT/" + name + "/Leg", swing_leg) || !parameter_handler::handler->addParam(ns + "/HIT/" + name + "/maxVel", maxVel) || !parameter_handler::handler->addParam(ns + "/HIT/" + name + "/maxDistance", maxDistance) || !parameter_handler::handler->addParam(ns + "/HIT/" + name + "/transitionTime", transitionTime)) {
        return false;
    }
    return true;
}
