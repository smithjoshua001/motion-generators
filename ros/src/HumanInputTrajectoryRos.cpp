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

    //
    sigma_time_ = ros::Time::now().toSec();
    current_pose_sigma_ << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    desired_pose_sigma_ << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
}

void HumanInputTrajectoryRos::prepareRos(ros::NodeHandle &nh) {
    joystick_sub = nh.subscribe("/HIT/" + name + "/joy_update", 1, &HumanInputTrajectoryRos::updateJoystick, this);
    outputPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/HIT/" + name + "/output_pose", 1);

    //
    sigma_sub_ = nh.subscribe("/pose_sigma_x", 1, &HumanInputTrajectoryRos::sigmaCallback, this);
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
    }
    if (enable && timeAccum > transitionTime.getValue()) {
        std::lock_guard<std::mutex> guard(desLock);
        HumanInputTrajectory<double>::update(dt);

        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setShouldBeGrounded(false);
        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(0.0);

        model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(kindr::Position3D(desiredPose.head<3>()));
        loco::RotationQuaternion rotation;
        rotation.w() = this->desiredPose[3];
        rotation.x() = this->desiredPose[4];
        rotation.y() = this->desiredPose[5];
        rotation.z() = this->desiredPose[6];
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
    // model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setSwingPhase(0.0);
}
using QD = quadruped_model::QuadrupedModel::QuadrupedDescription;
void HumanInputTrajectoryRos::updateJoystick(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    
    if (joy_msg->buttons.size()>0 && joy_msg->buttons[0]) {
        reset();
        if (model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->isGrounded()) {
            timeAccum = 0;
        }
        enable = true;
    }
    if (joy_msg->buttons.size()>0 && joy_msg->buttons[1]) {
        enable = false;
    }
    
    if (enable &&  !use_sigma_haptic_) {
        std::lock_guard<std::mutex> guard(desLock);
        desiredPose[0] += joy_msg->axes[0] * maxVel.getValue();
        desiredPose[1] += joy_msg->axes[1] * maxVel.getValue();
        desiredPose[2] += joy_msg->axes[2] * maxVel.getValue();
        if(joy_msg->axes.size()>3){
            
            Eigen::Quaterniond quat_dot(0,joy_msg->axes[3]*0 * maxVel.getValue(),joy_msg->axes[4] * maxVel.getValue(),joy_msg->axes[5]  * maxVel.getValue());
            // std::cout<<"QUAT DOT:: "<<quat_dot.coeffs().transpose()<<std::endl;
            Eigen::Quaterniond quat(desiredPose[3],desiredPose[4],desiredPose[5],desiredPose[6]);
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

void HumanInputTrajectoryRos::sigmaCallback(const geometry_msgs::PoseStamped::ConstPtr & pose_msg){

	if(use_sigma_haptic_){// if desired pose not initialized or if timeout occured
		if((current_pose_sigma_.head(3).sum() == 0 && current_pose_sigma_.tail(3).sum() == 0 && current_pose_sigma_[3] == 1) || pose_msg->header.stamp.sec - sigma_time_ >= sigma_timeout_){
	        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setShouldBeGrounded(true);
	        model->getLegsPtr()->getPtr(swing_leg.getValue())->getContactSchedulePtr()->setStancePhase(timeAccum / (timeAccum + (((transitionTime.getValue() * 0.80) - ((transitionTime.getValue() * 0.80) * 0.99)) / 0.99)));
	        current_pose_sigma_.head<3>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getPositionWorldToEndEffectorInWorldFrame().vector();
	        current_pose_sigma_.tail<4>() = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getOrientationWorldToEndEffector().vector();

	        ROS_ERROR_STREAM("(re-)set initial desired pose (difference = " << pose_msg->header.stamp.sec - sigma_time_ << " time out is " << sigma_timeout_);
	        ROS_ERROR_STREAM("init desired pose with initial pose of " << desiredPose.head(3));
	        ROS_ERROR_STREAM("and orientation of " << desiredPose.tail(4));
	    }

//		const kindr::RotationQuaternionPD & worldToBase = model->getTorsoPtr()->getMeasuredStatePtr()->getOrientationWorldToBase();
//		const kindr::RotationQuaternionPD & worldToEE = model->getLimbsPtr()->getPtr(swing_leg.getValue())->getEndEffectorPtr()->getStateMeasuredPtr()->getOrientationWorldToEndEffector();
//		const kindr::RotationQuaternionPD & baseToEE = worldToEE * worldToBase.inverted();

		std::lock_guard<std::mutex> guard(desLock);

		// TODO reset desired to current after timeout -> when button released no publication of topic
		// TODO add orientation

		bool output_position = false, output_orientation = false;

		ROS_WARN_COND(output_position, " ");
		ROS_WARN_COND(output_position, "Old des position is [%4.3f, %4.3f, %4.3f]", desiredPose[0], desiredPose[1], desiredPose[2]);
		ROS_WARN_COND(output_position, "Command add on position is [%4.3f, %4.3f, %4.3f]", pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);

		desiredPose[0] = current_pose_sigma_[0] + pose_msg->pose.position.x;
        desiredPose[1] = current_pose_sigma_[1] + pose_msg->pose.position.y;
        desiredPose[2] = current_pose_sigma_[2] + pose_msg->pose.position.z;

        ROS_WARN_COND(output_position, "New des position is [%4.3f, %4.3f, %4.3f]", desiredPose[0], desiredPose[1], desiredPose[2]);

        ROS_WARN_COND(output_orientation, " ");
        ROS_WARN_COND(output_orientation, "Old des position is [%4.2f, %4.2f, %4.2f, %4.2f]", desiredPose[3], desiredPose[4], desiredPose[5], desiredPose[6]);
        ROS_WARN_COND(output_orientation, "Command add on is [%4.2f, %4.2f, %4.2f, %4.2f]", pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);

        Eigen::Quaterniond quat_dot(pose_msg->pose.orientation.w , 0 * pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
        Eigen::Quaterniond quat(current_pose_sigma_[3],current_pose_sigma_[4],current_pose_sigma_[5],current_pose_sigma_[6]);
        quat_dot = (quat*quat_dot);
        quat_dot.coeffs()*=0.5;

        desiredPose[3] = current_pose_sigma_[3] + quat_dot.w();
        desiredPose[4] = current_pose_sigma_[4] + quat_dot.x();
        desiredPose[5] = current_pose_sigma_[5] + quat_dot.y();
        desiredPose[6] = current_pose_sigma_[6] + quat_dot.z();

        ROS_WARN_COND(output_orientation, "New des position is [%4.2f, %4.2f, %4.2f, %4.2f]", desiredPose[3], desiredPose[4], desiredPose[5], desiredPose[6]);

	}
	sigma_time_ = pose_msg->header.stamp.sec;
}

bool HumanInputTrajectoryRos::addParametersToHandler(const std::string &ns) {
    if (!parameter_handler::handler->addParam(ns + "/HIT/" + name + "/Leg", swing_leg) || !parameter_handler::handler->addParam(ns + "/HIT/" + name + "/maxVel", maxVel) || !parameter_handler::handler->addParam(ns + "/HIT/" + name + "/maxDistance", maxDistance) || !parameter_handler::handler->addParam(ns + "/HIT/" + name + "/transitionTime", transitionTime)) {
        return false;
    }
    return true;
}
