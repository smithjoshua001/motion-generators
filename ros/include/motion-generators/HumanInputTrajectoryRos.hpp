#pragma once

#include "motion-generators/HumanInputTrajectory.hpp"

#include <ros/ros.h>
#include <parameter_handler/parameter_handler.hpp>
#include <string>
#include <Eigen/Dense>
#include <kindr/Core>
#include <loco/common/loco_common.hpp>
#include <quadruped_model/quadruped_model.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Joy.h>
#include <mutex>

namespace motion_generators {
    namespace taskspace {
        class HumanInputTrajectoryRos : public HumanInputTrajectory<double> {
        public:
            HumanInputTrajectoryRos(std::string const &name, ros::NodeHandle &nh, loco::WholeBody *model);

            ~HumanInputTrajectoryRos() {}

            bool addParametersToHandler(const std::string &ns);

            void update(double dt) override;

            void prepareRos(ros::NodeHandle &nh);

            void reset() override;

            void updateJoystick(const sensor_msgs::Joy::ConstPtr &joy_msg);

        private:
            parameter_handler::Parameter<double> maxVel, maxDistance, transitionTime;

            kindr::HomTransformQuatD pose;
            kindr::TwistGlobalD Twist;

            std::string name;

            parameter_handler::Parameter<int> swing_leg;

            ros::Subscriber joystick_sub;
            ros::Publisher outputPose_pub;

            loco::WholeBody *model;

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener;
            bool started_contact;
            double timeAccum;
            bool enable;
            std::mutex desLock;
        };
    }
}