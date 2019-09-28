#pragma once

#include "motion-generators/TaskDynamicAttractor.hpp"

#include <ros/ros.h>
#include <parameter_handler/parameter_handler.hpp>
#include <string>
#include <Eigen/Dense>
#include <kindr/Core>
#include <loco/common/loco_common.hpp>
#include <anymal_model/AnymalModel.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace motion_generators {
    namespace taskspace {
        class TaskDynamicAttractorRos : public TaskDynamicAttractor {
        public:
            TaskDynamicAttractorRos(std::string const &name, ros::NodeHandle &nh, loco::WholeBody *model);

            bool startHook() {start_time = ros::Time::now().toSec(); return true;}
            bool addParametersToHandler(const std::string &ns);

            void update(double dt) override;

            void prepareRos(ros::NodeHandle &nh);

            void setDesiredPose(const geometry_msgs::PoseStamped::ConstPtr &desiredState_msg);

            template<typename Derived, typename Derived2> static void skewMatrix(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> &skewMat) {
                skewMat.setZero();
                skewMat(2, 1) = vector(0);
                skewMat(1, 2) = -vector(0);
                skewMat(0, 2) = vector(1);
                skewMat(2, 0) = -vector(1);
                skewMat(0, 1) = -vector(2);
                skewMat(1, 0) = vector(2);
            }

            void reset() override;

        private:
            parameter_handler::Parameter<double> gain, orientation_gain;

            kindr::HomTransformQuatD pose;
            kindr::TwistGlobalD Twist;

            std::string name;

            parameter_handler::Parameter<int> swing_leg;

            ros::Subscriber desiredPose_sub;
            ros::Publisher outputPose_pub;

            loco::WholeBody *model;

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener;
            bool started_contact;
            double time_elapsed;
        };
    }
}
