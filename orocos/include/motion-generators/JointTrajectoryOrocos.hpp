#pragma once
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/kinematics/JointAccelerations.hpp>
#include <CosimaUtilities/Timing.hpp>
#include <memory>

class JointTrajectoryOrocos {
protected:
    RTT::TaskContext *task;

    size_t dof;
    std::shared_ptr<JointTrajectory<float> > trajectory;

    double start_time, current_time;

    RTT::OutputPort<rstrt::kinematics::JointAngles> out_position_port;
    rstrt::kinematics::JointAngles out_position_var;

    RTT::OutputPort<rstrt::kinematics::JointVelocities> out_velocity_port;
    rstrt::kinematics::JointVelocities out_velocity_var;

    RTT::OutputPort<rstrt::kinematics::JointAccelerations> out_acceleration_port;
    rstrt::kinematics::JointAccelerations out_acceleration_var;
public:
    JointTrajectoryOrocos(RTT::TaskContext *task, std::shared_ptr<JointTrajectory<float> > trajectory) {
        this->task = task;
        this->trajectory = trajectory;
        task->addOperation("loadFromJSON", &JointTrajectoryOrocos::loadFromJSON, this);
    }

    void loadFromJSON(std::string filename) {
        std::cout << "LOAD FROM JSON!" << std::endl;
        if (!trajectory) {
            std::cout << "NO TRAJECTORY!" << std::endl;
        } else {
            std::cout << "TRAJECTORY!!" << std::endl;
        }
        trajectory->loadFromJSON(filename);
        std::cout << "LOADED!" << std::endl;
        this->dof = trajectory->getDof();
    }

    bool preparePorts() {
        task->ports()->clear();
        out_position_var = rstrt::kinematics::JointAngles(dof);
        out_velocity_var = rstrt::kinematics::JointVelocities(dof);
        out_acceleration_var = rstrt::kinematics::JointAccelerations(dof);

        out_position_port.setName("out_position_port");
        out_position_port.doc("Output port for sending position values");
        out_position_port.setDataSample(out_position_var);
        task->ports()->addPort(out_position_port);

        out_velocity_port.setName("out_velocity_port");
        out_velocity_port.doc("Output port for sending velocity values");
        out_velocity_port.setDataSample(out_velocity_var);
        task->ports()->addPort(out_velocity_port);

        out_acceleration_port.setName("out_acceleration_port");
        out_acceleration_port.doc("Output port for sending acceleration values");
        out_acceleration_port.setDataSample(out_acceleration_var);
        task->ports()->addPort(out_acceleration_port);
        return true;
    }

    bool startHook() {
        start_time = CosimaUtilities::getCurrentTime();
        return trajectory->getRunnable();
    }

    bool configureHook() {
        return trajectory->getRunnable() && preparePorts();
    }

    void stopHook() {}

    void cleanupHook() {}

    void updateHook() {
        current_time = CosimaUtilities::getCurrentTime() - start_time;
        trajectory->update(current_time);
        out_position_var.angles = trajectory->getPosition();
        out_velocity_var.velocities = trajectory->getVelocity();
        out_acceleration_var.accelerations = trajectory->getAcceleration();
        out_position_port.write(out_position_var);
        out_velocity_port.write(out_velocity_var);
        out_acceleration_port.write(out_acceleration_var);
    }

    std::shared_ptr<JointTrajectory<float> > getTrajectory() {
        return trajectory;
    }
};
