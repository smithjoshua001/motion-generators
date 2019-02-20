#include <motion-generators/ModifiedFourierTrajectory.hpp>
#include <motion-generators/JointDynamicAttractor.hpp>
#include <motion-generators/JointTrajectoryOrocos.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <CosimaUtilities/Timing.hpp>

class ModifiedFourierTrajectoryOrocos : public RTT::TaskContext, public JointTrajectoryOrocos {
private:
    // std::shared_ptr<ModifiedFourierTrajectory<float> > trajectory_mft;
    double start_time;
    double eta;
    bool finish;
public:
    ModifiedFourierTrajectoryOrocos(std::string name) : RTT::TaskContext(name),
        JointTrajectoryOrocos(this, std::make_shared < ModifiedFourierTrajectory<float> >()) {
        // trajectory_mft=this->trajectory
        finish = false;
        eta = 0.001;
    }
    bool startHook() {
        start_time = CosimaUtilities::getCurrentTime();
        return JointTrajectoryOrocos::startHook();
    }
    bool configureHook() {
        return JointTrajectoryOrocos::configureHook();
    }
    void updateHook() {
        if (!finish) {
            JointTrajectoryOrocos::updateHook();
        } else if (std::fmod(CosimaUtilities::getCurrentTime() - start_time, this->getTrajectory()->getPeriodLength()) < eta) {
            stopHook();
        }
    }
    void stopHook() {
        JointTrajectoryOrocos::stopHook();
    }
    void cleanupHook() {
        JointTrajectoryOrocos::cleanupHook();
    }
};

class JointDynamicAttractorOrocos : public RTT::TaskContext, public JointTrajectoryOrocos {
private:
    std::shared_ptr<JointDynamicAttractor<float> > trajectory_jda;
    RTT::InputPort <rstrt::robot::JointState> in_robot_port;
    RTT::FlowStatus in_robot_flow;
    rstrt::robot::JointState in_robot_var;
    bool first_config;

public:
    JointDynamicAttractorOrocos(std::string name) : RTT::TaskContext(name),
        trajectory_jda(std::make_shared < JointDynamicAttractor<float> >()),
        JointTrajectoryOrocos(this, trajectory_jda) {
        this->addProperty("finished", trajectory_jda->finished);
    }
    bool preparePorts() {
        bool out = JointTrajectoryOrocos::preparePorts();
        in_robot_var = rstrt::robot::JointState(trajectory_jda->getDof());
        in_robot_var.angles.setZero();
        in_robot_var.velocities.setZero();
        in_robot_var.torques.setZero();

        in_robot_port.setName("in_robot_port");
        in_robot_port.doc("input port for robot state values");
        this->ports()->addPort(in_robot_port);
        in_robot_var = RTT::NoData;
        return out;
    }
    void loadFromJSON(std::string filename) {
        JointTrajectoryOrocos::loadFromJSON(filename);
        in_robot_var = rstrt::robot::JointState(trajectory_jda->getDof());
    }
    void updateHook() {
        if (!first_config) {
            in_robot_flow = in_robot_port.readNewest(in_robot_var);
            if (in_robot_flow == RTT::NewData) {
                trajectory_jda->getInitialState().head(trajectory_jda->getDof()) = in_robot_var.angles;
                trajectory_jda->getInitialState().tail(trajectory_jda->getDof()) = in_robot_var.velocities;
                first_config = true;
            }
        } else {
            if (!trajectory_jda->finished) {
                JointTrajectoryOrocos::updateHook();
            } else {
                stopHook();
            }
        }
    }
    bool startHook() {
        bool out = JointTrajectoryOrocos::startHook();
        trajectory_jda->finished = false;
        first_config = false;
        return out;
    }
    bool configureHook() {
        return JointTrajectoryOrocos::configureHook();
    }
    void stopHook() {
        JointTrajectoryOrocos::stopHook();
    }
    void cleanupHook() {
        JointTrajectoryOrocos::cleanupHook();
    }
};

ORO_LIST_COMPONENT_TYPE(ModifiedFourierTrajectoryOrocos)
ORO_LIST_COMPONENT_TYPE(JointDynamicAttractorOrocos)
