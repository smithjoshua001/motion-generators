#include <motion-generators/SineWaveJoint.hpp>
#include <motion-generators/JointTrajectoryOrocos.hpp>

class SineWaveJointOrocos : public RTT::TaskContext, public JointTrajectoryOrocos {
private:
    // std::shared_ptr<SineWaveJoint<float> > trajectory_sine;
    double start_time;
    double eta;
    bool finish;
public:
    SineWaveJointOrocos(std::string name) : RTT::TaskContext(name),
        //trajectory_sine(std::make_shared < SineWaveJoint<float> >()),
        JointTrajectoryOrocos(this, std::make_shared < SineWaveJoint<float> >()) {
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
ORO_LIST_COMPONENT_TYPE(SineWaveJointOrocos);
