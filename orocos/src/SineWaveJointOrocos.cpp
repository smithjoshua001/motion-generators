#include <motion-generators/SineWaveJoint.hpp>
#include <motion-generators/JointTrajectoryOrocos.hpp>

class SineWaveJointOrocos : public JointTrajectoryOrocos {
private:
    // std::shared_ptr<SineWaveJoint<float> > trajectory_sine;
    double start_time;
    double eta;
    bool finish;
public:
    SineWaveJointOrocos(std::string name) :
        JointTrajectoryOrocos(name, std::shared_ptr<SineWaveJoint<float> > (new SineWaveJoint<float>())) {
        finish = false;
        this->addProperty("finish", finish);
        eta = 0.001;
    }
    virtual ~SineWaveJointOrocos() {
        std::cout << "DESTROY SWJO" << std::endl;
    }
    bool startHook() {
        start_time = CosimaUtilities::getCurrentTime();
        return JointTrajectoryOrocos::startHook();
    }
    void updateHook() override {
        JointTrajectoryOrocos::updateHook();
        if (finish && std::fmod(CosimaUtilities::getCurrentTime() - start_time, this->getTrajectory()->getPeriodLength()) < eta) {
            stopHook();
        }
    }
};

#include <motion-generators/SineCosineJoint.hpp>
class SineCosineJointOrocos : public JointTrajectoryOrocos {
private:
    // std::shared_ptr<SineWaveJoint<float> > trajectory_sine;
    double start_time;
    double eta;
    bool finish;
public:
    SineCosineJointOrocos(std::string name) :
        JointTrajectoryOrocos(name, std::shared_ptr<SineCosineJoint<float> > (new SineCosineJoint<float>())) {
        finish = false;
        this->addProperty("finish", finish);
        eta = 0.001;
    }
    virtual ~SineCosineJointOrocos() {
        std::cout << "DESTROY SWJO" << std::endl;
    }
    bool startHook() {
        start_time = CosimaUtilities::getCurrentTime();
        return JointTrajectoryOrocos::startHook();
    }
    void updateHook() override {
        JointTrajectoryOrocos::updateHook();
        if (finish && std::fmod(CosimaUtilities::getCurrentTime() - start_time, this->getTrajectory()->getPeriodLength()) < eta) {
            stopHook();
        }
    }
};
ORO_LIST_COMPONENT_TYPE(SineWaveJointOrocos);
ORO_LIST_COMPONENT_TYPE(SineCosineJointOrocos);
