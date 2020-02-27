#pragma once

#include <motion-generators/ModifiedFourierTrajectory.hpp>
#include <motion-generators/JointTrajectoryOrocos.hpp>

class ModifiedFourierTrajectoryOrocos : public JointTrajectoryOrocos {
public:
    ModifiedFourierTrajectoryOrocos(std::string name) : JointTrajectoryOrocos(name, std::shared_ptr< ModifiedFourierTrajectory<float> >(new ModifiedFourierTrajectory<float> ())) {}
};
