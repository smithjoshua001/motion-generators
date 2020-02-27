%module(directors="1") ModifiedFourierTrajectory

%{
    #define SWIG_FILE_WITH_INIT
    #include <motion-generators/Trajectory.hpp>
    #include <motion-generators/ModifiedFourierTrajectory.hpp>
%}

%include <typemaps.i>
%include <std_vector.i>
%include <eigen.i>
%include <std_string.i>
%include <std_shared_ptr.i>
%include <cpointer.i>

%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>)
%eigen_typemaps(Eigen::Matrix<double,Eigen::Dynamic,1>)

%include <motion-generators/Trajectory.hpp>
%include <motion-generators/JointTrajectory.hpp>
%include <motion-generators/ModifiedFourierTrajectory.hpp>

%template(SizeVector) std::vector<size_t>;
%template(CoeffecientVector) std::vector<Eigen::Matrix<double,Eigen::Dynamic,1>,Eigen::aligned_allocator<Eigen::Matrix<double,Eigen::Dynamic,1>>>;


%shared_ptr(TrajectoryStorage<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::Dynamic>);
%template(TrajectoryStorage_d) TrajectoryStorage<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::Dynamic>;

%shared_ptr(Trajectory<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::Dynamic>);
%feature("director") Trajectory<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::Dynamic>;
%template(Trajectory_d) Trajectory<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::Dynamic>;

%shared_ptr(JointTrajectory<double,Eigen::Dynamic>);
%feature("director") JointTrajectory<double,Eigen::Dynamic>;
%template(JointTrajectory_d) JointTrajectory<double,Eigen::Dynamic>;

%shared_ptr(ModifiedFourierTrajectory<double>);
%template(ModifiedFourierTrajectory_d) ModifiedFourierTrajectory<double>;
