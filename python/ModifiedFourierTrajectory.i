%module ModifiedFourierTrajectory

%{
    #define SWIG_FILE_WITH_INIT
    #include <motion-generators/ModifiedFourierTrajectory.hpp>
%}

%include <typemaps.i>
%include <std_vector.i>
%include <eigen.i>
%include <std_string.i>

%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>)
%eigen_typemaps(Eigen::Matrix<double,Eigen::Dynamic,1>)

%include <motion-generators/JointTrajectory.hpp>
%include <motion-generators/ModifiedFourierTrajectory.hpp>

%template(SizeVector) std::vector<size_t>;
%template(CoeffecientVector) std::vector<Eigen::Matrix<double,Eigen::Dynamic,1>,Eigen::aligned_allocator<Eigen::Matrix<double,Eigen::Dynamic,1>>>;

%template(Trajectory_d) Trajectory<double>;
%template(JointTrajectory_d) JointTrajectory<double>;
%template(ModifiedFourierTrajectory_d) ModifiedFourierTrajectory<double>;