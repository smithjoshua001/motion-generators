%module(directors="1") RegressorCostFunction
%{
    #define SWIG_FILE_WITH_INIT
    #include <motion-generators/RegressorCostFunction.hpp>
%}

%include <typemaps.i>
%include <std_vector.i>
%include <eigen.i>
%include <std_string.i>
%include <std_shared_ptr.i>
%include <numpy.i>
%include <pointer.i>

%apply bool & INOUT { bool &collision };

%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>)
%eigen_typemaps(Eigen::Matrix<double,Eigen::Dynamic,1>)
%eigen_typemaps(Eigen::Matrix<double,6,6>)
%eigen_typemaps(Eigen::Matrix<double,3,3>)
%eigen_typemaps(Eigen::Matrix<double,3,1>)
%eigen_typemaps(Eigen::Matrix<double,6,1>)
%eigen_typemaps(Eigen::Matrix<double,6,Eigen::Dynamic>)


%include <motion-generators/RegressorCostFunction.hpp>


%template(RegressorCostFunction_d) RegressorCostFunction<double>;
