#include <Eigen/Dense>

namespace neutral_axis_controllers{

    class ImpedanceController{
        public:
            void configure(Eigen::Matrix<double,6,6> K, Eigen::Matrix<double,6,6> D);
            Eigen::Vector3d Wrench compute_wrench(  Eigen::Vector6d x_d, Eigen::Vector6d dx_d, 
                                                    Eigen::Vector6d x, Eigen::Vector6d dx);

        private: 
            Eigen::Matrix<double,6,6> K_;
            Eigen::Matrix<double,6,6> D_;
    }
}