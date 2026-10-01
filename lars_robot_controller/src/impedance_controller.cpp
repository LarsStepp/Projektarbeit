#include "impedance_controller.hpp"

class CartesianImpedanceController{
        public:
            void configure(Eigen::Matrix<double,6,6> K, Eigen::Matrix<double,6,6> D){
                K_ = K;
                D_ = D;
            }
            Eigen::Vector3d compute_wrench(  Eigen::Vector6d x_d, Eigen::Vector6d dx_d, 
                                                    Eigen::Vector6d x, Eigen::Vector6d dx){

                                        Eigen::Vector6d wrench;
                                        wrench = K_*(x_d - x)+D_*(dx_d - dx);

                                        return wrench
            }

        private: 
            Eigen::Matrix<double,6,6> K_;
            Eigen::Matrix<double,6,6> D_;
    }