#include <Eigen/Dense>



namespace neutral_axis_controllers{
    class AdmittanceSystem{
        public:
            void initialize(const Eigen::Vector6d& x0);
            void update(const Wrench& F_explore,
                        const Wrench& F_interaction,
                        double dt);
            
            const Eigen::Vector6d& position() const
            {
                return x_d_;
            }

            const Eigen::Vector6d& velocity() const
            {
                return dx_d_;
            }

        private:
            
            Eigen::Matrix<double,6,6> M_;
            Eigen::Matrix<double,6,6> D_;
            Eigen::Matrix<double,6,1> x_d_;
            Eigen::Matrix<double,6,1> dx_d_;
            Eigen::Matrix<double,6,1> ddx_d_;

    }
}