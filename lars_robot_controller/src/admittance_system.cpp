#include "admittance_system.hpp"



class AdmittanceSystem
{
public:

    void initialize(
        const Eigen::Vector6d& x0)
    {
        x_d_ = x0;
        dx_d_.setZero();
        Eigen::Matrix<double,6,6> M_;
        Eigen::Matrix<double,6,6> D_;
        Eigen::Matrix<double,6,1> x_d_;
        Eigen::Matrix<double,6,1> dx_d_;
        Eigen::Matrix<double,6,1> ddx_d_;
    }

    void update(
        const Wrench& F_explore,
        const Wrench& F_interaction,
        double dt)
    {
        Eigen::Vector6d F =
            F_explore - F_interaction;

        Eigen::Vector6d ddx =
            M_.inverse() *
            (F - D_ * dx_d_);

        dx_d_ += ddx * dt;
        x_d_  += dx_d_ * dt;
    }

    const Eigen::VectorXd& position() const
    {
        return x_d_;
    }

    const Eigen::VectorXd& velocity() const
    {
        return dx_d_;
    }

private:

    Eigen::Matrix<double,6,6> M_;
    Eigen::Matrix<double,6,6> D_;

    Eigen::Matrix<double,6,1> x_d_;
    Eigen::Matrix<double,6,1> dx_d_;
    Eigen::Matrix<double,6,1> ddx_d_;
};