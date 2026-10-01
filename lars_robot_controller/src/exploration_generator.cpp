#pragma once
#include <Eigen/Dense>
#include "exploration_generator.hpp"

namespace neutral_axis_controllers{
    class ExplorationGenerator{
        public:
            Eigen::Vector6d excite(const rclcpp::Time& time){

                Eigen::Vector6d F_explore = A*sin(omega*time);
                return F_explore;

            };

        private:
            Eigen::Vector6d A = ;
            Eigen::Vector6d omega = ;


    }
}