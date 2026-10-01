#include <Eigen/Dense>

namespace neutral_axis_controllers{
    class ExplorationGenerator{
        public:
            Eigen::Vector6d excite(const rclcpp::Time& time);

        private:
            Eigen::Vector6d A;
            Eigen::Vector6d omega;

    }
}
