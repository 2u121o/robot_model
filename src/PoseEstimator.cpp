
#include "PoseEstimator.hpp"

PoseEstimator::PoseEstimator(const RobotState &initial_state)
{
    current_state_robot_state_ = initial_state;
    current_state_.resize(3);
    current_state_ << initial_state.x, initial_state.y, initial_state.theta;
    std::string file_config_path = "/home/dario/Workspace/robot_model/config/ukf_params.json"; 
    ukf_.initialize(current_state_, file_config_path);
    ukf_.setStateTransitionFunction([this](Eigen::VectorXd& prediction, const Eigen::VectorXd& state, const Eigen::VectorXd& input_k) -> void
                                    {
                                        std::cout << "stateTransition LAMBDA" << std::endl;
                                        this->stateTransition(prediction, state, input_k);
                                    });
    ukf_.setOutputTransitionFunction([this](Eigen::VectorXd& output, const Eigen::VectorXd& state, const Eigen::VectorXd& input_k) -> void
                                    {
                                        this->outputTransition(output, state, input_k);
                                    });
}

void PoseEstimator::estimatePose(const int input_k, const Eigen::VectorXd& ranges)
{
    std::cout << "estimatePose" << std::endl;
    ranges_ = ranges;
    Eigen::VectorXd output;
    // stateTransition(current_state_, input_k);
    // outputTransition(output, current_state_, input_k);
    Eigen::VectorXd input_k_vec(1);
    input_k_vec << input_k;
    std::cout << "estimatePose1" << std::endl;
    ukf_.computePrediction(input_k_vec);
    std::cout << "estimatePose2" << std::endl;
    ukf_.computeCorrection(ranges_);
    std::cout << "estimatePose3" << std::endl;
    ukf_.getState(current_state_);
}

void PoseEstimator::stateTransition(Eigen::VectorXd& prediction, const Eigen::VectorXd& state, const Eigen::VectorXd& input_k)
{
    std::cout << "stateTransition" << std::endl;
    double u_v = 0;
    double u_t = 0;

    double k = input_k(0);
    prediction.resize(state.size());

    if(k == 81.0)
    {
        u_t = 0.054532925;
    }
    else if(k == 82.0)
    {
        u_v = 1.5;
    }
    else if(k == 83.0)
    {
        u_t = -0.054532925;
    }
    else if(k == 84.0)
    {
        u_v = -1.5;   
    }

    double x = state(0);
    double y = state(1);
    double theta = state(2);

    x +=  STEP_SIZE*5*std::cos(theta)*u_v;
    y += - STEP_SIZE*5*std::sin(theta)*u_v;
    theta += STEP_SIZE*2*u_t;

    prediction << x, y, theta;

}


void PoseEstimator::outputTransition(Eigen::VectorXd& output, const Eigen::VectorXd& state, const Eigen::VectorXd& input_k)
{
    std::cout << "outputTransition" << std::endl;
    output.resize(ranges_.size());
    for(int i=0; i<ranges_.size(); i++)
    {
        double angle = sensor_settings_.angle_min+(i*sensor_settings_.angle_increment);
        double d = ranges_(i);
        double x_d = (d*std::cos(angle));
        double y_d = (d*std::sin(angle));

        Eigen::Vector2d pose_d(x_d, y_d);
        Eigen::Vector2d pose_d_robot_frame;
        Eigen::Matrix2d R;
        R << std::cos(state(2)), std::sin(state(2)),
             -std::sin(state(2)), std::cos(state(2));
        pose_d_robot_frame = R*pose_d + Eigen::Vector2d(state(0), state(1));

        output(i) = std::sqrt(std::pow(state(0)-pose_d_robot_frame(0),2)+std::pow(state(1)-pose_d_robot_frame(1),2));
    }
}

const RobotState& PoseEstimator::getStates() 
{
    current_state_robot_state_.x = current_state_(0);
    current_state_robot_state_.y = current_state_(1);
    current_state_robot_state_.theta = current_state_(2);
    return current_state_robot_state_;
}

void PoseEstimator::setSensorSettings(const SensorSettings &sensor_settings)
{
    sensor_settings_ = sensor_settings;
}