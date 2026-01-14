#pragma once

#include <string>
#include <vector>
#include <random>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "Robot.hpp"
#include "typekit.hpp"
#include "World.hpp"

struct StepResult {
    std::vector<double> obs;
    double reward;
    bool terminated;
    bool truncated;
};

class RobotEnv {
public:
    RobotEnv(const std::string& map_path, int robot_radius, bool add_noise, int max_steps);

    std::vector<double> reset(unsigned int seed);
    StepResult step(int action);

    void setGoal(double gx, double gy);

    RobotState getRobotState() const;
    Eigen::Vector2d getGoal() const;
    double getDistToGoal() const;

    void setGoalRadius(double r);
    void setRewardParams(double r_goal, double r_collision, double step_penalty);

    void setRender(bool enabled);
    void setRenderDelayMs(int ms);
    int getLastKey() const;

private:
    std::vector<double> buildObs(const Eigen::VectorXd& ranges) const;
    static double wrapToPi(double a);
    bool isCollisionHeuristic(const RobotState& before, const RobotState& after, int action) const;
    bool isReachedGoal() const;

    Eigen::Vector2d sampleFreePoint(std::mt19937& rng, int max_tries) const;

private:
    std::string map_path_;
    cv::Mat map_base_;

    int robot_radius_;
    bool add_noise_;

    Robot robot_;
    Eigen::Vector2d goal_;

    int max_steps_;
    int step_count_;

    double goal_radius_;

    double reward_goal_;
    double reward_collision_;
    double reward_step_penalty_;

    std::mt19937 rng_;

    World world_;
    bool render_enabled_;
    int render_delay_ms_;
    int last_key_;
};
