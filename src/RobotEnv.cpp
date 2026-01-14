#include "RobotEnv.hpp"
#include <cmath>

static bool isFreePixel(const cv::Mat& map, int x, int y) {
    if (x < 0 || y < 0 || x >= map.cols || y >= map.rows) return false;
    const cv::Vec3b p = map.at<cv::Vec3b>(y, x);
    return !(p[0] == 0 && p[1] == 0 && p[2] == 0);
}

RobotEnv::RobotEnv(const std::string& map_path, int robot_radius, bool add_noise, int max_steps)
: map_path_(map_path)
, map_base_(cv::imread(map_path_))
, robot_radius_(robot_radius)
, add_noise_(add_noise)
, robot_()
, goal_(Eigen::Vector2d(0.0, 0.0))
, max_steps_(max_steps)
, step_count_(0)
, goal_radius_(15.0)
, reward_goal_(10.0)
, reward_collision_(-10.0)
, reward_step_penalty_(-0.01)
, rng_(0)
, world_()
, render_enabled_(false)
, render_delay_ms_(1)
, last_key_(-1)
{
    RobotState s;
    s.x = 100.0;
    s.y = 100.0;
    s.theta = 0.0;

    robot_ = Robot(map_base_, s, robot_radius_, add_noise_);

    world_.setMap(map_base_);
    world_.setRobot(robot_);
}

void RobotEnv::setGoal(double gx, double gy) {
    goal_[0] = gx;
    goal_[1] = gy;
}

RobotState RobotEnv::getRobotState() const {
    return robot_.getStates();
}

Eigen::Vector2d RobotEnv::getGoal() const {
    return goal_;
}

double RobotEnv::getDistToGoal() const {
    RobotState s = robot_.getStates();
    Eigen::Vector2d p(s.x, s.y);
    return (goal_ - p).norm();
}

void RobotEnv::setGoalRadius(double r) {
    goal_radius_ = r;
}

void RobotEnv::setRewardParams(double r_goal, double r_collision, double step_penalty) {
    reward_goal_ = r_goal;
    reward_collision_ = r_collision;
    reward_step_penalty_ = step_penalty;
}

double RobotEnv::wrapToPi(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

bool RobotEnv::isReachedGoal() const {
    return getDistToGoal() <= goal_radius_;
}

bool RobotEnv::isCollisionHeuristic(const RobotState& before, const RobotState& after, int action) const {
    const bool is_translation = (action == 0 || action == 1);
    if (!is_translation) return false;

    const double dx = after.x - before.x;
    const double dy = after.y - before.y;
    const double d = std::sqrt(dx * dx + dy * dy);

    return d < 1e-6;
}

Eigen::Vector2d RobotEnv::sampleFreePoint(std::mt19937& rng, int max_tries) const {
    std::uniform_int_distribution<int> ux(1, map_base_.cols - 2);
    std::uniform_int_distribution<int> uy(1, map_base_.rows - 2);

    for (int i = 0; i < max_tries; ++i) {
        int x = ux(rng);
        int y = uy(rng);
        if (isFreePixel(map_base_, x, y)) {
            return Eigen::Vector2d(static_cast<double>(x), static_cast<double>(y));
        }
    }
    return Eigen::Vector2d(50.0, 50.0);
}

std::vector<double> RobotEnv::buildObs(const Eigen::VectorXd& ranges) const {
    SensorSettings ss;
    robot_.getSensorSettings(ss);

    RobotState s = robot_.getStates();
    Eigen::Vector2d p(s.x, s.y);

    Eigen::Vector2d rel = goal_ - p;
    const double d = rel.norm();
    const double ang = std::atan2(rel.y(), rel.x());
    const double theta_goal = wrapToPi(ang - s.theta);

    const int n = static_cast<int>(ranges.size());
    std::vector<double> obs;
    obs.resize(static_cast<size_t>(n + 2));

    const double denom = (ss.range_max > 1e-9) ? ss.range_max : 1.0;
    for (int i = 0; i < n; ++i) {
        double v = ranges[i] / denom;
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
        obs[static_cast<size_t>(i)] = v;
    }

    obs[static_cast<size_t>(n)] = d;
    obs[static_cast<size_t>(n + 1)] = theta_goal;

    return obs;
}

std::vector<double> RobotEnv::reset(unsigned int seed) {
    rng_.seed(seed);
    step_count_ = 0;

    if (map_base_.empty()) {
        map_base_ = cv::imread(map_path_);
    }

    RobotState init;
    Eigen::Vector2d p = sampleFreePoint(rng_, 5000);
    init.x = p.x();
    init.y = p.y();
    std::uniform_real_distribution<double> uth(-M_PI, M_PI);
    init.theta = uth(rng_);

    robot_ = Robot(map_base_, init, robot_radius_, add_noise_);

    Eigen::Vector2d g = sampleFreePoint(rng_, 5000);
    setGoal(g.x(), g.y());

    Eigen::VectorXd ranges;
    cv::Mat map_work = map_base_.clone();
    robot_.takeMeasurementsRange(map_work, ranges);

    return buildObs(ranges);
}

StepResult RobotEnv::step(int action) {
    cv::Mat map_work = map_base_.clone();

    int key = -1;
    if (action == 0) key = 82;
    if (action == 1) key = 84;
    if (action == 2) key = 81;
    if (action == 3) key = 83;

    const RobotState before = robot_.getStates();
    const double prev_d = getDistToGoal();

    robot_.moveRobot(map_work, key);

    if (render_enabled_) {
        cv::circle(
            map_work,
            cv::Point(static_cast<int>(goal_.x()), static_cast<int>(goal_.y())),
            6,
            cv::Scalar(0, 0, 255),
            -1
        );

        world_.setMap(map_work);
        world_.setRobot(robot_);
        world_.drawWorld(render_delay_ms_);
        last_key_ = world_.getK();
    } else {
        last_key_ = -1;
    }

    Eigen::VectorXd ranges;
    robot_.takeMeasurementsRange(map_work, ranges);

    const RobotState after = robot_.getStates();
    const bool collision = isCollisionHeuristic(before, after, action);
    const bool reached = isReachedGoal();

    const double new_d = getDistToGoal();
    const double progress = (prev_d - new_d);

    double reward = 0.0;
    reward += progress;
    reward += reward_step_penalty_;
    if (reached) reward += reward_goal_;
    if (collision) reward += reward_collision_;

    step_count_++;
    const bool truncated = (step_count_ >= max_steps_);
    const bool terminated = (reached || collision);

    StepResult out;
    out.obs = buildObs(ranges);
    out.reward = reward;
    out.terminated = terminated;
    out.truncated = truncated;
    return out;
}

void RobotEnv::setRender(bool enabled) {
    render_enabled_ = enabled;
}

void RobotEnv::setRenderDelayMs(int ms) {
    render_delay_ms_ = ms;
}

int RobotEnv::getLastKey() const {
    return last_key_;
}
