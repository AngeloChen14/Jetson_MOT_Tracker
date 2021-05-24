#pragma once

#include "kalman_filter.h"
#include <geometry_msgs/PoseStamped.h>

class Track {
public:
    // Constructor
    Track();

    // Destructor
    ~Track() = default;

    void Init(const geometry_msgs::Point& point);
    void Predict(double duration);
    void Update(const geometry_msgs::Point& point);
    geometry_msgs::Point GetStateAsPoint() const;
    geometry_msgs::Point GetVelAsPoint() const;
    float GetNIS() const;
    
    int coast_cycles_ = 0, hit_streak_ = 0;
    int state = 0; // 0 for tentative, 1 for confirmed

private:
    Eigen::VectorXd ConvertPointToObservation(const geometry_msgs::Point& point) const;
    geometry_msgs::Point ConvertStateToPoint(const Eigen::VectorXd &state) const;
    geometry_msgs::Point ConvertVelStateToPoint(const Eigen::VectorXd &state) const;

    KalmanFilter kf_;
};
