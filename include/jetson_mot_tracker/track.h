#pragma once

#include <opencv2/core.hpp>
#include "kalman_filter.h"
#include <geometry_msgs/PointStamped.h>
class Track {
public:
    // Constructor
    Track();

    // Destructor
    ~Track() = default;

    void Init(const geometry_msgs::Point& point);
    void Predict();
    void Update(const geometry_msgs::Point& point);
    geometry_msgs::Point GetStateAsPoint() const;
    float GetNIS() const;
    
    int coast_cycles_ = 0, hit_streak_ = 0;

private:
    Eigen::VectorXd ConvertPointToObservation(const geometry_msgs::Point& point) const;
    geometry_msgs::Point ConvertStateToPoint(const Eigen::VectorXd &state) const;

    KalmanFilter kf_;
};