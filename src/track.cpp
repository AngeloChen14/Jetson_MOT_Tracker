#include "jetson_mot_tracker/track.h" 


Track::Track() : kf_(5, 3) {

    /*** Define constant velocity model ***/
    // state - center_x, center_y, center_z, v_cx, v_cy
    kf_.F_ <<
            1, 0, 0, 1, 0,
            0, 1, 0, 0, 1,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

    // // Give high uncertainty to the unobservable initial velocities
    kf_.P_ <<
           10,  0,  0,     0, 0, 
            0, 10,  0,     0, 0, 
            0,  0, 10,     0, 0, 
            0,  0,  0,  1000, 0,
            0,  0,  0, 0,  1000;


    kf_.H_ <<
            1, 0, 0, 0, 0, 
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0;

    kf_.Q_ <<
            1.0, 0,  0, 0, 0,
            0, 1.0,  0, 0, 0,
            0,  0, 1.0, 0, 0,
            0,  0,  0, 10, 0,
            0,  0,  0, 0, 10;

    kf_.R_ <<
           0.1,  0,  0,
            0,  0.1,  0,
            0,  0,  0.1;
            
}


// Get predicted locations from existing trackers
// dt is time elapsed between the current and previous measurements
void Track::Predict(double duration) {
    kf_.Predict(duration);

    // hit streak count will be reset
    // if (coast_cycles_ > 0) {
    //     hit_streak_ = 0;
    // }
    // accumulate coast cycle count
    coast_cycles_++;
}


// Update matched trackers with assigned detections
void Track::Update(const geometry_msgs::Point& point) {

    // get measurement update, reset coast cycle count
    coast_cycles_ = 0;
    // accumulate hit streak count
    hit_streak_++;

    // observation - center_x, center_y, area, ratio
    Eigen::VectorXd observation = ConvertPointToObservation(point);
    kf_.Update(observation);


}


// Create and initialize new trackers for unmatched detections, with initial bounding box
void Track::Init(const geometry_msgs::Point& point) {
    kf_.x_.head(3) << ConvertPointToObservation(point);
    hit_streak_++;
}


/**
 * Returns the current bounding box estimate
 * @return
 */
geometry_msgs::Point Track::GetStateAsPoint() const {
    return ConvertStateToPoint(kf_.x_);
}

geometry_msgs::Point Track::GetVelAsPoint() const {
    return ConvertStateToPoint(kf_.x_);
}


float Track::GetNIS() const {
    return kf_.NIS_;
}


/**
 * Takes a bounding box in the form [x, y, width, height] and returns z in the form
 * [x, y, s, r] where x,y is the centre of the box and s is the scale/area and r is
 * the aspect ratio
 *
 * @param bbox
 * @return
 */
Eigen::VectorXd Track::ConvertPointToObservation(const geometry_msgs::Point& point) const{
    Eigen::VectorXd observation = Eigen::VectorXd::Zero(3);
    float center_x = point.x;
    float center_y = point.y;
    float center_z = point.z;
    observation << center_x, center_y,center_z;
    return observation;
}


/**
 * Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
 * [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
 *
 * @param state
 * @return
 */
geometry_msgs::Point Track::ConvertStateToPoint(const Eigen::VectorXd &state) const {
    // state - center_x, center_y, v_cx, v_cy, 
    geometry_msgs::Point point;
    point.x = static_cast<double>(state[0]);
    point.y = static_cast<double>(state[1]);
    point.z = static_cast<double>(state[2]);
    return point;
}

geometry_msgs::Point Track::ConvertVelStateToPoint(const Eigen::VectorXd &state) const {
    // state - center_x, center_y, v_cx, v_cy, 
    geometry_msgs::Point point;
    point.x = static_cast<double>(state[3]);
    point.y = static_cast<double>(state[4]);
    return point;
}