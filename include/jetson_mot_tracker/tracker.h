#pragma once

#include <map>
#include <geometry_msgs/PointStamped.h>
// #include <opencv2/core.hpp>
#include "track.h"
#include "munkres.h"
// #include "utils.h"
constexpr int kMaxCoastCycles = 20;
constexpr int kMinHits = 3;
constexpr float kMinConfidence = 0.4;
constexpr float kDistanThreshold = 1.0;

class Tracker {
public:
    Tracker();
    ~Tracker() = default;

    static float CalculateDistance(const geometry_msgs::Point& point, const Track& track);

    static void HungarianMatching(const std::vector<std::vector<float>>& dis_matrix,
                           size_t nrows, size_t ncols,
                           std::vector<std::vector<float>>& association);

/**
 * Assigns detections to tracked object (both represented as bounding boxes)
 * Returns 2 lists of matches, unmatched_detections
 * @param detection
 * @param tracks
 * @param matched
 * @param unmatched_det
 * @param distance_threshold
 */
    static void AssociateDetectionsToTrackers(const std::vector<geometry_msgs::Point>& detections,
                                       std::map<int, Track>& tracks,
                                       std::map<int, geometry_msgs::Point>& matched,
                                       std::vector<geometry_msgs::Point>& unmatched_det);

    void Update(const std::vector<geometry_msgs::Point>& detections);
    void Predict(double duration);

    std::map<int, Track> GetTracks();


private:
    // Hash-map between ID and corresponding tracker
    std::map<int, Track> tracks_;

    // Assigned ID for each bounding box
    int id_;
};