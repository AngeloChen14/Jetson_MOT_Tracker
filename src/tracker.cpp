#include "jetson_mot_tracker/tracker.h"


Tracker::Tracker() {
    id_ = 0;
}

float Tracker::CalculateDistance(const geometry_msgs::Point& det, const Track& track) {
    auto trk = track.GetStateAsPoint();
    auto det_x = det.x;
    auto det_y = det.y;
    auto trk_x = trk.x;
    auto trk_y = trk.y;

    auto dis = sqrt(pow((det_x-trk_x),2) + pow((det_y-trk_y),2));

    return dis;
}


void Tracker::HungarianMatching(const std::vector<std::vector<float>>& dis_matrix,
                                size_t nrows, size_t ncols,
                                std::vector<std::vector<float>>& association) {
    Matrix<float> matrix(nrows, ncols);
    // Initialize matrix with IOU values
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            // Multiply by -1 to find max cost
            if (dis_matrix[i][j] != 0) {
                matrix(i, j) = dis_matrix[i][j]; 
            }
            else {
                // TODO: figure out why we have to assign value to get correct result
                matrix(i, j) = 10.0f;
            }
        }
    }

   // Display begin matrix state.
   for (size_t row = 0 ; row < nrows ; row++) {
       for (size_t col = 0 ; col < ncols ; col++) {
           std::cout.width(10);
           std::cout << matrix(row,col) << ",";
       }
       std::cout << std::endl;
   }
   std::cout << std::endl;


    // Apply Kuhn-Munkres algorithm to matrix.
    Munkres<float> m;
    m.solve(matrix);

   // Display solved matrix.
//    for (size_t row = 0 ; row < nrows ; row++) {
//        for (size_t col = 0 ; col < ncols ; col++) {
//            std::cout.width(2);
//            std::cout << matrix(row,col) << ",";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << std::endl;

    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);
        }
    }
}


void Tracker::AssociateDetectionsToTrackers(const std::vector<geometry_msgs::Point>& detections,
                                            std::map<int, Track>& tracks,
                                            std::map<int, geometry_msgs::Point>& matched,
                                            std::vector<geometry_msgs::Point>& unmatched_det,
                                            float distance_threshold) {

    // Set all detection as unmatched if no tracks existing
    if (tracks.empty()) {
        for (const auto& det : detections) {
            unmatched_det.push_back(det);
        }
        return;
    }

    std::vector<std::vector<float>> dis_matrix;
    // resize IOU matrix based on number of detection and tracks
    dis_matrix.resize(detections.size(), std::vector<float>(tracks.size()));

    std::vector<std::vector<float>> association;
    // resize association matrix based on number of detection and tracks
    association.resize(detections.size(), std::vector<float>(tracks.size()));


    // row - detection, column - tracks
    for (size_t i = 0; i < detections.size(); i++) {
        size_t j = 0;
        for (const auto& trk : tracks) {
            dis_matrix[i][j] = CalculateDistance(detections[i], trk.second);
            j++;
        }
    }

    // Find association
    HungarianMatching(dis_matrix, detections.size(), tracks.size(), association);

    for (size_t i = 0; i < detections.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto& trk : tracks) {
            if (0 == association[i][j]) {
                // Filter out matched with low IOU
                if (dis_matrix[i][j] <= distance_threshold) {
                    matched[trk.first] = detections[i];
                    matched_flag = true;
                }
                // It builds 1 to 1 association, so we can break from here
                break;
            }
            j++;
        }
        // if detection cannot match with any tracks
        if (!matched_flag) {
            unmatched_det.push_back(detections[i]);
        }
    }
}


void Tracker::Run(const std::vector<geometry_msgs::Point>& detections, double duration) {

    /*** Predict internal tracks from previous frame ***/
    for (auto &track : tracks_) {
        track.second.Predict(duration);
    }

    // Hash-map between track ID and associated detection bounding box
    std::map<int, geometry_msgs::Point> matched;
    // vector of unassociated detections
    std::vector<geometry_msgs::Point> unmatched_det;

    // return values - matched, unmatched_det
    if (!detections.empty()) {
        AssociateDetectionsToTrackers(detections, tracks_, matched, unmatched_det);
    }
    // std::cout<<"Num of matches is: "<<matched.size();
    /*** Update tracks with associated bbox ***/
    for (const auto &match : matched) {
        const auto &ID = match.first;
        tracks_[ID].Update(match.second);
    }

    /*** Create new tracks for unmatched detections ***/
    for (const auto &det : unmatched_det) {
        Track tracker;
        tracker.Init(det);
        // Create new track and generate new ID
        tracks_[id_++] = tracker;
    }

    /*** Delete lose tracked tracks ***/
    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (it->second.coast_cycles_ > kMaxCoastCycles) {
            it = tracks_.erase(it);
        } else {
            it++;
        }
    }   
}


std::map<int, Track> Tracker::GetTracks() {
    return tracks_;
}