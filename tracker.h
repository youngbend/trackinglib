// Author: Benjamin Young (youngben@umich.edu)
// Made for EECS 467

#ifndef TRACKER_H
#define TRACKER_H

#include <vector>
#include <list>
#include <mutex>

class Tracker;

struct point {
    int row;
    int col;
};

class Target {
public:
    Target(point center_in, int radius_in) : 
        center(center_in), prev_center({-1,-1}), radius(radius_in), prev_radius(-1), loss_count(0), dead(false) {}
    
    void update(point center_in, int radius_in) {
        prev_center = center;
        prev_radius = radius;
        loss_count = 0;
        dead = false;
        center = center_in;
        radius = radius_in;
    }

    void lost() { loss_count += 1; }

    void kill() { dead = true; }

    point get_center() { return center; }

    friend class Tracker;

protected:
    point center;
    point prev_center;
    int radius;
    int prev_radius;
    int loss_count;
    bool dead;
};

class Tracker {
public:
    Tracker(int scan_offset_row, int scan_offset_col, int target_offset_in, int threshold_in, int tracking_offset_in, int tracking_timeout_in);

    std::vector<point> get_target_centers();

    void scan(unsigned char* image, int rows, int cols, int n_threads);

    void update_targets(unsigned char* image, int rows, int cols);

private:
    int row_scan_offset;
    int col_scan_offset;
    int target_offset;
    int threshold;
    int tracking_offset;
    int tracking_timeout;

    std::mutex target_lock;

    std::list<Target*> targets;

    void scan_thread(unsigned char* image, int rows, int cols, int lbound, int rbound);
    void update_targets_thread(unsigned char* image, int rows, int cols, Target *target);
    bool pinpoint_target(unsigned char* image, int rows, int cols, point start_loc, point &center, int &radius, Target *curr_target);
};

#endif
