// Author: Benjamin Young (youngben@umich.edu)
// Made for EECS 467

#include "tracker.h"
#include <iostream>
#include <algorithm>
#include <thread>
#include <math.h>
#include <numeric>
#include <functional>

using namespace std;

int gradient(int left, int right) { return left - right; }

double dot(const vector<double> &first, const vector<double> &second) {
    return inner_product(first.begin(), first.end(), second.begin(), 0.0);
}

double L2Norm(const vector<double> &vec) { return sqrt(dot(vec, vec)); }

vector<double> unitVec(const vector<double> &vec) {
    double norm = L2Norm(vec);
    vector<double> ret;
    for (auto v : vec) {
        ret.push_back(v / norm);
    }
    return ret;
}

void rotation(double &x, double &y, double theta) {
    double u = cos(theta) * x - sin(theta) * y;
    double v = sin(theta) * x + cos(theta) * y;
    x = u;
    y = v;
}

bool corr_coef_check(std::vector<double>& line_x, std::vector<double>& line_y){
    int n = line_x.size();
    if(n == 0) return false;

    for (size_t i = 0; i < line_x.size(); i++) {
        // Rotate points by 45 degrees to account for vertical or horizontal lines
        rotation(line_x[i], line_y[i], 0.785398);
    }

    double sum_x = accumulate(line_x.begin(), line_x.end(), 0.0);
    double sum_y = accumulate(line_y.begin(), line_y.end(), 0.0);
    double sq_sum_x = dot(line_x, line_x);
    double sq_sum_y = dot(line_y, line_y);
    double sum_xy = dot(line_x, line_y);

    double corr_coef = (n * sum_xy - sum_x * sum_y) / sqrt((n * sq_sum_x - sum_x * sum_x) * (n * sq_sum_y - sum_y * sum_y));

    if(abs(corr_coef) < 0.9) return false;

    return true;
}

Tracker::Tracker(int scan_offset_row, int scan_offset_col, int target_offset_in, int threshold_in, int tracking_offset_in, int tracking_timeout_in) :
    row_scan_offset(scan_offset_row), col_scan_offset(scan_offset_col), target_offset(target_offset_in), threshold(threshold_in),
    tracking_offset(tracking_offset_in), tracking_timeout(tracking_timeout_in) { }

vector<point> Tracker::get_target_centers() {
    vector<point> centers;
    for (auto target:targets) {
        centers.push_back(target->get_center());
    }
    return centers;
}

void Tracker::scan(unsigned char* image, int rows, int cols, int n_threads) {
    // Update targets
    update_targets(image, rows, cols);

    for (auto iter = targets.begin(); iter != targets.end();) {
        if ((*iter)->loss_count) {
        delete (*iter);
            iter = targets.erase(iter);
        }
        else iter++;
    }

    vector<thread> runners;
    for (int i = 0; i < n_threads; ++i) {
        runners.emplace_back(bind(&Tracker::scan_thread, this, image, rows, cols, i*(cols/n_threads), (i+1)*(cols/n_threads)));
    }

    for (size_t i = 0; i < runners.size(); ++i) {
        runners[i].join();
    }

    return;
}

void Tracker::update_targets(unsigned char* image, int rows, int cols) {
    vector<thread> runners;
    for (auto t : targets) {
        runners.emplace_back(bind(&Tracker::update_targets_thread, this, image, rows, cols, t));
    }

    for (size_t i = 0; i < runners.size(); i++) {
        runners[i].join();
    }

    for (auto iter = targets.begin(); iter != targets.end();) {
        if ((*iter)->dead) {
        delete (*iter);
        iter = targets.erase(iter);
    }
        else iter++;
    }
}

void Tracker::scan_thread(unsigned char* image, int rows, int cols, int lbound, int rbound) {

    for (int r = 0; r < rows; r += row_scan_offset) {
        for (int c = lbound + col_scan_offset; c < rbound; c += col_scan_offset) {
            if (gradient(image[r * cols + c - col_scan_offset], image[r * cols + c]) > threshold) {
                // image[r*cols + c - col_scan_offset] = 255;
                bool inside_target = false;
                for (auto t : targets) {
                    if ((t->center.row - t->radius) <= r && r <= (t->center.row + t->radius) && 
                        (t->center.col - t->radius) <= c && c <= (t->center.col + t->radius)) {
                            inside_target = true;
                            break;
                        }
                }
                
                if (inside_target) {
                    c += col_scan_offset;
                    continue;
                }

                point center;
                int radius;
                
                if (pinpoint_target(image, rows, cols, {r,c}, center, radius, nullptr)) {
                    target_lock.lock();
                    targets.push_back(new Target(center, radius));
                    target_lock.unlock();
                    // DEBUG
                    // image[center.row * cols + center.col - col_scan_offset] = 255;
                }
            }
        }
    }
}

void Tracker::update_targets_thread(unsigned char* image, int rows, int cols, Target *target) {
    double row_offset = 0;
    double col_offset = 0;
    double depth_offset = 0;
    double track_offset = tracking_offset;
    if (target->prev_center.row != -1 && target->prev_center.col != -1) {
        row_offset = target->center.row - target->prev_center.row;
        col_offset = target->center.col - target->prev_center.col;
        depth_offset = target->radius - target->prev_radius;
        if (depth_offset < 0) depth_offset = 0;
        if (target->loss_count) {
            double interpolation_factor = 1.9 - exp(-target->loss_count / 8.0);
            row_offset *= interpolation_factor;
            col_offset *= interpolation_factor;
            depth_offset *= interpolation_factor;
	    track_offset *= interpolation_factor;
        }
    }

    else if (target->loss_count) {
    	double interpolation_factor = 1.9 - exp(-target->loss_count / 8.0);
	track_offset *= interpolation_factor;
    }

    int top = int(target->center.row - target->radius + row_offset - depth_offset - track_offset);
    int bottom = int(target->center.row + target->radius + row_offset + depth_offset + track_offset);
    int left = int(target->center.col - target->radius + col_offset - depth_offset - track_offset);
    int right = int(target->center.col + target->radius + col_offset + depth_offset + track_offset);

    top = (top < 0) ? 0 : top;
    bottom = (bottom >= rows) ? rows-1 : bottom;
    left = (left < 0) ? 0 : left;
    right = (right >= cols) ? cols-1 : right;

    point center;
    int radius = 0;

    for (int r = top; r < bottom; r += row_scan_offset) {
        for (int c = left; c < right; c += col_scan_offset) {
            if (gradient(image[r*cols+c-col_scan_offset], image[r*cols+c]) > threshold) {
                if (pinpoint_target(image, rows, cols, {r,c}, center, radius, target)) {
                    target->update(center, radius);
                    return;
                }
            }
        }
    }

    target->lost();
    if (target->loss_count >= tracking_timeout) {
        target->kill();
    }
}

bool Tracker::pinpoint_target(unsigned char* image, int rows, int cols, point start_loc, point &center, int &radius, Target *curr_target) {
    int new_scan_offset_row = ceil(row_scan_offset / 2.0);
    int new_scan_offset_col = ceil(col_scan_offset / 2.0);

    int initial_left = -1;
    for (int c = start_loc.col; c > 0; c -= new_scan_offset_col) {
        if (gradient(image[start_loc.row * cols + c - new_scan_offset_col], image[start_loc.row * cols + c]) > threshold) {
            initial_left = c;
            break;
        }
    }
    if (initial_left == -1) return false;

    int initial_right = -1;
    for (int c = start_loc.col; c < cols; c += new_scan_offset_col) {
        if (gradient(image[start_loc.row * cols + c - new_scan_offset_col], image[start_loc.row * cols + c]) < -threshold) {
            initial_right = c;
            break;
        }
    }
    if (initial_right == -1) return false;

    // Column number of the top left, top right, bottom left, bottom right edges of the bar, in that order
    vector<int> vbar_bounds;

    // Row number of the top and bottom
    int top = -1;
    int bottom = -1;

    int min_intensity = 255;
    vector<int> left_right = {initial_left, initial_right};
    int gradient_counter = 0;

    vector<double> vbar_x;
    vector<double> vbar_y;


    for (int r = start_loc.row; r < rows; r += new_scan_offset_row) {
        if (r < 0) return false;

        bool edge_trigger = false;
        bool cross_encounter = false;
        int min_row_intensity = 255;


        for (int c = left_right[0] - target_offset; c < left_right[1] + target_offset; c += new_scan_offset_col) {
            if (c < new_scan_offset_col || c >= cols) return false;

            /* White to black */
            if (gradient(image[r * cols + c - new_scan_offset_col], image[r * cols + c]) > threshold) {
                // image[r*cols + c - new_scan_offset_col] = 255;
                cross_encounter = true;
                if (!edge_trigger) {
                    edge_trigger = true;
                    left_right[0] = c;

                    if (top == -1) {
                        top = r;
                        vbar_bounds.push_back(c);
                    }
                }
            }

            /* Black to white */
            else if (gradient(image[r * cols + c - new_scan_offset_col], image[r * cols + c]) < -threshold) {
                // image[r*cols + c - new_scan_offset_col] = 0;
                cross_encounter = true;
                if (edge_trigger) {
                    left_right[1] = c;
                    
                    vbar_x.push_back((left_right[1] + left_right[0])/2.0);
                    vbar_y.push_back(r);

                    if (top != -1 && vbar_bounds.size() == 1) {
                        vbar_bounds.push_back(c);
                    }
                    
                    break;
                }
            }


            if (top != -1 && image[r*cols+c] < min_row_intensity) {
                min_row_intensity = image[r*cols+c];
                if (min_row_intensity < min_intensity) {
                    min_intensity = min_row_intensity;
                }
            }
        }

        if (cross_encounter) gradient_counter = 0;

        else if (top != -1 && (min_row_intensity - min_intensity) >= threshold) {
            bottom = r - new_scan_offset_row;
            vbar_bounds.push_back(left_right[0]);
            vbar_bounds.push_back(left_right[1]);
            break;
        }

        else gradient_counter++;


        if (gradient_counter > 1.5 * (initial_right - initial_left)) return false;
    }

    
    if (top == -1 || bottom == -1 || vbar_bounds.size() != 4) return false;

    if (abs((vbar_bounds[1] - vbar_bounds[0]) - (vbar_bounds[3] - vbar_bounds[2])) > 3*target_offset) return false;

    if(!corr_coef_check(vbar_x, vbar_y)) return false;

    double center_row = (top + bottom) / 2.0;
    double center_column = accumulate(vbar_bounds.begin(), vbar_bounds.end(), 0) / 4.0 + new_scan_offset_col;
    int column_radius = (vbar_bounds[1] - vbar_bounds[0]) / 2;

    if ((vbar_bounds[1] - vbar_bounds[0]) > (bottom - top) / 2) return false;
    
    int left = -1;
    int right = -1;

    vector<int> hbar_bounds;
    vector<double> hbar_x;
    vector<double> hbar_y;


    vector<int> up_down = {int(center_row) - column_radius, int(center_row) + column_radius};
    for (int c = int(center_column) - column_radius; c >= new_scan_offset_col; c -= new_scan_offset_col) {
        bool edge_trigger = false;
        bool cross_encounter = false;
        int min_col_intensity = 255;

        for (int r = up_down[0] - target_offset; r < up_down[1] + target_offset; r += new_scan_offset_row) {
            if (r < 0 || r >= rows) return false;
            
            if (gradient(image[(r-new_scan_offset_row)*cols + c], image[r*cols+c]) > threshold) {
                cross_encounter = true;

                if (!edge_trigger) {
                    edge_trigger = true;
                    up_down[0] = r;
                }
            }


            else if (gradient(image[(r-new_scan_offset_row)*cols + c], image[r*cols+c]) < -threshold) {
                cross_encounter = true;
                if (edge_trigger) {
                   
                    up_down[1] = r;

                    hbar_x.push_back(c);
                    hbar_y.push_back((up_down[1] + up_down[0])/2.0);

                    break;
                }
            }

            if (image[r*cols+c] < min_col_intensity) {
                min_col_intensity = image[r*cols+c];
            }
        }
        if (!cross_encounter && abs(min_col_intensity - min_intensity) > threshold) {
            left = c + new_scan_offset_col;
            hbar_bounds.push_back(up_down[0]);
            hbar_bounds.push_back(up_down[1]);
            break;
        }
    }

    up_down = {int(center_row) - column_radius, int(center_row) + column_radius};
    for (int c = int(center_column) + column_radius; c < cols; c += new_scan_offset_col) {
        bool edge_trigger = false;
        bool cross_encounter = false;
        int min_col_intensity = 255;

        for (int r = up_down[0] - target_offset; r < up_down[1] + target_offset; r += new_scan_offset_row) {
            if (r < 0 || r >= rows) return false;
            
            if (gradient(image[(r-new_scan_offset_row)*cols + c], image[r*cols+c]) > threshold) {
                cross_encounter = true;

                if (!edge_trigger) {
                    edge_trigger = true;
                    up_down[0] = r;
                }
            }

            else if (gradient(image[(r-new_scan_offset_row)*cols + c], image[r*cols+c]) < -threshold) {
                cross_encounter = true;
                if (edge_trigger) {
                    up_down[1] = r;

                    hbar_x.push_back(c);
                    hbar_y.push_back((up_down[1] + up_down[0]) / 2.0);

                    break;
                }
            }

            if (image[r*cols+c] < min_col_intensity) {
                min_col_intensity = image[r*cols+c];
            }
        }


        if (!cross_encounter && abs(min_col_intensity - min_intensity) > threshold) {
            right = c - new_scan_offset_col;

            hbar_bounds.push_back(up_down[0]);
            hbar_bounds.push_back(up_down[1]);

            break;
        }

    }

    if (left == -1 || right == -1 || hbar_bounds.size() != 4) return false;

    if (abs((hbar_bounds[1] - hbar_bounds[0]) - (hbar_bounds[3] - hbar_bounds[2])) > 3*target_offset) return false;

    if (hbar_bounds[1] - hbar_bounds[0] > (right - left) / 2) return false;

    if(!corr_coef_check(hbar_x, hbar_y)) return false;

    center_column = (right + left) / 2.0;

    for (auto t : targets) {
        if (curr_target && t == curr_target) continue;
        else if ((t->center.row - t->radius) <= center_row && center_row <= (t->center.row + t->radius) && 
                 (t->center.col - t->radius) <= center_column && center_column <= (t->center.col + t->radius)) return false;
    }

    vector<double> up_vec = {(vbar_bounds[1] + vbar_bounds[0]) / 2.0 - center_column, double(center_row - top)};
    if (all_of(up_vec.begin(), up_vec.end(), [](double i) { return i == 0; })) return false;
    vector<double> up_unit = unitVec(up_vec);

    vector<double> down_vec = {(vbar_bounds[3] + vbar_bounds[2]) / 2.0 - center_column, double(center_row - bottom)};
    if (all_of(down_vec.begin(), down_vec.end(), [](double i) { return i == 0; })) return false;
    vector<double> down_unit = unitVec(down_vec);

    vector<double> right_vec = {double(right - center_column), center_row - (hbar_bounds[3] + hbar_bounds[2]) / 2.0};
    if (all_of(right_vec.begin(), right_vec.end(), [](double i) { return i == 0; })) return false;
    vector<double> right_unit = unitVec(right_vec);

    vector<double> left_vec = {double(left - center_column), center_row - (hbar_bounds[1] + hbar_bounds[0]) / 2.0};
    if (all_of(left_vec.begin(), left_vec.end(), [](double i) { return i == 0; })) return false;
    vector<double> left_unit = unitVec(left_vec);

    if (abs(L2Norm(up_vec) - L2Norm(right_vec)) > 3 * target_offset) return false;

    if (L2Norm(down_vec) < target_offset || L2Norm(left_vec) < target_offset) return false;

    // DEBUG
    // image[top*cols + (vbar_bounds[1] + vbar_bounds[0])/2] = 255;
    // image[bottom*cols + (vbar_bounds[3] + vbar_bounds[2])/2] = 255;
    // image[(hbar_bounds[1]+hbar_bounds[0])/2*cols + left] = 255;
    // image[(hbar_bounds[3]+hbar_bounds[2])/2*cols + right] = 255;
    // cout << "Center at: (" << center_row << "," << center_column << ")" << endl;
    // cout << "Orthogonals: " << dot(up_unit, right_unit) << "," << dot(up_unit, left_unit) << "," << dot(left_unit, down_unit) << "," << dot(right_unit, down_unit) << endl;
    // cout << "Parallels: " << dot(up_unit, down_unit) << "," << dot(right_unit, left_unit) << endl;
    // END DEBUG

    if (abs(dot(up_unit, right_unit)) > 0.35) return false;
    if (abs(dot(up_unit, left_unit)) > 0.35) return false;
    if (abs(dot(left_unit, down_unit)) > 0.35) return false;
    if (abs(dot(right_unit, down_unit)) > 0.35) return false;
    if (abs(abs(dot(up_unit, down_unit)) - 1) > 0.15) return false;
    if (abs(abs(dot(right_unit, left_unit)) - 1) > 0.15) return false;

    center = {int(ceil(center_row)), int(ceil(center_column))};
    radius = max(bottom - top, right - left) / 2;
    
    // cout << "Center at: (" << center_row << "," << center_column << ")" << " with radius = " << radius << endl;
    
    return true;
}
