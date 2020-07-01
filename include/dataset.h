/*
 * dataset.h
 *
 *  Created on: Jul 1, 2020
 *      Author: aman_virmani
 */

#ifndef INCLUDE_DATASET_H_
#define INCLUDE_DATASET_H_
/*
#include <iostream>
#include <string>
#include <algorithm>
#include <event.h>
#include <object.h>

//#include <stoi>


// Pretty print

#define _header(str) std::string("\033[95m" + std::string(str) + "\033[0m")
#define _plain(str) std::string("\033[37m" + std::string(str) + "\033[0m")
#define _blue(str) std::string("\033[94m" + std::string(str) + "\033[0m")
#define _green(str) std::string("\033[92m" + std::string(str) + "\033[0m")
#define _yellow(str) std::string("\033[93m" + std::string(str) + "\033[0m")
#define _red(str) std::string("\033[91m" + std::string(str) + "\033[0m")
#define _bold(str) std::string("\033[1m" + std::string(str) + "\033[0m")
#define _underline(str) std::string("\033[4m" + std::string(str) + "\033[0m")


class Dataset {
public:
    // Camera params
    static std::string dist_model= "equidistant";
    static std::string image_topic, event_topic, cam_pos_topic;

    // Calibration matrix
    static float fx, fy, cx, cy, k1, k2, k3, k4, p1, p2;

    // Camera resolution
    static unsigned int res_x, res_y;
    // Other parameters
    static std::string window_name;

private:
    static bool read_params (std::string path) {
        std::ifstream ifs;
        ifs.open(path, std::ifstream::in);
        if (!ifs.is_open()) {
            std::cout << _red("Could not open camera parameter file file at ")
                      << path << "!" << std::endl;
            return false;
        }

        const std::string& delims = ":";
        while (ifs.good()) {
            std::string line;
            std::getline(ifs, line);
            line = trim(line);
            auto sep = line.find_first_of(delims);

            std::string key   = line.substr(0, sep);
            std::string value = line.substr(sep + 1);
            key = trim(key);
            value = trim(value);

            if (key == "res_x")
                Dataset::res_x = std::stoi(value);

            if (key == "res_y")
                Dataset::res_y = std::stoi(value);

            if (key == "dist_model")
                Dataset::dist_model = value;

            if (key == "ros_image_topic")
                Dataset::image_topic = value;

            if (key == "ros_event_topic")
                Dataset::event_topic = value;

            if (key == "ros_pos_topic")
                Dataset::cam_pos_topic = value;
        }
        ifs.close();

        std::cout << _green("Read camera parameters: \n")
                  << "\tres:\t" << Dataset::res_y << " x " << Dataset::res_x << "\n"
                  << "\tdistortion model:\t" << Dataset::dist_model << "\n"
                  << "\tros image topic:\t" << Dataset::image_topic << "\n"
                  << "\tros event topic:\t" << Dataset::event_topic << "\n"
                  << "\tros camera pose topic:\t" << Dataset::cam_pos_topic << "\n";
        return true;
    }

public:
        // Project 3D point in camera frame to pixel
    template<class T> static void project_point(T p, int &u, int &v) {
        if (Dataset::dist_model == "radtan") {
            Dataset::project_point_radtan(p, u, v);
        } else if (Dataset::dist_model == "equidistant") {
            Dataset::project_point_equi(p, u, v);
        } else {
            std::cout << _red("Unknown distortion model! ") << Dataset::dist_model << std::endl;
        }
    }

    template<class T> static void project_point_equi(T p, int &u, int &v) {
        u = -1; v = -1;
        if (p.z < 0.001)
            return;

        float x_ = p.x / p.z;
        float y_ = p.y / p.z;
        float r = std::sqrt(x_ * x_ + y_ * y_);
        float th = std::atan(r);

        float th2 = th * th;
        float th4 = th2 * th2;
        float th6 = th2 * th2 * th2;
        float th8 = th4 * th4;
        float th_d = th * (1 + Dataset::k1 * th2 + Dataset::k2 * th4 + Dataset::k3 * th6 + Dataset::k4 * th8);

        float x__ = x_;
        float y__ = y_;
        if (r > 0.001) {
            x__ = (th_d / r) * x_;
            y__ = (th_d / r) * y_;
        }

        v = Dataset::fx * x__ + Dataset::cx;
        u = Dataset::fy * y__ + Dataset::cy;
    }

    static cv::Mat undistort(cv::Mat &img) {
        cv::Mat ret;
        cv::Mat K = (cv::Mat1d(3, 3) << Dataset::fx, 0, Dataset::cx, 0, Dataset::fy, Dataset::cy, 0, 0, 1);

        if (Dataset::dist_model == "radtan") {
            cv::Mat D = (cv::Mat1d(1, 4) << Dataset::k1, Dataset::k2, Dataset::p1, Dataset::p2);
            cv::undistort(img, ret, K, D, 1.0 * K);
        } else if (Dataset::dist_model == "equidistant") {
            cv::Mat D = (cv::Mat1d(1, 4) << Dataset::k1, Dataset::k2, Dataset::k3, Dataset::k4);
            cv::fisheye::undistortImage(img, ret, K, D, 1.0 * K);
        } else {
            std::cout << _red("Unknown distortion model! ") << Dataset::dist_model << std::endl;
        }

        return ret;
    }
};


#endif *//* INCLUDE_DATASET_H_ */
