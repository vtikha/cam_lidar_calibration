/*
 * Copyright 2023 Australian Centre For Robotics
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * 
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Author: Darren Tsai
 */

#include "cam_lidar_calibration/load_params.h"
#include <string>
#include <vector>

namespace cam_lidar_calibration
{
    void loadParams(const ros::NodeHandle& n, initial_parameters_t& i_params)
    {
        int cb_w, cb_h, w, h, e_x, e_y, i_width, i_height;
        n.getParam("camera_topic", i_params.camera_topic);
        n.getParam("camera_info", i_params.camera_info);
        n.getParam("lidar_topic", i_params.lidar_topic);
        n.getParam("chessboard/pattern_size/width", cb_w);
        n.getParam("chessboard/pattern_size/height", cb_h);
        i_params.chessboard_pattern_size = cv::Size(cb_w, cb_h);
        n.getParam("chessboard/square_length", i_params.square_length);
        n.getParam("chessboard/board_dimension/width", w);
        n.getParam("chessboard/board_dimension/height", h);
        i_params.board_dimensions = cv::Size(w, h);
        n.getParam("chessboard/translation_error/x", e_x);
        n.getParam("chessboard/translation_error/y", e_y);
        i_params.cb_translation_error = cv::Point3d(e_x, e_y, 0);

        std::string distortion_model;
        n.getParam("distortion_model", distortion_model);

        std::vector<double> K, D;
        n.getParam("K", K);
        n.getParam("D", D);

        std::cout << "dist model is " << distortion_model << std::endl;

        if (distortion_model == "equidistant" or distortion_model == "fisheye") {
            i_params.fisheye_model = true;
        // Pinhole
        } else 
        {
            i_params.fisheye_model = false;
        }        

        i_params.cameramat.at<double>(0, 0) = K[0];
        i_params.cameramat.at<double>(0, 2) = K[2];
        i_params.cameramat.at<double>(1, 1) = K[4];
        i_params.cameramat.at<double>(1, 2) = K[5];
        i_params.cameramat.at<double>(2, 2) = 1;

        i_params.distcoeff.at<double>(0) = D[0];
        i_params.distcoeff.at<double>(1) = D[1];
        i_params.distcoeff.at<double>(2) = D[2];
        i_params.distcoeff.at<double>(3) = D[3];
        i_params.distcoeff.at<double>(4) = D[4];
        i_params.distcoeff.at<double>(5) = D[5];
        i_params.distcoeff.at<double>(6) = D[6];
        i_params.distcoeff.at<double>(7) = D[7];
        i_params.distcoeff.at<double>(8) = D[8];
        i_params.distcoeff.at<double>(9) = D[9];
        i_params.distcoeff.at<double>(10) = D[10];
        i_params.distcoeff.at<double>(11) = D[11];
        i_params.distcoeff.at<double>(12) = D[12];
        i_params.distcoeff.at<double>(13) = D[13];

    }
}  // namespace cam_lidar_calibration

