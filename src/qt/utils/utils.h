#ifndef QT_UTILS_H
#define QT_UTILS_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <fstream>
#include <iostream>
// 文件夹操作
#include <vector>
#include "groundRemove/include/cloud.h"
#include <QImage>

namespace utils
{
   /**
   * @brief Initialize from 3d points.
   *
   * @param[in]  points  The points
   */
    void ReadKittiBinCloudByPath(const std::string & path, Cloud & cloud);
    Cloud::Ptr ReadKittiBinCloudByPath(const std::string & path);

    void ReadKittiImageByPath(const std::string & path, cv::Mat & img);
    
    void ReadKittiFileByDir(const std::string & dir, std::vector<std::string> &fileNames);

    QImage MatToQImage(const cv::Mat &image);

    point camera_to_lidar(float x, float y, float z, 
				const Eigen::MatrixXd & T_VELO_2_CAM, 
				const Eigen::MatrixXd & R_RECT_0);

    Cloud center_to_corner_box3d(const point & center, 
							 const float & h,
							 const float & w,
							 const float & l,
							 float yaw);  

    std::vector<Cloud::Ptr> ReadKittiLabelBoxByPath(const std::string & path,
                                                    Eigen::MatrixXd MATRIX_T_VELO_2_CAM,
                                                    Eigen::MatrixXd MATRIX_R_RECT_0,
                                                    int dataLen = 15);

}
#endif