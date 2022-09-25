//
// Created by phyorch on 24/10/18.
//

#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core_c.h"
#include <boost/format.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

string input_path = "/home/phyorch/MasterResearch/Data/2011_09_26/data/2011_09_26_drive_0048_sync/velodyne_points/data/0000000020.bin";
string image_path = "/home/phyorch/MasterResearch/Data/2011_09_26/data/2011_09_26_drive_0048_sync/image_02/data/0000000020.png";
string output_path = "/home/phyorch/MasterResearch/Data/0000000000.pcd";
string depth_map_path = "/home/phyorch/MasterResearch/Data/depth_map.jpg";
string depth_image_path = "/home/phyorch/MasterResearch/Data/depth_image.jpg";







void readKittiPclBinData(std::string &in_file, std::string& out_file) {
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        if (i >= 250111 && i < 255000) {
            cout << i << "   " << point.x << "  " << point.y << "  " << point.z << "    " << point.intensity << endl;
        }
        points->push_back(point);
    }
    input.close();
//    g_cloud_pub.publish( points );

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write<pcl::PointXYZI>(out_file, *points, false);
}


void projectPoint(cv::Mat &depth_map, pcl::PointXYZI &point, Eigen::Matrix3f &R, Eigen::Vector3f &t){
    float fx = 984.2439;
    float fy = 980.8141;
    float cx = 690;
    float cy = 233.1966;
    Eigen::Vector3f p(point.x, point.y, point.z);
    //Eigen::Matrix3f R_ = R.transpose();
    //Eigen::Vector3f t_ = -R_ * t;
    Eigen::Vector3f p_transfered = R * p + t;
    //Eigen::Vector3f p_transfered = R * p + t;
    int u = int(fx / p_transfered[2] * p_transfered[0] + cx);
    int v = int(fy / p_transfered[2] * p_transfered[1] + cy);
    if(0<=u && 0<=v && u<depth_map.cols && v<depth_map.rows){
        //cout << "Point " << test << "is a valid point." << v << " " << u << endl;
        depth_map.at<uchar>(v,u) = sqrt(point.x*point.x + point.y*point.y);  //255
        //cout << depth_map.at<float>(v,u);
    }
}


void projectData(std::string &in_file, std::string& out_file, cv::Mat &depth_map, Eigen::Matrix3f R, Eigen::Vector3f t) {
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        projectPoint(depth_map, point, R, t);
//        if(i>80000 && i<85000){
//            projectPoint(depth_map, point, R, t);
//            //cout << point.x << "  " << point.y << "  " << point.z << "  " << point.intensity << endl;
//        }

    }
    input.close();

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
}



void projectPoint_kitti(cv::Mat &depth_map, pcl::PointXYZI &point, Eigen::Matrix4f T, Eigen::Matrix4f R, Eigen::Matrix<float,3,4> P){
    Eigen::Vector4f p(point.x, point.y, point.z, 1);
    Eigen::Vector4f p_ = T * p;
    Eigen::Vector4f p__ = R * p_;
    Eigen::Vector3f p_projected = P * p__;
//    p_projected[0] = p_projected[0] / p_projected[2];
//    p_projected[1] = p_projected[1] / p_projected[2];
    int u = int(p_projected[0] / p_projected[2]);
    int v = int(p_projected[1] / p_projected[2]);
    if(0<=u && 0<=v && u<depth_map.cols && v<depth_map.rows){
        //cout << "Point " << test << "is a valid point." << v << " " << u << endl;
        depth_map.at<uchar>(v,u) = sqrt(point.x*point.x + point.y*point.y);  //sqrt(point.x*point.x + point.y*point.y)
    }
}

void projectData_kitti(std::string &in_file, std::string& out_file, cv::Mat &depth_map, Eigen::Matrix4f T, Eigen::Matrix4f R, Eigen::Matrix<float,3,4> P) {
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        projectPoint_kitti(depth_map, point, T, R, P);

    }
    input.close();

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
}




void colorTransfer(cv::Mat& depth_map, cv::Mat& depth_image)
{
    // ת��Ϊα��ɫͼ�� �� �Ҷ�ͼ��
    for (int y = 0; y<depth_map.rows; y++)
    {
        for (int x = 0; x<depth_map.cols; x++)
        {
            uchar val = depth_map.at<uchar>(y, x);
            uchar r, g, b;
            if(val==0){
                r = g = b = 0;
            }
            else if(val<=10){
                r = 220;
                g = 20;
                b = 60;
                depth_image.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
            }
            else if(val<=15){
                r = 255;
                g = 255;
                b = 3;
                depth_image.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
            }
            else if(val<=20){
                r = 173;
                g = 255;
                b = 47;
                depth_image.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
            }
            else if(val<=35){
                r = 10;
                g = 255;
                b = 20;
                depth_image.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
            }
            else if(val<=50){
                r = 0;
                g = 220;
                b = 120;
                depth_image.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
            }
            else if(val<=60){
                r = 0;
                g = 100;
                b = 200;
                depth_image.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
            }
            else{
                r = 0;
                g = 0;
                b = 255;
                depth_image.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
            }
        }
    }
}



int main() {
    cv::Mat depthImage = cv::imread(image_path);
    cv::Mat depthMap = cv::Mat::zeros(depthImage.rows, depthImage.cols, CV_8UC1);
    //Simple model
//    Eigen::Matrix3f Rotation;
//    Rotation << 7.533745e-03, -9.999714e-01, -6.166020e-04,
//            1.480249e-02, 7.280733e-04, -9.998902e-01,
//            9.998621e-01, 7.523790e-03, 1.480755e-02;
//    Eigen::Vector3f translation(-4.069766e-03, -7.631618e-02, -2.717806e-01);
//    projectData(input_path, output_path, depthMap, Rotation, translation);
//    cv::Mat depthMapNormalized = depthMap;
//    cv::normalize(depthMap, depthMapNormalized);
//    depthMapNormalized = depthMapNormalized * 255;

    //cv::imwrite(depth_map_path, depthMap);

    //KITTI_model
    Eigen::Matrix4f velo_to_cam;
    velo_to_cam << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
                   1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
                   9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
                   0, 0, 0, 1;
    Eigen::Matrix4f R_rect_00_extended;
    R_rect_00_extended << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0,
                          -9.869795e-03, 9.999421e-01, -4.278459e-03, 0,
                          7.402527e-03, 4.351614e-03, 9.999631e-01, 0,
                          0, 0, 0, 1;
    Eigen::Matrix<float,3,4> P_rect_00;
    P_rect_00 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00,
                 0.000000e+00, 7.215377e+02, 1.728540e+02, 0.000000e+00,
                 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00;
    projectData_kitti(input_path, output_path, depthMap, velo_to_cam, R_rect_00_extended, P_rect_00);


    //depthMap = depthMap * 3;
    cout << depthMap;
    colorTransfer(depthMap, depthImage);
    cv::imwrite(depth_map_path, depthMap);
    cv::imwrite(depth_image_path, depthImage);

    //readKittiPclBinData(input_path, output_path);
    return 0;
}

/*we need to consider the sky region*/