#ifndef GROUNDREMOVE_H_
#define GROUNDREMOVE_H_

// #include <ros/ros.h>
#include <Eigen/Core>
#include "bin.h"
#include "segment.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <ostream>
#include <mutex>
#include <algorithm>

struct GroundSegmentationParams{
    GroundSegmentationParams():
    /*另外添加的参数*/
    visualize(true),
    tHmin(-2.15),
    tHmax(1.0),
    tHDiff(0.2),
    hSensor(1.73),  
    r_min_bin(0.05), 
    r_max_bin(2),
    // r_min_square(3 * 3),
    // r_min_square(3.8 * 3.8),
    r_min_square(3 * 3),
    // r_max_square(20 * 20),
    r_max_square(120 * 120),
    // n_bins(30),
    n_bins(120),
    // n_segments(180),
    n_segments(240),
    // n_segments(432),
    // max_dist_to_line(0.15),
    max_dist_to_line(0.15),
    // max_dist_to_line(0.2),
    max_slope(0.35),  // 控制斜率， 斜率是直线拟合求出来的
    max_error_square(0.01),
    long_threshold(2.0),
    max_long_height(0.2),
    max_start_height(0.3),
    sensor_height(1.70),
    line_search_angle(2),  // 改为搜索的 segments 数量
    //###########
    n_threads(4),
    //###########
    min_split_dist(0.1),
    theta_start(65.1277),  // 90 - 24.8723
    theta_end(2),
    // angle_resolution(0.41)
    angle_resolution(0.41)
    { }
    // 是否可视化
    bool visualize;
    /*另外添加的参数*/
    double tHmin;
    double tHmax;
    double tHDiff;
    double hSensor;
    ///////////////
    // bin 的设置
    double r_min_bin;
    double r_max_bin;


    // 最小范围距离
    double r_min_square;

    double r_max_square;
    int n_bins;
    int n_segments;
    // 距离直线段最大允许距离， 被判断为地面的最大允许距离直线的距离
    double max_dist_to_line;
    double max_slope;
    double max_error_square;
    // 认为俩个点足够远的条件
    double long_threshold;
    // 
    double max_long_height;
    // 足够重新开始
    // Maximum heigh of starting line to be labelled ground.
    // 斜率不能太大， 足够长，当前点计算的 z 与 期望的 z 不能太大
    double max_start_height;
    // 传感器距离地面的高度
    double sensor_height;
    // 左右直线的搜索幅度距离
    double line_search_angle;
    // 线程数量
    int n_threads;

    double min_split_dist;

    double theta_start;

    double theta_end;

    double angle_resolution;
};

typedef std::pair<point, point> PointLine;

class GroundSegmentation
{
public:
    GroundSegmentationParams params_;
    GroundSegmentation(const GroundSegmentationParams & params = GroundSegmentationParams());
    // 全局地图
    std::vector<Segment> segments_;

    // Bin 的 index of every point
    std::vector<std::pair<int, int>> bin_index_;

    // 2D cooordinates (d, z) of every point in its respective segment
    std::vector<Bin::MinZPoint> segment_coordinates_;

    // 记录每个 bin 中使用的是那个点的索引
    // std::vector<std::vector<int>> bin2PointIdx_;

    // 计数用的
    std::atomic<int> count_;
    // 发布线段用的
    // 分割函数   19423264
    void segment(const Cloud & cloud, std::vector<int> * segmentation);

    void insertPoints(const Cloud & cloud);

    void insertPointThread(const Cloud & cloud, const size_t start_idx, const size_t end_idx);

    point minZPointTo3d(const Bin::MinZPoint & mzpoint, const double & angle);
    point minZPointTo3d(const double & d, const double & angle);

    void printParams();

    void getLines(std::list<PointLine> *lines);

    void lineFitThread(const int & start_idx,
                       const int & end_idx,
                       std::list<PointLine> *lines,
                       std::mutex *lines_mutex);

    /* 后面添加的 */
    void updateGround();
    
    void updateGroundThread(const size_t & start_idx, const size_t & end_idx);

    void applayMedianFilter();

    void applayMedianFilterMinZ();
    
    void outlierFilter();

    void outlierFilterThread(const size_t & start_idx, const size_t & end_idx);

    void groundAndElevated(const Cloud & cloud, std::vector<int> * segmentation);

    void groundAndElevatedThread(const Cloud & cloud,
                                std::vector<int> * segmentation,
                                int start_idx,
                                int end_idx
                                );

    void assignCluster(std::vector<int> * segmentation);

    void assignClusterThread(const unsigned int &start_index,
                                             const unsigned int &end_index,
                                             std::vector<int> *segmentation);

    
    int getBinIdxFromDist(const double & d);

    // 添加数字信息的工具
    // 根据 line 判断 bin 中的点是否是地面点
    void updateBinGround();
    void updateBinGroundThread(const size_t & start_idx, const size_t & end_idx);

    // 根据但前所在的 bin 是否是地面点判断
    void assignClusterByLine(std::vector<int> * segmentation);

    void assignClusterByLineThread(const unsigned int &start_index,
                                             const unsigned int &end_index,
                                             std::vector<int> *segmentation);

    // 获取可视化插入点    
    void getInsertedPoint(Cloud & cloud, Cloud & insertCloud);

    // 获取可视化的线段的点
    void getLinesPoint(Cloud & linesPoint);

    // 可视化的 lines
    std::list<PointLine> lines;

    // 设置选择屏幕上选择的点
    void setClickedPoint(double & x, double & y);

    // 选择屏幕上的集合
    void setSelectObjectID(std::vector<int> & selects);

// 选择的点
    bool hasClickedPoint;
    // point clickedPoint;
public:
    Cloud::Ptr thisCloud = nullptr;
    point clickedPoint;
    int debugSegIdx = -1;
    int debugBinIdx = -1;

    std::vector<int> selectObjectIDs;
};

class SegmentaionNode
{
private:
    GroundSegmentationParams params_;

public:
    SegmentaionNode(){}
    SegmentaionNode(const GroundSegmentationParams &params):
    params_(params),
    segmenter(params_)  // 初始化对象， 最好放在这儿， 不要放在下面的位置
    {
        // segmenter(params_);
        // segmenter.params_ = params_;
    }

    void scanCallBack(Cloud & cloud, Cloud & ground_cloud, Cloud & obstacle_cloud)
    {
        Cloud & cloudfilter = cloud;
        // for (auto it = cloud.begin(); it != cloud.end(); ++it)
        // {
        //     if ((it->x() * it->x() + it->y() * it->y()) <= params_.r_min_square  && 
        //         (std::abs(it->z() - params_.sensor_height) > params_.max_dist_to_line))
        //         continue;
        //     cloudfilter.emplace_back(*it);
        // }
        
        // GroundSegmentation segmenter(params_);
        // labels all cloud points to be ground of not
        std::vector<int> labels;
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        /////////////////////////////////////////////////////////////
        // fprintf(stderr, "---------- before segment ------%d\n", cloudfilter.size());
        segmenter.segment(cloudfilter, &labels);      
        // fprintf(stderr, "after segment %d\n", cloudfilter.size());         
        for (size_t i = 0; i < cloudfilter.size(); ++i)
        {
            if (labels[i] == 1)
                ground_cloud.push_back(cloudfilter[i]);
            else
                obstacle_cloud.push_back(cloudfilter[i]);
        }

        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms = end - start;
        std::cout << "groundRemove took about " << fp_ms.count() << " ms" << std::endl;
        // code end here
    }

    void getInsertedPoint(Cloud & cloud, Cloud & insertCloud)
    {
        segmenter.getInsertedPoint(cloud, insertCloud);
    }

    void getLinesPoint(Cloud & linesPoint)
    {
        segmenter.getLinesPoint(linesPoint);
    }

    void setClickedPoint(double & x, double & y)
    {
        segmenter.setClickedPoint(x, y);
    }

    void setSelectObjectID(std::vector<int> & selects)
    {
        segmenter.setSelectObjectID(selects);
    }

private:
    GroundSegmentation segmenter;
};
#endif /*GROUNDREMOVE*/
