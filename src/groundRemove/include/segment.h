#ifndef SEGMENT_H_
#define SEGMENT_H_

#include <list>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <mutex>
// #include <geometry_msgs/Point.h>
#include "cloud.h"
#include "bin.h"
#include <array>

class Segment
{
public:
    typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;

    typedef std::pair<double, double> LocalLine;

private:
    // const double max_slope_;
    // const double max_error_;
    // const double long_threshold_;
    // const double max_long_height_;
    // const double max_start_height_;
    // const double sensor_height_;

    double max_slope_;
    double max_error_;
    double long_threshold_;
    double max_long_height_;
    double max_start_height_;
    double sensor_height_;
    double r_max_bin_;
    double r_min_bin_;

    /*
    //另外添加的参数*/
    double tHmin_;
    double tHmax_;
    double tHDiff_;
    double hSensor_;
    //////////////////

    // split and merge
    double min_split_dist;
    ///////

    std::vector<Bin> bins_;

    std::list<Line> lines_;
    std::list<LocalLine> linesSlopes_;

    // std::mutex mutex_lock;

    std::vector<int> binsIdx;
    // 
    double theta_start, theta_end, angle_resolution;
    double max_dist_to_line;

public:
    Segment(const unsigned int& n_bins,
                 const double& max_slope,
                 const double& max_error,
                 const double& long_threshold,
                 const double& max_long_height,
                 const double& max_start_height,
                 const double& sensor_height,
                 const double& r_max_bin,
                 const double& r_min_bin,
                 const double& tHmin,
                 const double& tHmax,
                 const double& tHDiff,
                 const double& hSensor,
                 const double& min_split_distm,
                 const double& theta_start,
                 const double& theta_end,
                 const double& angle_resolution,
                 const double& max_dist_to_line);

                 
    // Segment & operator=(Segment &);
    inline long getLineSize() const {return lines_.size();}
    inline Bin& operator[](const size_t & index){return bins_[index];}

    inline std::vector<Bin>::iterator begin(){return bins_.begin();}

    inline std::vector<Bin>::iterator end(){return bins_.end();}    
    
    void fitSegmentLines();

    LocalLine fitLocalLine(std::list<Bin::MinZPoint> & points);

    double getMaxError(std::list<Bin::MinZPoint> & line_points, LocalLine & line);

    Line localLineToLine(const LocalLine &line, std::list<Bin::MinZPoint> & points);

    void getLines(std::list<Line> *segment_lines);

    /*后来添加的*/
    void updateHeightAndGround();

    void gaussSmoothen(const double & sigma, const int &samples);

    void gaussSmoothenLine(const double & sigma, const int &samples);

    void computeHDiffAdjacentCell();

    void outlierFilter();

    // 第二种分段线性函数
    // 第二种 fit SegmentLines() 方法  split-and-merge 方法
    void splitAndMerger();
    void splitAndMerger(const int start_idx, const int end_idx);

    // 利用俩侧端点预测直线
    LocalLine endpointFit(const std::list<Bin::MinZPoint> & curr_line_points);

    // 点到标准直线的距离公式
    // double distPointToLine(const Bin::MinZPoint & pt, std::array<double>)

    // merge Line 如果合理的话
    void mergeLine();

    // 挑选出合适的 line
    void filterLine();

    // 输入距离 给出所属的 bin 的 索引
    int getPointBinIdx(const double & dist, 
                        const double & bin_step, 
                        double r_min = 2.8);


    double verticalDistanceToLine(const double &d, const double &z);

    // 根据角分辨率来划分 BinIdx 的方法
    int getBinIdxFromDist(const double & d);

    // 使用自己拟合的直线判断该点是不是地面点
    void updateGroundLine();    

// 记录属于那个 segment idx
    int segmentIdx;
    bool isDebug;

}; 
#endif