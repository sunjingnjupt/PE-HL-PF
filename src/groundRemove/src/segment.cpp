#include "segment.h"
#define STOP getchar();

// Segment::Segment(const unsigned int& n_bins,
//                  const double& max_slope,
//                  const double& max_error,
//                  const double& long_threshold,
//                  const double& max_long_height,
//                  const double& max_start_height,
//                  const double& sensor_height) :
//                  bins_(n_bins),
//                  max_slope_(max_slope),
//                  max_error_(max_error),
//                  long_threshold_(long_threshold),
//                  max_long_height_(max_long_height),
//                  max_start_height_(max_start_height),
//                  sensor_height_(sensor_height){}
//代码实现

Segment::Segment(const unsigned int& n_bins,
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
                 const double& min_split_dist,
                 const double& theta_start,
                 const double& theta_end,
                 const double& angle_resolution,
                 const double& max_dist_to_line):
				 bins_(n_bins),
				 max_slope_(max_slope),
                 max_error_(max_error),
                 long_threshold_(long_threshold),
                 max_long_height_(max_long_height),
                 max_start_height_(max_start_height),
                 sensor_height_(sensor_height),
                 r_max_bin_(r_max_bin),
                 r_min_bin_(r_min_bin),
                //  mutex_lock(),
                 tHmin_(tHmin),
                 tHmax_(tHmax),
                 tHDiff_(tHDiff),
                 hSensor_(hSensor),
                 min_split_dist(min_split_dist),
                 binsIdx(n_bins),
                 theta_start(theta_start),
                 theta_end(theta_end),
                 angle_resolution(angle_resolution),
                 max_dist_to_line(max_dist_to_line)
                 {
                    segmentIdx = 0;
                    isDebug = false;
                 }


// 拟合每个 Segment 线段
void Segment::fitSegmentLines()
{
    // 后期有很多改进的位置， 比如 直线拟合的算法需要改进
    // char stop = getchar();

    // for (auto it = bins_.begin(); it != bins_.end(); ++it)
    // {
    //     if (it->hasPoint())
    //         it->print();
    // }

    // std::cout << "fitSegmentLines start ..." << std::endl;
    std::vector<Bin>::iterator firstIter = bins_.begin();
    while(!firstIter->hasPoint())
    {
        firstIter++;
        // 找不到点的情况
        if (firstIter == bins_.end())
            return;
    }

    // firstIter->print();
    // 找到了第一个非空的点
    // 添加第一个点
    bool is_long_line = false;
    double curr_ground_height = -sensor_height_;
    std::list<Bin::MinZPoint> curr_line_points(1, firstIter->getMinZPoint());
    LocalLine currLine = std::make_pair(0, 0);

    // std::cout << "currLine out for loop : " << currLine.first << " " << currLine.second << "\n";
    // ###################################################################
    // int nums = 0;
    // for (auto it = bins_.begin(); it != bins_.end(); ++it)
    // {
    //     if (it->hasPoint()) nums++;
    // }
    // std::cout << "bins_'s has null empty point : " << nums << std::endl;
    // ###################################################################

    for (auto currPoint = firstIter + 1; currPoint != bins_.end(); ++currPoint)
    {
        //
        // std::cout << "\n\n\ncurrPoint is :";
        // currPoint->print();
        if (!currPoint->hasPoint()) continue; // 没有点就直接迭代下一个点
        Bin::MinZPoint curr_point = currPoint->getMinZPoint();

        //
        // std::cout << "curr_line_points :\n";
        // for (auto it = curr_line_points.begin(); it != curr_line_points.end(); ++it)
        // {
        //     std::cout << "(" << it->d << "," << it->z <<")\n";
        // }

        if (curr_point.d - curr_line_points.back().d > long_threshold_) 
        {
            is_long_line = true;   // 是比较长的
            // std::cout << "isLongLine " << curr_point.d  << "   "<<curr_line_points.back().d << "\n";            
        }
        

        // 当前的点的数目大于等于 2 个的时候， 
        if(curr_line_points.size() >= 2)
        {
            double expected_z = std::numeric_limits<double>::max();
            if (is_long_line && curr_line_points.size() > 2)
            {

                // 如果但前点适合这条直线， 那么我们期望的计算的他的高度为 expected_z
                expected_z = currLine.first * curr_point.d + currLine.second;
            }

            // STOP
            curr_line_points.push_back(curr_point);
            /*
            for (auto it = curr_line_points.begin(); it != curr_line_points.end(); ++it)
            {
                std::cout << "(" << it->d << "," << it->z <<")\n";
            }
            */
            currLine = fitLocalLine(curr_line_points);
            
            const double error = getMaxError(curr_line_points, currLine);

            // 判断是不是合理
            // std::cout << "error : " << error
            //           << "\nmax_slope : " << currLine.first
            //           << "\necpected_z - curr_point.z : " << std::fabs(expected_z - curr_point.z)
            //           << std::endl;
            //std::cout << "curr_line_points has " << curr_line_points.size() << " points" << std::endl;
            // for (auto it = curr_line_points.begin(); it != curr_line_points.end(); ++it)
            // {
            //     std::cout << it->d << "  ";
            // }

            if (error > max_error_ ||
                std::fabs(currLine.first) >  max_slope_||
                is_long_line && std::fabs(expected_z - curr_point.z) > max_long_height_)
            {
                
                // std::cout << "\nerror : " << error << "     " << max_error_
                //     << "\nmax_slope : " << currLine.first  << " " << max_slope_
                //     << "\nisLongLine : " << is_long_line 
                //     << "\necpected_z - curr_point.z : " << std::fabs(expected_z - curr_point.z)
                //     << "\nmax_long_height_ : " << max_long_height_
                //     << "\ncurrLine args: " << currLine.first << " " << currLine.second << "\n"
                //     << std::endl;
                // std::cout << "bad condition " << std::endl;
                // std::cout << "curr_line_points : " << curr_line_points.size() << std::endl;

                // STOP
                curr_line_points.pop_back();
                // 判断合理就添加地面线段
                if (curr_line_points.size() >= 3)
                {
                    // 只有三个点以三才会重新拟合直线
                    // 有一个坏点的情况， 如果前面只有俩个点显然不可能构成一条合理的直线
                    // 只有三个点才能表示一个合理的平面， 不然就是俩个点的凸尖
                    // 所以再次拟合后， 添加到线段中去
                    const LocalLine new_line = fitLocalLine(curr_line_points);
                    lines_.push_back(localLineToLine(new_line, curr_line_points));
                    // 最后一条直线所在的地面
                    curr_ground_height = new_line.first * curr_line_points.back().d + new_line.second;
                }
                curr_line_points.erase(curr_line_points.begin(), --curr_line_points.end());
                is_long_line = false;
                --currPoint; // 回退后以前一条直线所在的点的最后一个点为开始点， 继续迭代下去
            }
            else
            {
                // 好的点， 让直线继续伸展， 不需要做任何事情
            }
            
        }
        else
        {
            // STOP
            // 刚开始的地面的点是传感器的负数 也就是 -1.7， 主方向向下
            // std::cout << "curr_line_points.back().z : " << curr_line_points.back().z
            //           << "\ncurr_ground_height : " << curr_ground_height
            //           << "\nmax_start_height_ : " << max_start_height_ << "\n";
            //  点数不足俩个， 没法构成直线
            if (curr_point.d - curr_line_points.back().d < long_threshold_ &&
                 std::fabs(curr_line_points.back().z - curr_ground_height) < max_start_height_)
            {
                curr_line_points.push_back(curr_point);
            }
            else
            {
                curr_line_points.clear();
                curr_line_points.push_back(curr_point);// 新的起点
            }           
        }        
    }

    if (curr_line_points.size() > 2)
    {
        // 待会看一下效果
        //STOP
        const LocalLine new_line = fitLocalLine(curr_line_points);
        lines_.push_back(localLineToLine(new_line, curr_line_points));
    }

    // {
    //     // mutex_lock.lock();
    //     // std::cout << "this segment get about : " << lines_.size() << " lines" << std::endl;
    //     // mutex_lock.unlock();
    //     // std::cout << lines_.size() << std::endl;
    // }
    if (!lines_.size())
    {
        std::cout << "find a empty segment \n";
        for (auto it = bins_.begin(); it != bins_.end(); ++it)
        {
            if (it->hasPoint())
                it->print();
        }
        // STOP
    }
}


Segment::LocalLine Segment::fitLocalLine(std::list<Bin::MinZPoint> & points)
{
    // for (auto it = points.begin(); it != points.end(); ++it)
    // {
    //     printf("(%f, %f)\n ", it->d, it->z);
    // }

    const unsigned int n_points = points.size();
    Eigen::MatrixXd X(n_points, 2);
    Eigen::VectorXd Y(n_points);

    unsigned int counter = 0;
    for (auto iter = points.begin(); iter != points.end(); ++iter)
    {
        X(counter, 0) = iter->d;
        X(counter, 1) = 1;
        Y(counter) = iter->z;
        ++counter;
    }

    // 这里缺少一个 Eigen/Dense 头文件
    Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
    LocalLine line_result;

    line_result.first = result(0);
    line_result.second = result(1);
    return line_result;
}

double Segment::getMaxError(std::list<Bin::MinZPoint> & line_points, LocalLine & line)
{
    // double error = std::numeric_limits<double>::min();
    double max_error = 0;
    
    for (auto it = line_points.begin();it != line_points.end(); ++it)
    {
        const double residual = it->z - (line.first * it->d + line.second);
        const double error = residual * residual;
        if (error > max_error)
            max_error = error;
    }

    return max_error;
}

// 只需要保存俩个端点就好
Segment::Line Segment::localLineToLine(const LocalLine &line, std::list<Bin::MinZPoint> & points)
{
    Line line_res;
    double first_d = points.front().d;
    double first_z = points.front().d * line.first + line.second;

    double second_d = points.back().d;
    double second_z = points.back().d * line.first + line.second;

    line_res.first.d = first_d;
    line_res.first.z = first_z;
    line_res.second.d = second_d;
    line_res.second.z = second_z;

    return line_res;
}

void Segment::getLines(std::list<Segment::Line> *segment_lines)
{
    // std::cout << "bin has " << lines_.size() << "lines\n";
    *segment_lines = lines_;
}

void Segment::updateHeightAndGround()
{
    /*跟新最小值， 和跟新是否是地面*/
    for (std::vector<Bin>::iterator it = bins_.begin(); it != bins_.end(); ++it)
    {
        double zi = it->getMinZ();
        // bool show = false;
        // if (fabs(zi - (-0.3)) < 0.05)
        //     show = true;
        if (zi > tHmin_ && zi < tHmax_) it->updateHeight(zi);
        else if (zi >= tHmax_) it->updateHeight(hSensor_);
        else it->updateHeight(tHmin_);

        // if (show)
        //     printf("current zi %f\n",it->getHeight());
    }
    // 高斯滤波， 每个 segment
    gaussSmoothen(1,3);
    computeHDiffAdjacentCell();

    for (std::vector<Bin>::iterator it = bins_.begin(); it != bins_.end(); ++it)
    {
        if (it->getSmoothed() < tHmax_ && it->getHDiff() < tHDiff_)
        {
            it->updateGround();     
        }
        else if (it->getHeight() < tHmax_ && it->getHDiff() < tHDiff_)
        {
            it->updateGround();
        }   

    }
}

// void gaussSmoothenLine(const double & sigma, const int &samples);

void Segment::gaussSmoothenLine(const double & sigma, const int &samples)
{
    std::vector<double> kernel(samples);
    double mean = samples / 2;

    double sum = 0;
    for (int x = 0; x < samples; ++x)
    {
        kernel[x] = exp(-0.5 * (pow((x - mean) / sigma, 2.0))) / (2 * M_PI * sigma * sigma);
        // kernel[x] = exp(-0.5 * (pow((x - mean) / sigma, 2.0))) / (sigma * sqrt(2 * M_PI));
        sum += kernel[x];
    }

    // Normalize the kernel
    for (int x = 0; x < samples; ++x)
    {
        kernel[x] /= sum;
    }

    assert(kernel.size() == samples);

    int sampleSide = samples / 2;
    unsigned long ubound = bins_.size();

    for (long i = 0; i < ubound; ++i)
    {
        double smoothed = 0;
        for (long j = i - sampleSide; j <= i + sampleSide; ++j)
        {
            if (j >= 0 && j < ubound)
            {
                int sampleWeightIndex = sampleSide + (j - i);
                smoothed += kernel[sampleWeightIndex] * bins_[j].getHeight();
            }
        }
        bins_[i].updateSmoothedZ(smoothed);
    }    
}


void Segment::gaussSmoothen(const double & sigma, const int &samples)
{
    std::vector<double> kernel(samples);
    double mean = samples / 2;

    double sum = 0;
    for (int x = 0; x < samples; ++x)
    {
        kernel[x] = exp(-0.5 * (pow((x - mean) / sigma, 2.0))) / (2 * M_PI * sigma * sigma);
        // kernel[x] = exp(-0.5 * (pow((x - mean) / sigma, 2.0))) / (sigma * sqrt(2 * M_PI));
        sum += kernel[x];
    }

    // Normalize the kernel
    for (int x = 0; x < samples; ++x)
    {
        kernel[x] /= sum;
    }

    assert(kernel.size() == samples);

    int sampleSide = samples / 2;
    unsigned long ubound = bins_.size();

    for (long i = 0; i < ubound; ++i)
    {
        double smoothed = 0;
        for (long j = i - sampleSide; j <= i + sampleSide; ++j)
        {
            if (j >= 0 && j < ubound)
            {
                int sampleWeightIndex = sampleSide + (j - i);
                smoothed += kernel[sampleWeightIndex] * bins_[j].getHeight();
            }
        }
        bins_[i].updateSmoothed(smoothed);
    }    
}

void Segment::computeHDiffAdjacentCell()
{
    // for (int i = 0; i <  bins_.size(); ++i)
    // {
    //     if (i == 0)
    //     {
    //         double hD = bins_[i].getHeight() - bins_[i + 1].getHeight();
    //         bins_[i].updateHDiff(hD);
    //     }
    //     else if (i == (bins_.size() - 1))
    //     {
    //         double hD = bins_[i].getHeight() - bins_[i - 1].getHeight();
    //         bins_[i].updateHDiff(hD);
    //     }
    //     else
    //     {
    //         double preHD = bins_[i].getHeight() - bins_[i - 1].getHeight();
    //         double postHD = bins_[i].getHeight() - bins_[i + 1].getHeight();
    //         if (preHD > postHD)
    //             bins_[i].updateHDiff(preHD);
    //         else
    //             bins_[i].updateHDiff(postHD);
    //     }
        
    // }

    for (int i = 0; i < bins_.size(); ++i)
    {
        if (i == 0)
        {
            double hD = bins_[i].getHeight() - bins_[i + 1].getHeight();
            bins_[i].updateHDiff(hD);
        }
        else if (i == (bins_.size() - 1))
        {
            double hD = bins_[i].getHeight() - bins_[i - 1].getHeight();
            bins_[i].updateHDiff(hD);
        }
        else
        {
            double preHD = bins_[i].getHeight() - bins_[i - 1].getHeight();
            double postHD = bins_[i].getHeight() - bins_[i + 1].getHeight();
            if (preHD > postHD)
                bins_[i].updateHDiff(preHD);
            else
                bins_[i].updateHDiff(postHD);
        }
    }
}

void Segment::outlierFilter()
{
    for (int binIdx  = 1; binIdx < bins_.size() - 2; ++binIdx)
    {
        if (bins_[binIdx].isThisGround() &&
            bins_[binIdx + 1].isThisGround() &&
            bins_[binIdx - 1].isThisGround() &&
            bins_[binIdx + 2].isThisGround())
        {
            double height1 = bins_[binIdx - 1].getHeight();
            double height2 = bins_[binIdx].getHeight();
            double height3 = bins_[binIdx + 1].getHeight();
            double height4= bins_[binIdx + 2].getHeight();
            // 一开始， 就将 zi < tHmin 的高度设置为 tHmin, 凹 到地面以下的部分设置为 tHmin
            if (height1 != tHmin_ && height2 == tHmin_ && height3 != tHmin_)
            {
                double newH = (height1 + height3) / 2;
                bins_[binIdx].updateHeight(newH);
                bins_[binIdx].updateGround();
            }
            else if(height1 != tHmin_ && height2 == tHmin_ && height3 == tHmin_ && height4 != tHmin_)
            {
                double newH = (height1 + height4) / 2;
                bins_[binIdx].updateHeight(newH);
                bins_[binIdx].updateGround();
            }

        }
    }
}

void Segment::splitAndMerger()
{
    // 分段直线提取算法
    int numBin = bins_.size();
    // 高斯滤波最低点
    splitAndMerger(0, numBin);
    // 得到的 segment 的 line 是按顺序的， 递归的规则就是这样
    // for (auto it = lines_.begin(); it != lines_.end(); ++it)
    // {
    //     printf("%f -> %f\n", it->first.d, it->second.d);
    //     double angleRate = (it->second.z - it->first.z) / (it->second.d - it->first.d);
    //     printf("(%f, %f)\n", angleRate, it->first.z - angleRate * it->first.d);
    // }

    // 填充直线段, split and merge 的 merge 部分
    mergeLine();
    // fprintf(stderr, "max_slope_:%f\n", max_slope_);
}

// 合并且过滤一部分拟合错误的直线
void Segment::mergeLine()
{
    // 是否是选中的点， 落在这个 segment 中
    // 所以就会显示信息
    int numLine = lines_.size();
    if (numLine == 0) return;
    // 只有一条线的情况
    if (numLine == 1)
    {
        auto it = lines_.begin();
        auto it_slope = linesSlopes_.begin();
        if (it_slope->first > max_slope_)
        {
            lines_.erase(it);
            linesSlopes_.erase(it_slope);
        }
    }

    std::list<Segment::Line>::iterator firstLineIter = lines_.begin();
    std::list<Segment::LocalLine>::iterator firstSlopeIter = linesSlopes_.begin();
    std::list<Segment::Line>::iterator secondLineIter = lines_.begin();
    ++secondLineIter;
    std::list<Segment::LocalLine>::iterator secondSlopeIter = linesSlopes_.begin();
    ++secondSlopeIter;
    // double bin_step = (120 - 2.8) / (bins_.size());

    // printf("\n\n\n\n\n\n\n");
    // for(auto it = lines_.begin(); it != lines_.end(); ++it)
    // {
    //     it->first.print();
    //     printf(" ---->  ");
    //     it->second.print();
    //     printf("\nbinIdx %d -->  %d\n", getPointBinIdx(it->first.d, bin_step), getPointBinIdx(it->second.d, bin_step));
    // }

    for (;secondLineIter != lines_.end(); ++firstLineIter, ++secondLineIter, ++firstSlopeIter, ++secondSlopeIter)
    {
        // firstLineIter->second.print();
        // printf(" ---->  ");
        // secondLineIter->first.print();
        // printf("\n\n");
        // 距离太近不需要填充
        // printf("\nsecondLineIter->first.d - firstLineIter->second.d = %f", secondLineIter->first.d - firstLineIter->second.d);
        // const int binIdxFirst = getPointBinIdx(firstLineIter->second.d, bin_step);
        const int binIdxFirst = getBinIdxFromDist(firstLineIter->second.d);
        // const int binIdxSecond = getPointBinIdx(secondLineIter->first.d, bin_step);
        const int binIdxSecond = getBinIdxFromDist(secondLineIter->first.d);
        // printf("\n binIdxFirst %d, binIdxSecond %d ", binIdxFirst, binIdxSecond);
        if (std::abs(binIdxFirst - binIdxSecond) >= 4 || std::abs((secondLineIter->first.d - firstLineIter->second.d > 3))) continue;
        // double expect_z = secondLineIter->first.d * firstSlopeIter->first + firstSlopeIter->second;
        // 预测的与实际的高度相差不大的情况下， 插入一个线段
        LocalLine vec1, vec2;
        vec1.first = firstLineIter->second.d - firstLineIter->first.d;
        vec1.second = firstLineIter->second.z - firstLineIter->first.z;

        vec2.first = secondLineIter->first.d - firstLineIter->second.d;
        vec2.second = secondLineIter->first.z - firstLineIter->second.z;

        double cosAngle = (vec1.first * vec2.first + vec1.second * vec2.second) / (
                sqrt(vec1.first * vec1.first + vec1.second * vec1.second) * 
                sqrt(vec2.first * vec2.first + vec2.second * vec2.second));
        // if (std::abs(slope - firstSlopeIter->first) < 0.1)
        // 夹角越大值越小
        // printf("cos theate %f (%f)\n", std::abs(cosAngle), cos(5.0 / 180 * M_PI));
        // if (isDebug)
        // {
        //     fprintf(stderr, "current angle :%f\n", cosAngle / M_PI * 180);
        // }

        if (std::abs(cosAngle) > cos(6.0 / 180 * M_PI))
        {
            // printf("\n binIdxFirst %d, binIdxSecond %d ", binIdxFirst, binIdxSecond);
            // printf("\ninsert a  line of (%f, %f) --> (%f, %f)\n", 
            //                 firstLineIter->second.d,
            //                 firstLineIter->second.z,
            //                 secondLineIter->first.d,
            //                 secondLineIter->first.z);
            lines_.insert(secondLineIter, std::make_pair(firstLineIter->second, secondLineIter->first));
            double slope = (secondLineIter->first.z - firstLineIter->second.z) / (secondLineIter->first.d - firstLineIter->second.d);
            double cutb = firstLineIter->second.z - slope * firstLineIter->second.d;
            linesSlopes_.insert(secondSlopeIter, std::make_pair(slope, cutb));
            ++firstLineIter;
            ++firstSlopeIter;            
        }
    }

    // if (isDebug)
    // {
    //     fprintf(stderr, "after insert the line : %d\n", lines_.size());
    //     auto it_slope = linesSlopes_.begin();
    //     for(auto it = lines_.begin(); it != lines_.end(); ++it, ++it_slope)
    //     {
    //         it->first.print();
    //         fprintf(stderr, " ---->  ");
    //         it->second.print();
    //         fprintf(stderr, "    (slope %f  )", it_slope->first);
    //         fprintf(stderr, "\nbinIdx %d -->  %d\n",getBinIdxFromDist(it->first.d), getBinIdxFromDist(it->second.d));
    //     }
    //     fprintf(stderr, "\n\n\n");
    // }

    // 过滤不需要的直线
    bool is_long_line = false;
    double curr_ground_height  = -sensor_height_;
    auto it = lines_.begin();
    auto it_slope = linesSlopes_.begin();
    LocalLine currLine = std::make_pair(0, -sensor_height_);
    // 如果是添加一个比较长的线段， 那么其斜率肯定不能过大， 不然容易形成很大的天空点和线段
    double long_line_slope_threshold = max_slope_ / 3;
    for (; it != lines_.end();)
    {
        // if (it->second.d - it->first.d > long_threshold_)
        //     is_long_line = true;
        // double max_start = ((1 - 0.4) / (120 - 3.4)) * (it->first.d - 3.4);
        // if (std::abs(it_slope->first) >= 0.15 && it->first.d < 4.0)
        // {
        //     printf("\n\n\n\n\n\nslope %f\n", it_slope->first);
        //     it->first.print();
        //     printf("---->");
        //     it->second.print();
        //     printf("\nmax_slope_ %f\n", max_slope_);
        // }
        // printf("\n\nlines_.size %ld\n", lines_.size());
        // printf("lineSlopes_.size %ld\n", linesSlopes_.size());
        // 如果是第一跟线段， 那么允许起始的线段高度变化大一点， 有些是起始点为 -1.73, 然而有本分起始点是 -2.20 米
        double max_start_height_factor = 1.0;
        if (it == lines_.begin())
        {
            max_start_height_factor = 1.5;
        }
        // 如果但前线段过长， 那么就只允许其角度抬起的很小
        // 如果但前线段不长， 那么就让预设的值来进行
        double curren_line_length = (it->second.d - it->first.d);
        if (std::abs(it_slope->first) > max_slope_ ||
            std::abs(currLine.first * it->first.d + currLine.second - it->first.z) > 
            (max_start_height_ * max_start_height_factor) ||
            (curren_line_length > 1 && std::abs(it_slope->first) > long_line_slope_threshold))
        {
            // 第三个或者条件为， 如果线段过长， 且幅度大于预设值的三分之一， 就删除掉
            // if (it->first.d < 8)
            // {
            //     printf("\n\n\n\n\ncurrLine.first * it->first.d + currLine.second - it->first.z %f\n", 
            //                         currLine.first * it->first.d + currLine.second - it->first.z);
            //     printf("\nerase a line\n");
            //     it->first.print();
            //     it->second.print();
            //     printf("slope %f\n", it_slope->first);

            // }

            lines_.erase(it++);
            linesSlopes_.erase(it_slope++);
        }
        else
        {
            ++it;
            currLine = *it_slope;
            ++it_slope;
            //currLine = *it_slope;// 保留上一个对呀的 currLine 而非是删除的 currLine 这里是个 bug

            // curr_ground_height = it->second.z;
        }    
        // printf("after lines_.size %ld\n", lines_.size());
        // printf("after lineSlopes_.size %ld\n", linesSlopes_.size());
    }

    // it = lines_.begin();
    // it_slope = linesSlopes_.begin();
    // for (; it != lines_.end(); ++it)
    // {
    //     // if (it->first.d < 7 && std::abs(it_slope->first) > max_slope_)
    //     // {
    //     //    printf("\n\n\n\n\n\nslope : %f  %f\n", it_slope->first, it_slope->second);
    //     //    it->first.print();
    //     //    it->second.print();
    //     // }
    //     double slope = (it->second.z - it->first.z) / (it->second.d - it->first.d);
    //     if (std::abs(slope) >= max_slope_ && it->first.d < 10.0)
    //     {
    //         printf("\n\n\n\n\n\nslope %f\n", slope);
    //         it->first.print();
    //         printf("---->");
    //         it->second.print();
    //         printf("\n");
    //     }
    // }
    // if (isDebug)
    // {
    //     // fprintf(stderr, "(%f--->%f)\n", lines_.back().second.d - lines_.front().first.d);
    //     double length = lines_.back().second.d  - lines_.front().first.d;
    //     fprintf(stderr, "curr segment has %d lines\n", lines_.size());
    //     fprintf(stderr, "curr segment lines length: %f\n", length);

    //     for (auto it = lines_.begin(); it != lines_.end(); ++it)
    //     {
    //         fprintf(stderr, "(d: %f --> %f)\n", it->first.d, it->second.d);
    //         fprintf(stderr, "(z: %f --> %f)\n", it->first.z, it->second.z);
    //     }
    // }

}

void Segment::splitAndMerger(const int start_idx, const int end_idx)
{
    // STOP
    // std::cout << "start_idx = " << start_idx << " end_idx = " << end_idx << std::endl;
    // 收集这个区段的所有点
    int numPoint = 0;
    std::list<Bin::MinZPoint> current_partition_points;
    
    // 搜集所有点
    for (int idx = start_idx; idx < end_idx; ++idx)
    {
        if (bins_[idx].hasPoint())
        {
            numPoint++;
            // current_partition_points.push_back(bins_[idx].getMinZPoint());
        }
    }
    // 添加俩个端点
    // printf("\n\n\nnumPoint %ld\n", current_partition_points.size());
    if (numPoint <= 1) return;

    // 端点检测方法
    if (numPoint > 2)
    {
        for (int idx = start_idx; idx != end_idx; ++idx)
        {
            if (bins_[idx].hasPoint())
            {
                current_partition_points.emplace_back(bins_[idx].getMinZPoint());
                break;
            }
        }
        for (int idx = end_idx - 1; idx != start_idx - 1;--idx)
        {
            if (bins_[idx].hasPoint())
            {
                current_partition_points.emplace_back(bins_[idx].getMinZPoint());
                break;
            }
        }
    }

    // 直线拟合方法

    if (numPoint == 2)
    {
        // 俩个点的情况, 是不太可信的线段
        return;        
        // LocalLine local_line = endpointFit(current_partition_points);
        // Line new_line = localLineToLine(local_line, current_partition_points);
        // lines_.push_back(new_line);
    } 
    else
    {
        double dist_max = 0;
        double gap_max = 0; // 最大的间隙 可以映射后求
        int dist_max_idx = 1;
        Bin::MinZPoint firstPoint, secondPoint;
        firstPoint = current_partition_points.front();
        secondPoint = current_partition_points.back();

        LocalLine local_line = endpointFit(current_partition_points);
        // printf("start process fitLocalLine\n");
        // LocalLine local_line = fitLocalLine(current_partition_points);
        // printf("\nfitLocalLine fisished\n");
        // 找出距离它最远的点
        // std::list<Bin::MinZPoint>::iterator it = current_partition_points.begin();
        // it++;
        /*
        点到直线的距离
        */
        int gap_idx = -1;
        double tmp = sqrt(1 + local_line.first * local_line.first);
        int num_bin_gap = 3;
        bool gap_condition = false;
        for (int idx = start_idx; idx < end_idx; ++idx)
        {
            if (num_bin_gap <= 0 )
            {
                gap_condition = true;
                gap_idx = idx - 3;
                break;
            }
            if (!bins_[idx].hasPoint()) 
            {
                --num_bin_gap;
                continue;
            }
            Bin::MinZPoint currMinZPoint = bins_[idx].getMinZPoint();
            double dist = std::abs(local_line.first * (currMinZPoint).d - 
                    (currMinZPoint).z + local_line.second) / tmp;
            if (dist > dist_max)
            {
                dist_max = dist;
                dist_max_idx = idx;
            }
            num_bin_gap = 3;
        }
        // printf("dist_max : %f, dist_max_idx : %d\n", dist_max, dist_max_idx);
        if (!gap_condition && dist_max < min_split_dist)
        {
            // 添加成型的点
            lines_.push_back(localLineToLine(local_line, current_partition_points));
            linesSlopes_.emplace_back(local_line);
        }
        else
        {
            // if (start_idx == dist_max_idx)
            // {
            //     splitAndMerger(dist_max_idx + 1, end_idx);
            // }
            // else if (end_idx == dist_max_idx)
            // {
            //     splitAndMerger(start_idx, dist_max_idx - 1);
            // }
            // else
            // {
                // 如果是分段开始的点的误差最大， 那么可能导致死递归 比如 14-80， 14点为最大点
                // 会被分割为 14 - 14, 14 - 80 需要进行特殊处理
                // printf("star_idx, gap_idx, dist_max_idx, end_idx, %d %d %d %d\n", start_idx, gap_idx, dist_max_idx, end_idx);
                // dist_max_idx = (dist_max >= min_split_dist) ? dist_max_idx : gap_idx;
                if (gap_condition)
                {
                    splitAndMerger(start_idx, gap_idx);
                    splitAndMerger(gap_idx + 3, end_idx);
                }
                else
                {                
                    splitAndMerger(start_idx, dist_max_idx);
                    splitAndMerger(dist_max_idx, end_idx);
                }
            // }   
        }        
     
    }   

}

Segment::LocalLine Segment::endpointFit(const std::list<Bin::MinZPoint> & curr_line_points)
{
    // 理论上没有斜率为无穷大的
    LocalLine line_res;
    line_res.first = (curr_line_points.back().z - curr_line_points.front().z) /
                        (curr_line_points.back().d - curr_line_points.front().d);
    // b = y - A * x
    line_res.second = curr_line_points.front().z - line_res.first * curr_line_points.front().d;
    return line_res;
}

// 平均获取点的方式
int Segment::getPointBinIdx(const double & dist, const double & bin_step, double r_min)
{
    if (dist < r_min && dist > 120) return -1;
    int binIdx = (dist - r_min) / bin_step;
    return binIdx; 
}

// 根据角度获取点的方式
int Segment::getBinIdxFromDist(const double & d)
{
    int idxRes = 0;
    double angle_res = angle_resolution;
    if (d >= 20)
        angle_res /= 2;
    idxRes = (atan2(d, hSensor_) * 180 / M_PI - theta_start) / angle_res;
    assert(idxRes >= 0 && idxRes < bins_.size());
    return idxRes;
}

void Segment::filterLine()
{
    // lines_
    // linesSlopes_
    if (lines_.size() <= 1) return;
    std::list<Segment::Line>::iterator firstLineIter = lines_.begin();
    std::list<Segment::LocalLine>::iterator firstSlopeIter = linesSlopes_.begin();
    std::list<Segment::Line>::iterator secondLineIter = lines_.begin();
    ++secondLineIter;
    std::list<Segment::LocalLine>::iterator secondSlopeIter = linesSlopes_.begin();
    ++secondSlopeIter;


    for (;secondLineIter != lines_.end(); ++firstLineIter, ++secondLineIter, ++firstSlopeIter, ++secondSlopeIter)
    {
        // firstLineIter->second.print();
        // printf(" ---->  ");
        // secondLineIter->first.print();
        // printf("\n\n");
        // 距离太近不需要填充
        // printf("\nsecondLineIter->first.d - firstLineIter->second.d = %f", secondLineIter->first.d - firstLineIter->second.d);
        // const int binIdxFirst = getPointBinIdx(firstLineIter->second.d, bin_step);
        // const int binIdxSecond = getPointBinIdx(secondLineIter->first.d, bin_step);
        // printf("\n binIdxFirst %d, binIdxSecond %d ", binIdxFirst, binIdxSecond);
        // if (std::abs(binIdxFirst - binIdxSecond) >= 4) continue;
        // double expect_z = secondLineIter->first.d * firstSlopeIter->first + firstSlopeIter->second;
        // 预测的与实际的高度相差不大的情况下， 插入一个线段
        LocalLine vec1, vec2;
        vec1.first = firstLineIter->second.d - firstLineIter->first.d;
        vec1.second = firstLineIter->second.z - firstLineIter->first.z;

        vec2.first = secondLineIter->first.d - firstLineIter->second.d;
        vec2.second = secondLineIter->first.z - firstLineIter->second.z;

        double cosAngle = (vec1.first * vec2.first + vec1.second * vec2.second) / (
                sqrt(vec1.first * vec1.first + vec1.second * vec1.second) * 
                sqrt(vec2.first * vec2.first + vec2.second * vec2.second));
        // if (std::abs(slope - firstSlopeIter->first) < 0.1)
        // 夹角越大值越小
        // printf("cos theate %f (%f)\n", std::abs(cosAngle), cos(5.0 / 180 * M_PI));
        if (std::abs(cosAngle) > cos(5.0 / 180 * M_PI))
        {

        }
    }
}


double Segment::verticalDistanceToLine(const double &d, const double &z) 
{
    static const double kMargin = 0.1;
    double distance = -1;
    for (auto it = lines_.begin(); it != lines_.end(); ++it) 
    {
        if (it->first.d - kMargin < d && it->second.d + kMargin > d) 
        {
            const double delta_z = it->second.z - it->first.z;
            const double delta_d = it->second.d - it->first.d;
            const double expected_z = (d - it->first.d)/delta_d *delta_z + it->first.z;
            distance = std::fabs(z - expected_z);
        }
    }
    return distance;
}

void Segment::updateGroundLine()
{
    // printf("updateGroundLine() done\n");
    for (auto it = bins_.begin(); it != bins_.end(); ++it)
    {
        if (!it->hasPoint()) continue;
        // Bin::MinZPoint point_2d = 
        double dist = verticalDistanceToLine(it->getMinD(), it->getMinZ());
        if (dist < max_dist_to_line && dist != -1)
        {
            it->updateGround2();
        }        
    }
}

















