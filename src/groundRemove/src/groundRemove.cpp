#include "groundRemove.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>



GroundSegmentation::GroundSegmentation(const GroundSegmentationParams & params):
                        params_(params),
                        segments_(params_.n_segments, Segment(params_.n_bins,
                                          params_.max_slope,
                                          params_.max_error_square,
                                          params_.long_threshold,
                                          params_.max_long_height,
                                          params_.max_start_height,
                                          params_.sensor_height,
                                          params_.r_max_bin,
                                          params_.r_min_bin,
                                          params_.tHmin,
                                          params_.tHmax,
                                          params_.tHDiff,
                                          params_.hSensor,
                                          params.min_split_dist,
                                          params_.theta_start,
                                          params_.theta_end,
                                          params_.angle_resolution,
                                          params_.min_split_dist))
                        // bin2PointIdx_(params_.n_segments, std::vector<int> (params_.n_bins, -1))
{
    // 在使用 atiomic 的时候，初始化全部在 : 后实现， 不再括号内部实现， 原因还不知道
    std::atomic_init(&count_, 0);
    // fprintf(stderr, "in GroundSegmentation params_.max_slope: %f\n", params_.max_slope);
    // fprintf(stderr, "in GroundSegmentation params.max_slope: %f\n", params.max_slope);
}

// 插入点云
void GroundSegmentation::insertPoints(const Cloud & cloud)
{
    // printParams();
    // printf("insertPoints use %d threads\n", params_.n_threads);
    std::vector<std::thread> threads(params_.n_threads);
    int num_pre_thread = cloud.size() / params_.n_threads; 

    // std::cout << "params_.n_threads " << params_.n_threads << std::endl;  
    for (int idx = 0; idx < params_.n_threads - 1; ++idx)
    {
        // std::cout << "thread in for :" << idx << std::endl;
        int start_idx = idx * num_pre_thread;
        int end_idx = (idx + 1) * num_pre_thread - 1;
        threads[idx] = std::thread(&GroundSegmentation::insertPointThread, 
                                    this, cloud, start_idx, end_idx);        
    }

    const size_t start_idx = num_pre_thread * (params_.n_threads - 1);
    const size_t end_idx = cloud.size() - 1;
    threads[params_.n_threads - 1] = std::thread(&GroundSegmentation::insertPointThread, 
                                    this, cloud, start_idx, end_idx);

    // 等待所有的线程结束任务
    for (auto it = threads.begin(); it != threads.end(); ++it)
    {
        it->join();
    }
    
}

// 注意添加点的时候， 点是不能均匀分布的， 所以需要最后一个线程多分一点点云
void GroundSegmentation::insertPointThread(const Cloud & cloud, 
                        const size_t start_idx, 
                        const size_t end_idx)
{
    // detaX = (r_max_bin - r_min_bin) / (n_bins - 1)
    // bin_idx = (range - r_min + r_min_bin + 2 * detaX) / (r_min_bin + detaX);
    // std::cout << "thread id is :" << std::this_thread::get_id() << std::endl;
    // double detaX = (params_.r_max_bin - params_.r_min_bin) / (params_.n_bins - 1);
    const double segment_step = 2 * M_PI / (params_.n_segments - 1e-6);
    // 减去 1e-6 是为了放置出现 0 和 n_segment 数出现， 使得数组访问越界
    // const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
    //          / params_.n_bins;
    // const double r_min = sqrt(params_.r_min_square);

    // double a = detaX / 2;
    // double b = (params_.r_min_bin - detaX / 2);

    // 处理每个线程的点
    for (size_t idx = start_idx; idx <= end_idx; ++idx)
    {
        // count_++;
        PointXYZ point(cloud[idx].x(), cloud[idx].y(), cloud[idx].z());
        const double range_square = point.x * point.x + point.y * point.y;
        const double range = sqrt(range_square);
        if (range_square < params_.r_max_square && range_square > params_.r_min_square)
        {
            const double angle = std::atan2(point.y , point.x);
            // 计算 binIdx
            // const unsigned int bin_idx = (range - r_min) / bin_step;

            const int bin_idx = getBinIdxFromDist(range);

            // double c = r_min - params_.r_min_bin - range;
            // unsigned int bin_idx = (-b + sqrt(b * b  - 4 * a * c)) / (2 * a);
            // if (bin_idx < 0 || bin_idx >= params_.n_bins)
            // {
            //     printf("d:%f, bin_idx %d\n", range, bin_idx);
            // }
            assert(bin_idx >= 0 && bin_idx < params_.n_bins);
            // unsigned int bin_idx = (range - r_min + params_.r_min_bin + 2 * detaX) /
            //                             (params_.r_min_bin + detaX);
            // if (bin_idx >= params_.n_bins)
            // {
            //     std::cout << "range : " << range << "\n";
            //     std::cout << "detaX : " << detaX << "\n";
            //     std::cout << "bin_idx : " << bin_idx << std::endl;
            //     bin_idx = params_.n_bins - 1;
            // }
            // fprintf(stderr, "\n\ncurrent idx %d\n", idx);
            // fprintf(stderr, "has %d point\n", cloud.size());
            // fprintf(stderr, "--------------------------------->>>>>\n");
            const int segment_idx = (angle + M_PI) / segment_step;
            bin_index_[idx] = std::make_pair(segment_idx, bin_idx);
            segments_[segment_idx == params_.n_segments? 0:segment_idx][bin_idx].addPoint(range, point.z, idx);
            // std::cout << "segment_idx " << segment_idx <<"\n" << "bin_idx " << bin_idx << "\n";
            // std::cout << "segments_[segment_idx][bin_idx].hasPoint() " <<
            //              segments_[segment_idx][bin_idx].hasPoint() << "\n";
            // bin2PointIdx_[segment_idx][bin_idx] = idx;
        }    
        else
        {
            bin_index_[idx] = std::make_pair(-1, -1); 
        }
        // 保存每个点的 range 和 point.z 避免重复计算， 空间换时间策略
        segment_coordinates_[idx] = Bin::MinZPoint(range, point.z);            
    }    
    
}

point GroundSegmentation::minZPointTo3d(const Bin::MinZPoint & mzpoint, const double & angle)
{
    point point;
    point.x() = cos(angle) * mzpoint.d;
    point.y() = sin(angle) * mzpoint.d;
    point.z() = mzpoint.z;
    return point;
}

point GroundSegmentation::minZPointTo3d(const double & d, const double & angle)
{
    point point;
    point.x() = cos(angle) * d;
    point.y() = sin(angle) * d;
    point.z() = 0;
    return point;
}

void GroundSegmentation::printParams()
{
    std::cout << "params_.line_search_angle : " << params_.line_search_angle << std::endl;
    std::cout << "params_.long_threshold : " << params_.long_threshold << std::endl;
    std::cout << "params_.max_dist_to_line : " << params_.max_dist_to_line << std::endl;
    std::cout << "params_.max_error_square : " << params_.max_error_square << std::endl;
    std::cout << "params_.max_long_height : " << params_.max_long_height << std::endl;
    std::cout << "params_.max_slope : " << params_.max_slope << std::endl;
    std::cout << "params_.max_start_height : " << params_.max_start_height << std::endl;
    std::cout << "params_.n_bins : " << params_.n_bins << std::endl;
    std::cout << "params_.n_segments : " << params_.n_segments << std::endl;
    std::cout << "params_.n_threads : " << params_.n_threads << std::endl;
    std::cout << "params_.r_max_square : " << params_.r_max_square << std::endl;
    std::cout << "params_.r_max_bin : " << params_.r_max_bin << std::endl;
    std::cout << "params_.r_min_bin : " << params_.r_min_bin<< std::endl;
    std::cout << "params_.sensor_height : " << params_.sensor_height << std::endl;
    std::cout << "params_.visualize : " << params_.visualize << std::endl;
}

// 获取所得到的线段， 为了后面可视化调试使用
void GroundSegmentation::getLines(std::list<PointLine> *lines)
{
    // bool visulize = lines;
    std::mutex line_mutex;
    // 多线程
    std::vector<std::thread> thread_vec(params_.n_threads);

    // std::chrono::high_resolution_clock::time_point start_fit = std::chrono::high_resolution_clock::now();
    for (int idx = 0; idx < params_.n_threads; ++idx)
    {
        const unsigned int start_idx = params_.n_segments / params_.n_threads * idx;
        const unsigned int end_idx = params_.n_segments / params_.n_threads * (idx + 1);
        thread_vec[idx] = std::thread(&GroundSegmentation::lineFitThread, this, 
                                                    start_idx, end_idx,lines, &line_mutex);
    }

    // 等待所有线程完成任务
    for(auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }

    // std::chrono::high_resolution_clock::time_point end_fit = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> fp_ms = end_fit - start_fit;
    // std::cout << "getLines took about " << fp_ms.count() << " ms" << std::endl;
    // std::cout << "getLines has " << lines->size() << " points\n";
}

// 线程分配 线段拟合
void GroundSegmentation::lineFitThread(const int & start_idx,
                       const int & end_idx,
                       std::list<PointLine> *lines,
                       std::mutex *lines_mutex)
{
    //
    const bool visualize = lines;
    const double seg_step = 2 * M_PI / params_.n_segments;
    // 可视化线段居于 Bin 的中间
    double angle = -M_PI + start_idx * seg_step;
    // double angle = -M_PI + start_idx * seg_step + seg_step / 2;
    for (int idx = start_idx; idx < end_idx; ++idx)
    {
        // 调试点击的位置
        if (idx == debugSegIdx)
        {
            segments_[idx].isDebug = true;
            for (auto it = segments_[idx].begin(); it != segments_[idx].end(); ++it)
            {
                // fprintf(stderr, "%d  ", it->hasPoint());
                if (it->hasPoint())   
                {
                    point tmp;
                    tmp = (*thisCloud)[it->pointID];
                    fprintf(stderr,"first seg point (%f, %f)\n", tmp.x(), tmp.y());   
                    break;       
                }
            }
        }
        //
        // segments_[idx].fitSegmentLines();   
        // printf("new segment \n\n\n\n");   
         
        segments_[idx].splitAndMerger(); 
        if (visualize)
        {            
            std::list<Segment::Line> segment_lines;
            segments_[idx].getLines(&segment_lines);
            // std::cout << "after segments_[idx].fitSegmentLines lines has  " << segment_lines.size() << " points\n";
            for (auto line_iter = segment_lines.begin(); line_iter != segment_lines.end(); ++line_iter)
            {
                point start = minZPointTo3d(line_iter->first, angle);
                point end = minZPointTo3d(line_iter->second, angle);
                lines_mutex->lock();  // 在添加元素的时候， 调用容器， 最好加上 lock 防止出错
                lines->emplace_back(start, end); // 后续可以绘制在 rviz 中使用
                lines_mutex->unlock();
            }
        }

        angle += seg_step;
    }
}

void GroundSegmentation::updateGround()
{
    std::vector<std::thread> thread_vec(params_.n_threads);
    size_t start_idx = 0, end_idx = 0;
    for (int idx = 0; idx < params_.n_threads; ++idx)
    {
        start_idx = params_.n_segments / params_.n_threads * idx;
        end_idx = params_.n_segments / params_.n_threads * (idx + 1);
        thread_vec[idx] = std::thread(&GroundSegmentation::updateGroundThread, this, start_idx, end_idx);
    }

    for (std::vector<std::thread>::iterator it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}

void GroundSegmentation::updateGroundThread(const size_t & start_idx, const size_t & end_idx)
{
    for (size_t idx = start_idx; idx < end_idx; ++idx)
    {
        segments_[idx].updateHeightAndGround();
    }
}

void GroundSegmentation::outlierFilter()
{
    std::vector<std::thread> thread_vec(params_.n_threads);
    size_t start_idx = 0, end_idx = 0;
    for (int idx = 0; idx < params_.n_threads; ++idx)
    {
        start_idx = params_.n_segments / params_.n_threads * idx;
        end_idx = params_.n_segments / params_.n_threads * (idx + 1);
        thread_vec[idx] = std::thread(&GroundSegmentation::outlierFilterThread, this, start_idx, end_idx);
    }

    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}

void GroundSegmentation::outlierFilterThread(const size_t & start_idx, const size_t & end_idx)
{
    for (size_t idx = start_idx; idx < end_idx; ++idx)
    {
        segments_[idx].outlierFilter();
    }
}

void GroundSegmentation::applayMedianFilter()
{
    for (int segIdx = 1; segIdx < params_.n_segments - 1; ++segIdx)
    {
        for (int binIdx = 1; binIdx < params_.n_bins - 1; ++binIdx)
        {
            // printf("%d, %d\n", segIdx, binIdx);
            // if (segments_[segIdx][binIdx + 1].isThisGround()&&
            //     segments_[segIdx][binIdx - 1].isThisGround() &&
            //     segments_[segIdx + 1][binIdx].isThisGround() &&
            //     segments_[segIdx - 1][binIdx].isThisGround())
            // {
                    std::vector<double> sur{segments_[segIdx][binIdx + 1].getHeight(),
                                            segments_[segIdx][binIdx + 1].getHeight(),
                                            segments_[segIdx + 1][binIdx].getHeight(),
                                            segments_[segIdx - 1][binIdx].getHeight()};
                    std::sort(sur.begin(), sur.end());
                    double m1 = sur[1];
                    double m2 = sur[2];
                    double median = (m1 + m2) / 2;
                    segments_[segIdx][binIdx].updateHeight(median);
                    // 将其设置为 地面点 跟新其高度， 如果条件成熟的话
                    segments_[segIdx][binIdx].updateGround();
            // }                
        }
    }
}

void GroundSegmentation::applayMedianFilterMinZ()
{
    for (int segIdx = 1; segIdx < params_.n_segments - 1; ++segIdx)
    {
        for (int binIdx = 1; binIdx < params_.n_bins - 1; ++binIdx)
        {
            // printf("%d, %d\n", segIdx, binIdx);
            if (segments_[segIdx][binIdx + 1].isThisGround()&&
                segments_[segIdx][binIdx - 1].isThisGround() &&
                segments_[segIdx + 1][binIdx].isThisGround() &&
                segments_[segIdx - 1][binIdx].isThisGround())
            {
                    std::vector<double> sur{segments_[segIdx][binIdx + 1].getMinZ(),
                                            segments_[segIdx][binIdx + 1].getMinZ(),
                                            segments_[segIdx + 1][binIdx].getMinZ(),
                                            segments_[segIdx - 1][binIdx].getMinZ()};
                    std::sort(sur.begin(), sur.end());
                    double m1 = sur[1];
                    double m2 = sur[2];
                    double median = (m1 + m2) / 2;
                    segments_[segIdx][binIdx].updateMinZ(median);
                    // 将其设置为 地面点 跟新其高度， 如果条件成熟
            }                
        }
    }
}


void GroundSegmentation::groundAndElevated(const Cloud & cloud, std::vector<int> * segmentation)
{

    int num_pre_thread = cloud.size() / params_.n_threads;
    std::vector<std::thread> thread_vec(params_.n_threads);
    // std::mutex mtx, mtx2;
    for (int idx = 0; idx < params_.n_threads - 1; ++idx)
    {
        int start_idx = idx * num_pre_thread;
        int end_idx = (idx + 1) * num_pre_thread;
        thread_vec[idx] = std::thread(&GroundSegmentation::groundAndElevatedThread, this, 
                cloud, segmentation, start_idx, end_idx);    
    }
    // printf("groundAndElevated\n");
    unsigned int start_idx = (params_.n_threads - 1) * num_pre_thread;
    unsigned int end_idx = cloud.points().size() - 1;
    thread_vec[params_.n_threads - 1] = std::thread(&GroundSegmentation::groundAndElevatedThread, this, 
                cloud, segmentation, start_idx, end_idx);
    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}

void GroundSegmentation::groundAndElevatedThread(const Cloud & cloud,
                                std::vector<int> *segmentation,
                                int start_idx,
                                int end_idx
                                    )
{
    // printf("groundAndElevatedThread\n");
    for (int idx = start_idx; idx < end_idx; ++idx)
    {
        // printf("Thread %d\n",idx);
        const int binIdx = bin_index_[idx].second;
        const int segIdx = bin_index_[idx].first;
        if (binIdx == -1) continue;
        // printf("binIdx : %d, segIdx : %d\n", binIdx, segIdx);
        // 当前所在的 bin 是否是地面点
        if (segments_[segIdx][binIdx].isThisGround())
        {
            double hGround = segments_[segIdx][binIdx].getHGround();
            if (cloud[idx].z() < (hGround + 0.25))
            {
                // 地面 1 
                // printf("isGround : segmentation")
                segmentation->at(idx) = 1;
            }
            else
            {
                segmentation->at(idx) = 0;
            }            
        }
    }
}
// mian segmentation function
void GroundSegmentation::segment(const Cloud & cloud, std::vector<int> * segmentation)
{
    // std::cout << "Segmentation cloud with " << cloud.points.size() << " Points." << std::endl;
    // 初始化
    segmentation->clear();
    int num_points = cloud.points().size();
    segmentation->resize(num_points, 0);
    segment_coordinates_.resize(num_points);
    bin_index_.resize(num_points);
    //
    hasClickedPoint = false;
    // 分配点云 insertPoint
    // fprintf(stderr, "%d points\n", cloud.size());
    thisCloud = std::make_shared<Cloud>(cloud);
    insertPoints(cloud);   
    // fprintf(stderr, "%insertPoints(cloud);");
    int groundRemoveID = 1;
    if (groundRemoveID == 0)
    {   
        // std::cout << "inserPoints finished \n" ;
        /*接下来是使用 很简单的梯度方法， 来自于
        3D-LIDAR Muliti Object Tracking for Autonomous Driving*/
        /////////////////////////////////////////////////////  去地开始 //////////////////////////    
        updateGround();
        // std::cout << "updateGround finished \n";
        applayMedianFilter();
        // std::cout << "applayMedianFilter finished \n";
        outlierFilter();
        // std::cout << "outlierFilter finished \n";
        groundAndElevated(cloud, segmentation);
        // std::cout << "groundAndElevated finished \n";
        std::list<PointLine> lines;
        /////////////////////////////////////////////////////  去地结束 //////////////////////////}
    }
    else
    {   
        // 获取线段 
        /////////////////////////////////////////////////// 使用 getline 的代码 ////////////////////
        /*时间超时了很多*/  
        // std::list<PointLine> lines;
        //     fprintf(stderr, "applayMedianFilter %d\n");
        //     applayMedianFilter();
        getLines(&lines);    
        // fprintf(stderr, "getLines(&lines);\n");
        // 可视化部分结果
        // 跟新 bin 是否是地面点， 根据是否在地面线段上
        updateBinGround();   
        // fprintf(stderr, "updateBinGround();\n");
        // 给点云分类
        // assignCluster(segmentation);
        // 第三种给点云分类的方法， 利用地面点的距离分类
        assignClusterByLine(segmentation);
        // fprintf(stderr, "assignClusterByLine(segmentation);\n");
   }
}

void GroundSegmentation::assignCluster(std::vector<int> * segmentation)
{
  std::vector<std::thread> thread_vec(params_.n_threads);
  const size_t cloud_size = segmentation->size();
  for (int i = 0; i < params_.n_threads; ++i) {
    const unsigned int start_index = cloud_size / params_.n_threads * i;
    const unsigned int end_index = cloud_size / params_.n_threads * (i+1);
    thread_vec[i] = std::thread(&GroundSegmentation::assignClusterThread, this,
                                start_index, end_index, segmentation);
  }
  for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();
  }

}

void GroundSegmentation::assignClusterThread(const unsigned int &start_index,
                                             const unsigned int &end_index,
                                             std::vector<int> *segmentation) 
{
    const double segment_step = 2 * M_PI / params_.n_segments;
    for (unsigned int i = start_index; i < end_index; ++i)
    {
        Bin::MinZPoint point_2d = segment_coordinates_[i];
        const int segment_index = bin_index_[i].first;
        if (segment_index >= 0) 
        {
            double dist = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);
            // Search neighboring segments.
            int steps = 1;
            // dist == -1 说明不再找到的 lines 的范围内
            // 如果此处没有线段， 就在其领域搜索，搜索的角步长， 和搜索的角度
            fprintf(stderr, "in assignClusterThread::line_search_angle %f\n", params_.line_search_angle);
            while (dist == -1 && steps * segment_step < params_.line_search_angle) 
            {
                // Fix indices that are out of bounds.
                int index_1 = segment_index + steps;
                while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;
                int index_2 = segment_index - steps;
                while (index_2 < 0) index_2 += params_.n_segments;
                // Get distance to neighboring lines.
                // 看相邻的俩个曲线， 判断其距离
                const double dist_1 = segments_[index_1].verticalDistanceToLine(point_2d.d, point_2d.z);
                const double dist_2 = segments_[index_2].verticalDistanceToLine(point_2d.d, point_2d.z);
                // Select larger distance if both segments return a valid distance.
                if (dist_1 > dist) {
                dist = dist_1;
                }
                if (dist_2 > dist) {
                dist = dist_2;
                }
                ++steps;
            }

            // 每个点找直线， 这样快很多， 找到了就不找了
            // 如果距离不是 -1, 且小于距离地面最近允许距离， 那么被分类为 1 ，也就是 1 代表地面

            if (dist < params_.max_dist_to_line && dist != -1) 
            {
                segmentation->at(i) = 1;
            }

        }
    }
}

int GroundSegmentation::getBinIdxFromDist(const double & d)
{
    int idxRes = 0;
    double angle_resolution = params_.angle_resolution;
    if (d >= 20)
        angle_resolution /= 2;

    idxRes = (atan2(d, params_.hSensor) * 180 / M_PI - params_.theta_start) / angle_resolution;
    // printf("idxRes %d\n", idxRes);
    // printf("d %f\n", d);
    // printf("hSensor %f\ntheta_start %f\nangle_resolution %f\n", 
    //             params_.hSensor, params_.theta_start, params_.angle_resolution);
    // assert(idxRes >= 0 && idxRes < params_.n_bins);
    if (idxRes <= 0)
        idxRes = 0;
    return idxRes;
}

void GroundSegmentation::updateBinGround()
{
    std::vector<std::thread> thread_vec(params_.n_threads);
    size_t start_idx = 0, end_idx = 0;
    for (int idx = 0; idx < params_.n_threads; ++idx)
    {
        start_idx = params_.n_segments / params_.n_threads * idx;
        end_idx = params_.n_segments / params_.n_threads * (idx + 1);
        thread_vec[idx] = std::thread(&GroundSegmentation::updateBinGroundThread, this, start_idx, end_idx);
    }

    for (std::vector<std::thread>::iterator it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}

void GroundSegmentation::updateBinGroundThread(const size_t & start_idx, const size_t & end_idx)
{
    for (unsigned int i = start_idx; i < end_idx; ++i)
    {
        segments_[i].updateGroundLine();
    }
}


void GroundSegmentation::assignClusterByLine(std::vector<int> * segmentation)
{
    // fprintf(stderr, "in assignClusterByLine::line_search_angle %f\n", params_.line_search_angle);
    // fprintf(stderr, "in assignClusterByLine::max_slope %f\n", params_.max_slope);
    std::vector<std::thread> thread_vec(params_.n_threads);
    const size_t cloud_size = segmentation->size();
    const int num_pre_thread = cloud_size / params_.n_threads; 
    for (int i = 0; i < params_.n_threads - 1; ++i) 
    {
        const int start_index = num_pre_thread * i;
        const int end_index = num_pre_thread * (i+1);
        thread_vec[i] = std::thread(&GroundSegmentation::assignClusterByLineThread, this,
                                start_index, end_index, segmentation);
    }

    const size_t start_idx = num_pre_thread * (params_.n_threads - 1);
    const size_t end_idx = cloud_size;
    thread_vec[params_.n_threads - 1] = std::thread(&GroundSegmentation::assignClusterByLineThread, 
                    this, start_idx, end_idx, segmentation);
    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();
    }

}

void GroundSegmentation::assignClusterByLineThread(const unsigned int &start_index,
                                             const unsigned int &end_index,
                                             std::vector<int> *segmentation) 
{ 
    // fprintf(stderr, "_params.n_segments: %d\n", params_.n_segments);
    const double segment_step = 2 * M_PI / params_.n_segments;
    for (unsigned int idx = start_index; idx < end_index; ++idx)
    {     
        // 是否调试   
        bool isDebug = false;

        Bin::MinZPoint point_2d = segment_coordinates_[idx];
        const int segment_idx = bin_index_[idx].first;
        const int bin_idx = bin_index_[idx].second;
        // 寻找 debug 的 Bin 的格子
        // if (segment_idx == debugSegIdx && bin_idx == debugBinIdx)
        // {
        //     fprintf(stderr, "segment_idx, bin_idx : (%d, %d)\n", segment_idx, bin_idx);
        //     isDebug = true;
        // }
        
        // if (std::find(selectObjectIDs.begin(), selectObjectIDs.end(), idx) != selectObjectIDs.end())
        // {
        //     isDebug = true;
        // }

        //
        
        if (segment_idx >= 0 && bin_idx >= 0) // 当前 bin 有点
        {
            // 当前点有点， 且是地面点， 且符合距离的情况就是地面点， 其他都是提升点            
            // segmentation->at(idx) = 1; 
            // 如果此处没有线段， 或者没有地面， 就找隔壁圆心
            // 如果本地找到了线段
            bool is_ground = false;
            bool between_one_and_two = false; // 距离在指定的允许距离的一倍到俩倍之间
            // fprintf(stderr, "segment_idx %d, bin_idx %d\n", segment_idx, bin_idx);
            if (segments_[segment_idx][bin_idx].isThisGround())
            {
                double dist = point_2d.z - segments_[segment_idx][bin_idx].getMinZ();
                if (dist < params_.max_dist_to_line)
                {
                    // if (isDebug)
                        // fprintf(stderr, "dist: %f\n", dist);    
                    segmentation->at(idx) = 1;
                    continue;
                }
                if (dist >= params_.max_dist_to_line && dist < 2 * params_.max_dist_to_line)
                {
                    between_one_and_two = true;
                }
            }
            
            
            // else
            
            if (!segments_[segment_idx][bin_idx].isThisGround() || between_one_and_two)  
            {
                // 不是地面点就去隔壁找找， 或者说大于一倍小于俩倍的距离点都去隔壁找找
                // if (true) //不去隔壁找
                // {
                    // 去几个隔壁找
                    int steps = 1;
                    bool find_ground = false;
                    while (!find_ground && steps * segment_step < params_.line_search_angle)
                    {
                        if (isDebug)
                        {
                            fprintf(stderr, "steps[%d] * segment_step[%f] = :%f < params_.line_search_angle[%f]", 
                                steps, segment_step, steps * segment_step ,params_.line_search_angle);
                        }
                        // Fix indices that are out of bounds.
                        int index_1 = segment_idx + steps;
                        while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;
                        int index_2 = segment_idx - steps;
                        while (index_2 < 0) index_2 += params_.n_segments;
                        find_ground = segments_[index_1][bin_idx].isThisGround();

                        if (isDebug)
                        {
                            fprintf(stderr, "find segment in neighbor [%d][%d]\n", index_1, bin_idx);
                            fprintf(stderr, "find_ground %d, is_ground %d\n", find_ground, is_ground);
                        }                           
                        
                        if (find_ground && !is_ground)
                        {
                            if (point_2d.z - segments_[index_1][bin_idx].getMinZ() < params_.max_dist_to_line)
                            {
                                is_ground = true;
                                if (isDebug)
                                    fprintf(stderr, "dist 640: %f\n", point_2d.z - segments_[index_1][bin_idx].getMinZ());
                                segmentation->at(idx) = 1;
                                // break;  // 找到了地面就不找了
                            }
                            break;  // 在邻居这边， 一旦发现可以参考的地面就不再
                                    // 继续找下去。 不管但前的是否是地面， 都以邻居的地面为标准
                        }

                        find_ground = segments_[index_2][bin_idx].isThisGround();
                        if (find_ground && !is_ground)
                        {
                            if (point_2d.z - segments_[index_2][bin_idx].getMinZ() < params_.max_dist_to_line)
                            {
                                is_ground = true;
                                if (isDebug)
                                    fprintf(stderr, "dist 653: %f\n", point_2d.z - segments_[index_2][bin_idx].getMinZ());
                                segmentation->at(idx) = 1;
                                // break; // 找到了就步找了
                            }
                            break; // 在邻居这边， 一旦发现可以参考的地面就不再继续找下去。
                        }  
                        ++steps;                  
                    }        
                // }
            }
            
        }
        
    }
    
}

void GroundSegmentation::getInsertedPoint(Cloud & cloud, Cloud & insertCloud)
{
    for (int seg_idx= 0; seg_idx < params_.n_segments; ++seg_idx)
    {
        for (int bin_idx = 0; bin_idx < params_.n_bins; ++bin_idx)
        {
            point pt;
            if (segments_[seg_idx][bin_idx].isThisGround())
            {
                auto cloudPt = cloud[segments_[seg_idx][bin_idx].pointID];
                pt.x() = cloudPt.x();
                pt.y() = cloudPt.y();
                pt.z() = cloudPt.z();
                insertCloud.emplace_back(pt);  
            }
        }
    }
}

void GroundSegmentation::getLinesPoint(Cloud & linesPoint)
{
    for (auto it = lines.begin(); it != lines.end(); ++it)
    {
        linesPoint.emplace_back(it->first);
        linesPoint.emplace_back(it->second);    
    }
}

void GroundSegmentation::setClickedPoint(double & x, double & y)
{
    hasClickedPoint = true;
    clickedPoint.x() = static_cast<float>(x);
    clickedPoint.y() = static_cast<float>(y);
    clickedPoint.z() = 0.0;

    const double segment_step = 2 * M_PI / params_.n_segments;
    const double angle = std::atan2(y, x);
    // const int segment_idx = (angle + M_PI) / segment_step;
    debugSegIdx = (angle + M_PI) / segment_step;
    double dist = std::sqrt(x * x + y *y);
    debugBinIdx = getBinIdxFromDist(dist);
    // for (int idx = 0; idx < params_.n_segments; ++idx)
    // {
    //     if (idx == segment_idx)
    //     {
    //         fprintf(stderr, "current choose angle %f\n", angle * 180 / M_PI);
    //         fprintf(stderr, "current choose idx %d\n", idx);

    //         segments_[idx].isDebug = true;

    //         for (auto it = segments_[idx].begin(); it != segments_[idx].end(); ++it)
    //         {
    //             fprintf(stderr, "%d  ", it->hasPoint());
    //             // if (it->hasPoint())   
    //             // {
    //             //     point tmp;
    //             //     tmp = (*thisCloud)[it->pointID];
    //             //     fprintf(stderr,"first seg point (%f, %f)\n", tmp.x(), tmp.y());   
    //             //     break;       
    //             // }
    //         }

    //     }
    // }
}

void GroundSegmentation::setSelectObjectID(std::vector<int> & selects)
{
    selectObjectIDs.assign(selects.begin(), selects.end());
}