#include "component_clustering.h"


cluster::cluster():params_(params())
{
    std::atomic_init(&hasPoint, 0);
    numCluster = 0;
    numGrid = 200;  // 网格的数量
    roiM = 100;  // 100 miter 
    kernelSize = 3; // 也就是 8 连通

    grid = mapGrid(numGrid, std::vector<int>(numGrid, hasPoint));
}

cluster::cluster(const float & roiM,
            const int & numGrid,
            const int & numCluster,
            const int & kernelSize,
            PointCloud & input,
            const params & inputParam
            ):
        roiM(roiM),
        numGrid(numGrid),
        numCluster(numCluster),
        kernelSize(kernelSize),
        cloud(input),
        params_(inputParam)

{
    std::atomic_init(&hasPoint, 0);
    grid = mapGrid(numGrid, std::vector<int>(numGrid, hasPoint));

    double rad_horizontal = params_.start_angle_horizontal;
    for (int i = 0; i < params_.numCols; ++i)
    {
        _col_angles.emplace_back(rad_horizontal);
        rad_horizontal += params_.horizontal_step;
    }
}

void cluster::mapCloud2Grid()
{
    std::mutex cubeVecMutex;
    std::vector<std::thread> threads(params_.numThread);
    // std::cout << "numThread " << params_.numThread << "\n";
    int num_pre_thread = cloud.size() / params_.numThread; 

    // std::cout << "params_.n_threads " << params_.n_threads << std::endl;  
    for (int idx = 0; idx < params_.numThread - 1; ++idx)
    {
        // std::cout << "thread in for :" << idx << std::endl;
        int start_idx = idx * num_pre_thread;
        int end_idx = (idx + 1) * num_pre_thread - 1;
        threads[idx] = std::thread(&cluster::mapCloud2GridThread, 
                                    this, start_idx, end_idx, &cubeVecMutex);        
    }

    const size_t start_idx = num_pre_thread * (params_.numThread - 1);
    const size_t end_idx = cloud.size() - 1;
    threads[params_.numThread - 1] = std::thread(&cluster::mapCloud2GridThread, 
                                    this, start_idx, end_idx, &cubeVecMutex);

    // 等待所有的线程结束任务
    for (auto it = threads.begin(); it != threads.end(); ++it)
    {
        it->join();
    }

}

void cluster::mapCloud2GridThread(const unsigned int & start_idx, const unsigned int & end_idx,
                                    std::mutex * RectVecMutex)
{
    // printf("cloud size %ld\n", cloud.points.size());
    // printf("start idx %d, end_idx %d\n" , start_idx, end_idx);
    // printf("mapCloud2GridThread\n");
    // printf("numGrid %d, roiM %d\n", numGrid, roiM);
    for (int idx = start_idx; idx <= end_idx; ++idx)
    {
        point point(cloud[idx].x(), cloud[idx].y(), cloud[idx].z());
        double x = point.x();
        double y = point.y();
        
        double xC = x + roiM / 2;
        double yC = y + roiM / 2;
        // std::cout << "xC " << xC << " yC " << yC << '\n';
        if (xC < 0 || xC >= roiM || yC < 0 || yC >= roiM) 
            continue;

        // 添加 gird 的显示
        // RectVecMutex->lock();
        // cubeVec.emplace_back(std::make_pair(xC, yC));
        // RectVecMutex->unlock();

        // 有点的地方就显示一个矩形
        // 考虑是否将矩形也变成可选的 ID 选项
        int row = std::floor(xC / roiM * numGrid);
        int col = std::floor(yC / roiM * numGrid);

        // std::cout << "row " << row << " col " << col << '\n';
        // std::cout << "grid size " << grid[0].size() << " "<< grid.size() << "\n";
        grid[row][col] = -1; // 表示有点了
    }
}

// 插入要显示的矩形框框
void cluster::insertCubeVec()
{
    float size = roiM / numGrid;
    for (int row = 0; row < numGrid; ++row)
    {
        for (int col = 0; col < numGrid; ++col)
        {
            if (grid[row][col] == -1)
            {
                float xC = float(row) / numGrid * roiM - roiM / 2;
                float yC = float(col) / numGrid * roiM - roiM / 2;
                Rect2D rect(xC, yC, 0);
                rect.setSize(size);
                rect2DVec.emplace_back(rect);
            }
        }
    }
}

void cluster::componentClustering()
{    
    findComponent();    
}

void cluster::findComponent()
{
    mapCloud2Grid();
    // 便利每一个网络格子, 寻找每个有点的，并保存在 vec 中

    insertCubeVec();
    // fprintf(stderr, "num of non-empty voxels %d, total[%d]\n", cubeVec.size(), numGrid * numGrid);
    //
    for (int row = 0; row < numGrid; ++row)
    {
        for (int col = 0; col < numGrid; ++col)
        {
            if (grid[row][col] != -1)
                continue;
            numCluster++; // 每次搜索一次会搜索直到结束， 所以 ID 会加一
            search(row, col);            
        }
    }
}

void cluster::search(int Row, int Col)
{
    int mean = kernelSize / 2;
    for (int kI = -mean; kI <= mean; ++kI)
    {
        if (kI + Row < 0 || kI + Row >= numGrid) continue;
        for (int kJ = -mean; kJ <= mean; ++kJ)
        {
            if (kJ + Col < 0 || kJ + Col >= numGrid) continue;
            if (grid[kI + Row][kJ + Col] == -1)
            {
                grid[kI + Row][kJ + Col] = numCluster;
                search(kI + Row, kJ + Col);
            }
        }
    }
} 


void cluster::getClusterImg(cv::Mat & img)
{
    cv::Mat res = cv::Mat(numGrid, numGrid, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int idx = 0; idx < cloud.size(); ++idx)
    {
        point point(cloud[idx].x(), cloud[idx].y(), cloud[idx].z());
        double x = point.x();
        double y = point.y();
        double z = point.z();

        double xC = x + roiM / 2;
        double yC = y + roiM / 2;

        if (xC < 0 || xC >= roiM || yC < 0 || yC >= roiM)
            continue;
        
        int xI = floor(numGrid * xC / roiM);
        int yI = floor(numGrid * yC / roiM);

        int clusterNum = grid[xI][yI];
        if (clusterNum != 0)
        {
            res.at<cv::Vec3b>(xI, yI)[0] = (500 * clusterNum) % 255;
            res.at<cv::Vec3b>(xI, yI)[1] = (100 * clusterNum) % 255;
            res.at<cv::Vec3b>(xI, yI)[2] = (150 * clusterNum) % 255;
        }
    }

    img = res;
}

void cluster::makeClusteredCloud(PointCloud & clusterCloud, std::vector<Cloud::Ptr> & clusters)
{
    // 搜集聚类后的点云
    clusters.clear();
    std::vector<Cloud::Ptr> clustersTmp;
    clustersTmp.clear();

    for (int idx = 0; idx < numCluster + 1; ++idx)
    {
        Cloud::Ptr cloudPtr(new Cloud);
        clustersTmp.emplace_back(cloudPtr);
    }

    for (int idx = 0; idx < clusterCloud.size(); ++idx)
    {
        point point(clusterCloud[idx].x(), clusterCloud[idx].y(), clusterCloud[idx].z());
        float x = point.x();
        float y = point.y();
        float z = point.z();        
        // 计算点到雷达二维平面距离        
        point.toSensor2D = std::sqrt(x * x + y * y);        
        float angle_rad = std::atan2(y, x);
        // 避免重复计算， 时间换空间方法。
        point.atan2Val = angle_rad;
        float xC = x + roiM / 2;
        float yC = y + roiM / 2;

        if (xC < 0 || xC >= roiM || yC < 0 || yC >= roiM)
            continue;
        
        int xI = floor(numGrid * xC / roiM);
        int yI = floor(numGrid * yC / roiM);
        int clusterNum = grid[xI][yI];
        clusterCloud[idx].classID = clusterNum;
        // 添加点云
        // fprintf(stderr, "cluster size [%d]\n", clusters.size());
        // fprintf(stderr, "clusterNum [%d]\n", clusterNum);
        // assert(clusterNum <= clustersTmp.size());
        // fprintf(stderr, "size %d, current %d--------\n", clustersTmp.size(), clusterNum);
        clustersTmp[clusterNum]->emplace_back(point);
        // 获取最大最小值
        if (point.z() > clustersTmp[clusterNum]->maxZ)
            clustersTmp[clusterNum]->maxZ = point.z();
        if (point.z() < clustersTmp[clusterNum]->minZ)
            clustersTmp[clusterNum]->minZ = point.z();

        // fprintf(stderr, "makeClusteredCloud---------------------\n");
        if (angle_rad > clustersTmp[clusterNum]->maxAngle)
        {
            clustersTmp[clusterNum]->maxAngle = angle_rad;
            // 记录 L 俩端点所处点云位置的索引
            clustersTmp[clusterNum]->maxLPoint = clustersTmp[clusterNum]->size() - 1;
        }
        if (angle_rad < clustersTmp[clusterNum]->minAngle)
        {
            clustersTmp[clusterNum]->minAngle = angle_rad;
            clustersTmp[clusterNum]->minLPoint = clustersTmp[clusterNum]->size() - 1;
        }
        // fprintf(stderr, "-------------------------------------end\n");
    }

    // 排除点数较少的障碍物
    for (auto it = clustersTmp.begin(); it != clustersTmp.end(); ++it)
    {
        if ((*it)->size() > 5);
        {
            clusters.emplace_back((*it));
        }
    }

    // for (int idx = 0; idx < numCluster; ++idx)
    // {
    //     fprintf(stderr, "clusterID[%d] --> [%d]\n", idx, clusters[idx]->size());
    // }
}

size_t cluster::ColFromAngle(float angle_cols)
{
    size_t found = 0;
    // printf("angle_cols %f\n", angle_cols);

    found = std::upper_bound(_col_angles.begin(), _col_angles.end(), angle_cols) -
        _col_angles.begin();
    // printf("found %d\n", found);
    if (found == 0) return found;
    if (found == _col_angles.size()) return found - 1;

    double diff_next = std::abs(_col_angles[found] - angle_cols);
    double diff_prev = std::abs(angle_cols - _col_angles[found - 1]);
    return diff_next < diff_prev ? found : found - 1;
}

size_t cluster::ColFromAngle(float angle_cols, float minAngle, 
                             float maxAngle, 
                             const size_t & numSegment,
                             const bool & debug)
{
    // 每一份的步长
    float step = (maxAngle - minAngle) / numSegment;
    // 角度等分
    int Col = std::floor((angle_cols - minAngle) / step);
    // if (Col < 0 || Col >= numSegment)
    // {
    //     fprintf(stderr, "minAngle %f, maxAngle %f, step %f, Col %d\n", minAngle, maxAngle, step, Col);
    //     fprintf(stderr, "between %f, real dist %f\n", maxAngle - minAngle, angle_cols - minAngle);
    // }
    // return std::floor((angle_cols - minAngle) / step);
    // if (debug)
    // {
    //     fprintf(stderr, "info: step %f,  angle_cols %f,  minAngle %f , maxAngle %f\n", 
    //                 step, angle_cols, minAngle, maxAngle);
    // }
    return Col;
}

// 从聚类的点中 获取 L_shape 的边缘点
void cluster::getLShapePoints(const std::vector<Cloud::Ptr> & clusters, 
                                Cloud::Ptr & points,
                                const int & debugID,
                                const float & lShpaeHorizonResolution)
{ 
    // 除了边缘俩条线， 另外添加 79 条线 减去 1 得到 80 个区间
    // int numSampPoint = 79;  
    int iCluster = 0;
    // fprintf(stderr, "getLShapePoints------------------------\n");
    for (auto clusterIt = clusters.begin(); clusterIt != clusters.end(); ++clusterIt, ++iCluster)
    {
        // std::unordered_map<int, point> pointMap; // 为了可视化
        // std::unordered_map<int, int> colToPoint; // 为了修改 lshapePoint 点标记
        // Cloud pts;
        // 用来存储计算得来的 列索引
        std::vector<int> colVec;
        int ptIdx = 0;
        bool debugBool = false;
        if (iCluster == debugID)
            debugBool = true;
        auto & cluster = (*clusterIt);
        if (cluster->size() < 3)
            continue;
        // 是否需要修正跨越角度
        bool needFixed = false;
        float minAngle = cluster->minAngle;
        float maxAngle = cluster->maxAngle;        
        // 修正跨过 pi ----> pi 这个位置的点
        if (maxAngle - minAngle > M_PI)
        {
            float minAngleFixed = 999.0f;
            float maxAngleFixed = -999.0f;
            int minLPointIdx;
            int maxLPointIdx;
            int iPoint = 0;
            for (auto ptIt = cluster->begin(); ptIt != cluster->end(); ++ptIt)
            {
                // 相当于旋转 180 度角
                float angle;
                if (ptIt->atan2Val > 0.0f)
                    angle = ptIt->atan2Val - M_PI;
                else
                    angle = ptIt->atan2Val + M_PI;
                
                if (angle < minAngleFixed)
                {
                    minAngleFixed = angle;
                    cluster->minLPoint = iPoint;
                }
                if (angle > maxAngleFixed)
                {
                    maxAngleFixed = angle;
                    cluster->maxLPoint = iPoint;
                }
                ++iPoint;
            }
            minAngle = minAngleFixed;
            maxAngle = maxAngleFixed;
            needFixed = true;
        }
        
        // 逻辑上， 角度对应的点的数量
        int numSampPoint = (maxAngle - minAngle) / M_PI * 180 / lShpaeHorizonResolution;
        int numSampPointComp;
        // if (cluster->size() / 3 > numSampPoint)
        //     numSampPointComp = numSampPoint;
        // else
        //     numSampPointComp = std::floor(cluster->size() / 3);        
        numSampPointComp = numSampPoint;
        // colIdx 到 实际点云簇中的点
        std::vector<int> ToPointID(numSampPointComp + 1, -1);
        // 存储最小的距离
        std::vector<float> ToDist(numSampPointComp + 1, 999);
        
        // 点云数量太少的， 算法不合适
        // if (cluster->size() < 20)
        //     continue;
        
        // if (debugBool)
        //     fprintf(stderr, "cluster has numPoint: %d\n", cluster->size());
        for (auto ptIt = cluster->begin(); ptIt != cluster->end(); ++ptIt)
        {
            // float angle = std::atan2(ptIt->y(), ptIt->x());
            float angle = ptIt->atan2Val;
            if (needFixed)
            {
                if (angle > 0)
                {
                    angle -= M_PI;
                }
                else
                {
                    angle += M_PI;
                }                
            }
            // size_t colIdx = ColFromAngle(angle);
            // 角度跨过 0 度线时，会导致差值大于 180 度情况， 例如 minAngle -3.141381 , maxAngle 3.141573
            // 所以需要重新修正
            size_t colIdx;
            colIdx = ColFromAngle(angle, minAngle, maxAngle, numSampPointComp, debugBool);
            // if (debugBool)
            // {
            //     fprintf(stderr, "colIdx : %d\n", colIdx);
            // }
            // if (colIdx >= 20)
            //     fprintf(stderr, "colIdx %d, angle %f, minAngle %f, maxAngle %f\n", 
            //                 colIdx, angle, minAngle, maxAngle);
            assert(colIdx <= numSampPointComp); // 不能选取超过20个点的数目
            // 没有该索引， 就直接插入
            if (ptIt->toSensor2D < ToDist[colIdx])
            {
                ToDist[colIdx] = ptIt->toSensor2D;
                ToPointID[colIdx] = ptIdx;
            }
            ptIdx++;
        }

        // 判断该对象是否是对称对象
        // if (debugBool)
        // {
        // // int numSymPoint = 0;  // 对称点的对数
        // fprintf(stderr, "==================== size %d\n", ToDist.size());
        // 滤波
        // int kernelSize = 5;
        // int mean = (kernelSize - 1) / 2;
        // for (int idx = mean; idx < ToDist.size() - mean; ++idx)
        // {
        //     int meanPoint = 0;
        //     float sum = 0.0f;
        //     for (int i = -mean; i <= mean; ++i)
        //     {
        //         // 是否使用 continue 把边缘包含进去
        //         if (ToDist[idx + i] < 200)
        //         {
        //             sum += ToDist[idx + i];
        //             meanPoint++;
        //         }
        //     }
        //     ToDist[idx] = sum / meanPoint;
        // }

        int numPoints = ToDist.size();
        if (debugBool)
        {
            float minDistGlobal = *std::min_element(ToDist.begin(), ToDist.end());
            // 绘制*号， 相当于值方图
            for (int idx = 0; idx < numPoints/2; idx++)
            {
                float minDist = ToDist[2 * idx];
                for (int i = 1; i < 4;++i)
                {
                    if (minDist > ToDist[4 * idx + i])
                        minDist = ToDist[4 * idx + i]; 
                }
                int numStart = std::floor((minDist - minDistGlobal) * 100);
                if (minDist > 200)
                {
                    numStart = 0;
                }
                // else
                // {
                //     numStart = numStart % 10;
                // }              
                
                for (int iStart = 0; iStart < numStart; ++iStart)
                {
                    fprintf(stderr, "*");
                }
                fprintf(stderr, "\n");
            }
        }

        // 车宽在一个范围内， 使用进一步对称判断
        for (int idx = 0; idx < numPoints / 2; ++idx)
        {
            if (debugBool)
            {
                fprintf(stderr, "current idx : %d, %d\n", idx, numPoints - idx - 1);
                fprintf(stderr, "%f ", ToDist[idx]);
                fprintf(stderr, "%f ", ToDist[numPoints - idx - 1]);
            }
            float diff = ToDist[idx] - ToDist[numPoints - idx - 1];
            if (debugBool)
                fprintf(stderr, "  ----  %f ", diff);
            if (std::abs(diff) < 0.20)
            {
                // ++numSymPoint;
                cluster->numSymPoints++;
            }
            if (debugBool)
                fprintf(stderr, "\n");                
        }
        // fprintf(stderr, "number Symmetric Points %d\n", cluster->numSymPoints);
        // fprintf(stderr, "====================\n");
        // }

        // 为当前每个区域所选中的点赋予边缘点属性
        // 如果不对称则
        // if (cluster->numSymPoints >= 30)
        // {
        //     for (int idx = 25; idx < ToPointID.size() / 2; ++idx)
        //     {
        //         if (ToPointID[idx] == -1)
        //         {
        //             // fprintf(stderr, "idx %d\n", idx);
        //             continue;
        //         }
        //         points->emplace_back((*cluster)[ToPointID[idx]]);
        //         points->emplace_back((*cluster)[ToPointID[numPoints - idx - 1]]);
        //         (*cluster)[ToPointID[idx]].isLShapePoint = 1;
        //     }           
        // }
        // else
        // {
        //     for (int idx = 0; idx < ToPointID.size(); ++idx)
        //     {
        //         if (ToPointID[idx] == -1)
        //         {
        //             // fprintf(stderr, "idx %d\n", idx);
        //             continue;
        //         }
        //         points->emplace_back((*cluster)[ToPointID[idx]]);
        //         (*cluster)[ToPointID[idx]].isLShapePoint = 1;
        //     }
        // }
        // 不进行， 对称与否切割
        for (int idx = 0; idx < ToPointID.size(); ++idx)
        {
            if (ToPointID[idx] == -1)
            {
                // fprintf(stderr, "idx %d\n", idx);
                continue;
            }
            points->emplace_back((*cluster)[ToPointID[idx]]);
            (*cluster)[ToPointID[idx]].isLShapePoint = 1;
        }
        // 结束判断对称
            /*
            if (pointMap.find(colIdx) == pointMap.end())
            {
                // 标记为 边缘点
                pointMap.insert(std::make_pair(colIdx, *ptIt));
                colToPoint.insert(std::make_pair(colIdx, ptIdx));
            }
            else
            {
                // 如果距离小， 就跟新点
                // fprintf(stderr, "(%f, %f, %f) --> %f\n", 
                //         ptIt->x(), ptIt->y(), ptIt->z(), ptIt->toSensor2D);

                if (ptIt->toSensor2D < pointMap[colIdx].toSensor2D)
                {
                    point ptTmp = (*ptIt);
                    // ptTmp.z() = cluster->minZ;
                    pointMap[colIdx] = ptTmp;
                    colToPoint[colIdx] = ptIdx;
                }
            }  
            ++ptIdx;    
        }
        for (auto it = pointMap.begin(); it != pointMap.end(); ++it)
        {
            points->emplace_back(it->second);
            (*cluster)[colToPoint[it->first]].isLShapePoint = 1;
        }
        */
    }
}



/// depth_clustering
depth_clustering::depth_clustering(const PointCloud & cloud, bool filter, float angle_threshold):
        // _cloud(cloud),
        _filter(filter),
        _params(params())
{
    _angle_threshold = (angle_threshold / 180 * M_PI);
    _cloud = cloud;
    show_depth_image = cv::Mat(_params.numBeam, _params.numCols, CV_8UC3, cv::Scalar(0, 0, 0));
    fillVector();
    FillCosSin();
    _depthImage = cv::Mat::zeros(_params.numBeam, _params.numCols, cv::DataType<float>::type);
    sinColAlpha = sin(_params.horizontal_step);
    cosColAlpha = cos(_params.horizontal_step);
    sinRowAlpha = sin(_params.vehicle_step);
    cosRowAlpha = cos(_params.vehicle_step);  

    Neighborhood[0] = PixelCoord(-1, 0);
    Neighborhood[1] = PixelCoord(1, 0);
    Neighborhood[2] = PixelCoord(0, -1);
    Neighborhood[3] = PixelCoord(0, 1);   

    numCluster = 0; 
}


void depth_clustering::fillVector(/*std::vector<double> & input, const Direction & direction*/)
{
    float rad = _params.start_angle;
    for (int i = 0; i < _params.numBeam; ++i)
    {
        _row_angles.emplace_back(rad);
        rad += _params.vehicle_step;
    }

    // _row_angles.assign(_params.raw_angles.begin(), _params.raw_angles.end());
    double rad_horizontal = _params.start_angle_horizontal;
    for (int i = 0; i < _params.numCols; ++i)
    {
        _col_angles.emplace_back(rad_horizontal);
        rad_horizontal += _params.horizontal_step;
    }

    assert(_row_angles.size() == _params.numRows);
    assert(_col_angles.size() == _params.numCols);

    // fprintf(stderr, "numBeam[%d], numRows[%d], calculate[%d]\n", 
    //         _params.numBeam,
    //         _params.numRows,
    //         (int)((_params.end_angle - _params.start_angle) / 0.4)
    // );
    // fprintf(stderr, "start_angle[%f], end_angle[%f]\n", _params.start_angle, _params.end_angle);
}

void depth_clustering::FillCosSin()
{
    _row_angles_cosines.clear();
    _row_angles_sines.clear();

    for (const auto & angle : _row_angles)
    {
        _row_angles_sines.emplace_back(sin(angle));
        _row_angles_cosines.emplace_back(cos(angle));
    }

    // for (const auto & angle : _col_angles)
    // {

    // }
}

// 根据 角度， 查找 Row 索引
size_t depth_clustering::RowFromAngle(float angle_rows)
{
    size_t found = 0;
    found = std::upper_bound(_row_angles.begin(), _row_angles.end(), angle_rows) -
        _row_angles.begin();
    
    if (found == 0) return found;
    if (found == _row_angles.size()) return found - 1;

    double diff_next = std::abs(_row_angles[found] - angle_rows);
    double diff_prev = std::abs(angle_rows - _row_angles[found - 1]);

    return diff_next < diff_prev ? found : found - 1;
    
}

size_t depth_clustering::ColFromAngle(float angle_cols)
{
    size_t found = 0;
    // printf("angle_cols %f\n", angle_cols);

    found = std::upper_bound(_col_angles.begin(), _col_angles.end(), angle_cols) -
        _col_angles.begin();
    // printf("found %d\n", found);
    if (found == 0) return found;
    if (found == _col_angles.size()) return found - 1;

    double diff_next = std::abs(_col_angles[found] - angle_cols);
    double diff_prev = std::abs(angle_cols - _col_angles[found - 1]);
    return diff_next < diff_prev ? found : found - 1;
}

cv::Mat depth_clustering::CreateAngleImage(const cv::Mat & depth_image)
{
    cv::Mat angle_image = cv::Mat::zeros(depth_image.size(), cv::DataType<float>::type);
    cv::Mat x_mat = cv::Mat::zeros(depth_image.size(), cv::DataType<float>::type);
    cv::Mat y_mat = cv::Mat::zeros(depth_image.size(), cv::DataType<float>::type);

    const auto & sines_vec = _row_angles_sines;
    const auto & cosines_vec = _row_angles_cosines;

    // 环形图像
    float  dx, dy;
    x_mat.row(0) = depth_image.row(0) * cosines_vec[0];
    y_mat.row(0) = depth_image.row(0) * sines_vec[0];

    // 
    for (int r = 1; r < angle_image.rows; ++r)
    {
        x_mat.row(r) = depth_image.row(r) * cosines_vec[r];
        y_mat.row(r) = depth_image.row(r) * sines_vec[r];

        for ( int c = 0; c < angle_image.cols; ++c)
        {
            dx = fabs(x_mat.at<float>(r, c) - x_mat.at<float>(r - 1, c));
            dy = fabs(y_mat.at<float>(r, c) - y_mat.at<float>(r - 1, c));
            angle_image.at<float>(r, c) = atan2(dy, dx);
        }
    }
    return angle_image;
}

void depth_clustering::createDepthImage()
{
    for (int idx = 0; idx < _cloud.size(); ++idx)
    {
        point point(_cloud[idx].x(), _cloud[idx].y(), _cloud[idx].z());
        double dist_to_sensor = sqrt(point.x() * point.x() + point.y() * point.y() + point.z() * point.z());
        double angle_rows = asin(point.z() / dist_to_sensor);
        double angle_cols = atan2(point.y(), point.x());

        size_t bin_rows = RowFromAngle(angle_rows);
        size_t bin_cols = ColFromAngle(angle_cols);

        _depthImage.at<float>(bin_rows, bin_cols) = dist_to_sensor;
        // 存储每个点映射到的坐标
        pointToImageCoord.emplace_back(std::make_pair(bin_rows, bin_cols));

    }     
    
    if (_cloud.size() < 200)
    {
        for(int idx = 0; idx < _cloud.size(); ++idx)
        {
            fprintf(stderr, "point id(%d) ---> (%f, %f, %f)(%d, %d)[%d]\n", idx, 
                        _cloud[idx].x(), _cloud[idx].y(), _cloud[idx].z(),
                        pointToImageCoord[idx].first, pointToImageCoord[idx].second,
                        _cloud[idx].classID);
        }
    }
}

cv::Mat depth_clustering::getVisualizeDepthImage()
{
    cv::Mat res;
    for (int r = 0; r < _depthImage.rows; ++r)
    {
        for (int c = 0; c < _depthImage.cols; ++c)
        {
            float dist_to_sensor = _depthImage.at<float>(r, c);
            int value = mapToColor(dist_to_sensor);
            // 没有点映射的位置
            if (dist_to_sensor < 0.001f)
                continue;
            show_depth_image.at<cv::Vec3b>(r, c)[0] = value;
            show_depth_image.at<cv::Vec3b>(r, c)[1] = 255;
            show_depth_image.at<cv::Vec3b>(r, c)[2] = 255;
        }
    }
    cv::cvtColor(show_depth_image, res, cv::COLOR_HSV2BGR);
    cv::flip(res, res, -1);
    return res;
}

int depth_clustering::mapToColor(float val)
{
    // cv::Vec3b res;
    if (val <= _params.min_dist) val = _params.min_dist;
    if (val >= _params.max_dist) val = _params.max_dist;
    // int value = static_cast<int>((val - _params.min_dist) / _params.dist_length * 255);
    int value = static_cast<int>((val - _params.min_dist) / _params.dist_length * 180);
    // res = {value, value, value};
    return value;
    // printf("val %f   value %d\n", val, value);
}


cv::Mat depth_clustering::RepairDepth(const cv::Mat& no_ground_image, int step,
                                    float depth_threshold) 
{
    cv::Mat inpainted_depth = no_ground_image.clone();
    for (int c = 0; c < inpainted_depth.cols; ++c) 
    {
        for (int r = 0; r < inpainted_depth.rows; ++r) 
        {
            float& curr_depth = inpainted_depth.at<float>(r, c);
            // 说明这里没有点映射过来
            if (curr_depth < 0.001f) 
            {
                int counter = 0;
                float sum = 0.0f;
                // i 是走向后面的， j 是走向后面的， 并且 都在同一列上面，进行插值
                for (int i = 1; i < step; ++i) 
                {
                    if (r - i < 0) 
                    {
                        continue;
                    }
                    for (int j = 1; j < step; ++j) 
                    {
                        if (r + j > inpainted_depth.rows - 1) 
                        {
                            continue;
                        }
                        const float& prev = inpainted_depth.at<float>(r - i, c);
                        const float& next = inpainted_depth.at<float>(r + j, c);
                        if (prev > 0.001f && next > 0.001f &&
                            fabs(prev - next) < depth_threshold) 
                        {
                            sum += prev + next;
                            counter += 2;
                        }
                    }
                }
                if (counter > 0) 
                {
                    curr_depth = sum / counter;
                }
            }
        }
    }
    return inpainted_depth;
}

// 主函数
void depth_clustering::depthCluster()
{
    createDepthImage();
    _depthImage = RepairDepth(_depthImage, 5, 1.0f);
    if (_filter)
        _depthImage = ApplySavitskyGolaySmoothing(_depthImage, 5);   
    int numPixel = 0;
    // for(int row = 0; row < _depthImage.rows; ++row)
    // {
    //     for (int col = 0; col < _depthImage.cols; ++col)
    //     {
    //         if (_depthImage.at<float>(row, col) > 0.001f)
    //         {
    //             numPixel++;
    //         }
    //     }
    // }
    // fprintf(stderr, "has no empty pixel: %d\n", numPixel);
    // auto angle_image = CreateAngleImage(_depthImage);
    // _depthImage = ApplySavitskyGolaySmoothing(angle_image, 5); 
    // 为depthImage 打标签
    ComputeLabels();
    // 给点云打标签 classID
    // LabelCloud();
    fprintf(stderr, "cluster :%d\n", numCluster);
    // fprintf(stderr, "\n\n\n\n\n\n\n\n");
}

cv::Mat depth_clustering::ApplySavitskyGolaySmoothing(const cv::Mat & image, int window_size)
{
    cv::Mat kernel = GetSavitskyGolayKernel(window_size);
    cv::Mat smoothed_image;  // init an empty smoothed image
    cv::filter2D(image, smoothed_image, -1, kernel, cv::Point(-1, -1),
                0, cv::BORDER_REFLECT101);
    return smoothed_image;
}

cv::Mat depth_clustering::GetSavitskyGolayKernel(int window_size) const 
{
    if (window_size % 2 == 0) 
    {
        throw std::logic_error("only odd window size allowed");
    }
    bool window_size_ok = window_size == 5 || window_size == 7 ||
                        window_size == 9 || window_size == 11;
    if (!window_size_ok) {
    throw std::logic_error("bad window size");
    }
    // below are no magic constants. See Savitsky-golay filter.
    cv::Mat kernel;
    switch (window_size) {
    case 5:
        kernel = cv::Mat::zeros(window_size, 1, CV_32F);
        kernel.at<float>(0, 0) = -3.0f;
        kernel.at<float>(0, 1) = 12.0f;
        kernel.at<float>(0, 2) = 17.0f;
        kernel.at<float>(0, 3) = 12.0f;
        kernel.at<float>(0, 4) = -3.0f;
        kernel /= 35.0f;
        return kernel;
    case 7:
        kernel = cv::Mat::zeros(window_size, 1, CV_32F);
        kernel.at<float>(0, 0) = -2.0f;
        kernel.at<float>(0, 1) = 3.0f;
        kernel.at<float>(0, 2) = 6.0f;
        kernel.at<float>(0, 3) = 7.0f;
        kernel.at<float>(0, 4) = 6.0f;
        kernel.at<float>(0, 5) = 3.0f;
        kernel.at<float>(0, 6) = -2.0f;
        kernel /= 21.0f;
        return kernel;
    case 9:
        kernel = cv::Mat::zeros(window_size, 1, CV_32F);
        kernel.at<float>(0, 0) = -21.0f;
        kernel.at<float>(0, 1) = 14.0f;
        kernel.at<float>(0, 2) = 39.0f;
        kernel.at<float>(0, 3) = 54.0f;
        kernel.at<float>(0, 4) = 59.0f;
        kernel.at<float>(0, 5) = 54.0f;
        kernel.at<float>(0, 6) = 39.0f;
        kernel.at<float>(0, 7) = 14.0f;
        kernel.at<float>(0, 8) = -21.0f;
        kernel /= 231.0f;
        return kernel;
    case 11:
        kernel = cv::Mat::zeros(window_size, 1, CV_32F);
        kernel.at<float>(0, 0) = -36.0f;
        kernel.at<float>(0, 1) = 9.0f;
        kernel.at<float>(0, 2) = 44.0f;
        kernel.at<float>(0, 3) = 69.0f;
        kernel.at<float>(0, 4) = 84.0f;
        kernel.at<float>(0, 5) = 89.0f;
        kernel.at<float>(0, 6) = 84.0f;
        kernel.at<float>(0, 7) = 69.0f;
        kernel.at<float>(0, 8) = 44.0f;
        kernel.at<float>(0, 9) = 9.0f;
        kernel.at<float>(0, 10) = -36.0f;
        kernel /= 429.0f;
        return kernel;
    }
    return kernel;
}

cv::Mat depth_clustering::visualzieDiffAngleImage()
{
    cv::Mat colors = cv::Mat::zeros(_depthImage.rows, _depthImage.cols, CV_8UC3);

    float max_dist = 20.0f;
    for (int r = 0; r < _depthImage.rows; ++r) 
    {
        for (int c = 0; c < _depthImage.cols; ++c) 
        {
            if (_depthImage.at<float>(r, c) < 0.01f) 
            {
                continue;
            }
            uint8_t row_color = 255 * (_depthImage.at<float>(r, c) / max_dist);
            uint8_t col_color = 255 * (_depthImage.at<float>(r, c) / max_dist);
            cv::Vec3b color(255 - row_color, 255 - col_color, 0);
            colors.at<cv::Vec3b>(r, c) = color;
        }   
    }
    return colors;
}


// cv::Mat depth_clustering::ZeroOutGroundBFS(const cv::Mat & image,
//                              const cv::Mat & angle_image,
//                              const float & threshold,
//                              int kernel_size) const
// {
//     cv::Mat res = cv::Mat::zeros(image.size(), CV_32F);
//     cv::Mat image_labeler = cv::Mat::zeros(image.size(), cv::DataType<uint16_t>::type);

//     for (int c = 0; c < image.cols; ++c)
//     {
//         int r = image.rows - 1;
//         while (r > 0 && image.at<float>(r, c) < 0.001f)
//         {
//             --r;
//         }

//         PixelCoord current_coord = PixelCoord(r, c);
//         uint16_t current_label = image_labeler.at<uint16_t>(current_coord.row, current_coord.col);

//         if (current_label > 0)
//         {
//             // 已经标记过了，直接跳过
//             continue;
//         }

//         LabelOneComponent(1, current_coord, angle_image);
//     }

//     kernel_size = std::max(kernel_size - 2, 3);
//     cv::Mat kernel = GetUniformKernel(kernel_size, CV_8U);
//     cv::Mat dilated = cv::Mat::zeros(image_labeler.size(), image_labeler.type);
//     cv::dilate(image_labeler, dilated, kernel);

//     for (int r = 0; r <dilated.rows; ++r)
//     {
//         for (int c = 0; c < dilated.cols; ++c)
//         {
//             if (dilated.at<uint16_t>(r, c) == 0)
//             {
//                 res.at<float>(r, c) = image.at<float>(r, c);
//             }
//         }
//     }

//     return res;
// }

void depth_clustering::LabelOneComponent(uint16_t label, 
                                         const PixelCoord & start,
                                         const cv::Mat & angle_image)
{

}

cv::Mat depth_clustering::GetUniformKernel(int window_size, int type) const 
{
    if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
    }
    cv::Mat kernel = cv::Mat::zeros(window_size, 1, type);
    kernel.at<float>(0, 0) = 1;
    kernel.at<float>(window_size - 1, 0) = 1;
    kernel /= 2;
    return kernel;
}

void depth_clustering::ComputeLabels()
{
    _label_image = cv::Mat::zeros(_depthImage.size(), cv::DataType<uint16_t>::type);
    uint16_t label = 1;
    for (int row = 0; row < _label_image.rows; ++row)
    {
        for (int col = 0; col < _label_image.cols; ++col)
        {
            if (_depthImage.at<float>(row, col) < 0.001f)
            {
                // depth is zero, not interested
                continue;
            }
            // else
            // {
            //     fprintf(stderr, "[%d, %d] = %f\n", row, col, _depthImage.at<float>(row, col));
            // }            

            if (_label_image.at<uint16_t>(row, col) > 0)
            {
                // has labeled;
                continue;
            }

            LabelOneComponent(label, PixelCoord(row, col));
            label++;
        }
    }
    // 统计有多少对象
    numCluster = (--label);
}

void depth_clustering::LabelOneComponent(uint16_t label, const PixelCoord & start)
{
    // fprintf(stderr, "[info] NeighborHood: \n");
    // for (auto step : Neighborhood)
    // {
    //     fprintf(stderr, "(%d, %d)\n", step.row, step.col);
    // }

    // fprintf(stderr, "[info] _angle_threshold %f\n", _angle_threshold);
    // fprintf(stderr, "current label %d\n", label);
    std::queue<PixelCoord> labeling_queue;
    labeling_queue.push(start);
    size_t max_queue_size = 0;
    while (!labeling_queue.empty())
    {
        max_queue_size = std::max(labeling_queue.size(), max_queue_size);
        const PixelCoord current = labeling_queue.front();
        labeling_queue.pop();
        uint16_t current_label = _label_image.at<uint16_t>(current.row, current.col);
        if (current_label > 0)
        {
            continue;
        }
        
        // set label
        _label_image.at<uint16_t>(current.row, current.col) = label;
        auto current_depth = _depthImage.at<float>(current.row, current.col);
        if (current_depth < 0.001f)
        {
            // no point here
            continue;
        }

        for (const auto & step : Neighborhood)
        {
            PixelCoord neighbor = current + step;
            if (neighbor.row < 0 || neighbor.row >= _label_image.rows)
            {
                // point does not fit
                continue;
            }

            neighbor.col = WrapCols(neighbor.col);
            uint16_t neigh_label = _label_image.at<uint16_t>(neighbor.row,  neighbor.col);
            if (neigh_label > 0)
            {
                // we have already labeled this one
                continue;
            }          

            auto diff = DiffAt(current, neighbor);
            // fprintf(stderr, "current coord [%d, %d](%f) --> [%d, %d](%f)\n", 
            //         current.row, current.col,_depthImage.at<float>(current.row, current.col), 
            //         neighbor.row, neighbor.col, _depthImage.at<float>(neighbor.row, neighbor.col));
            // fprintf(stderr, "current diff :%f\n", diff);
            if (diff > _angle_threshold)
            {
                labeling_queue.push(neighbor);
            }
        }
    }
}

int16_t depth_clustering::WrapCols(int16_t col) const 
{
    // we allow our space to fold around cols
    if (col < 0) 
    {
      return col + _label_image.cols;
    }
    if (col >= _label_image.cols) 
    {
      return col - _label_image.cols;
    }
    return col;
}

float depth_clustering::DiffAt(const PixelCoord & from, const PixelCoord & to) const
{
    const float & current_depth = _depthImage.at<float>(from.row, from.col);
    const float & neighbor_depth = _depthImage.at<float>(to.row, to.col);
    // fprintf(stderr, "current_depth : %f\n", current_depth);
    // fprintf(stderr, "neighbor_depth : %f\n", neighbor_depth);

    float cosAlpha;
    float sinAlpha;
    if (from.row == 0 && to.row == _params.numRows)
    {
        sinAlpha = 0;
        cosAlpha = 1;
    }
    else if (from.row != to.row)
    {
        // fprintf(stderr, "row changed\n");
        sinAlpha = sinRowAlpha;
        cosAlpha = cosRowAlpha; 
        // if (from.row < to.row)
        // {
        //     sinAlpha = _row_angles_sines[from.row];
        //     cosAlpha = _row_angles_cosines[from.row];      
        // }
        // else
        // {
        //     sinAlpha = _row_angles_sines[to.row];
        //     cosAlpha = _row_angles_cosines[to.row];  
        // }
        
    }
    else
    {
        // fprintf(stderr, "col changed\n");
        sinAlpha = sinColAlpha;
        cosAlpha = cosColAlpha;
    }

    // fprintf(stderr, "sinAlpha : %f, cosAlpha : %f\n", sinAlpha, cosAlpha);
    float d1 = std::max(current_depth, neighbor_depth);
    float d2 = std::min(current_depth, neighbor_depth);

    float beta = std::atan2((d2 * sinAlpha), (d1 - d2 * cosAlpha));
    // fprintf(stderr, "beta : %f\n", beta);
    return fabs(beta);
}

cv::Mat depth_clustering::visSegmentImage()
{
    cv::Mat color_image(_label_image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int row = 0; row < _label_image.rows; ++row)
    {
        for (int col = 0; col < _label_image.cols; ++col)
        {
            auto label = _label_image.at<uint16_t>(row, col);
            auto random_color = _params.RANDOM_COLORS[label % _params.RANDOM_COLORS.size()];
            cv::Vec3b color = cv::Vec3b(random_color[0], random_color[1], random_color[2]);
            color_image.at<cv::Vec3b>(row, col) = color;
        }
    }

    cv::flip(color_image, color_image, -1);
    return color_image;
}

void depth_clustering::LabelCloud(Cloud & cloud)
{
    for (int idx = 0; idx < cloud.size(); ++idx)
    {
        // 该点的坐标， 向 _label_image 找索引
        auto cood = pointToImageCoord[idx];
        cloud[idx].classID = _label_image.at<uint16_t>(cood.first, cood.second);
    }

    if (cloud.size() < 200)
    {
        for(int idx = 0; idx < cloud.size(); ++idx)
        {
            fprintf(stderr, "point id(%d) ---> (%f, %f, %f)(%d, %d)[%d]\n", idx, 
                        cloud[idx].x(), cloud[idx].y(), cloud[idx].z(),
                        pointToImageCoord[idx].first, pointToImageCoord[idx].second,
                        cloud[idx].classID);
        }
    }
}


