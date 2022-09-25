#include <array>
#include <random>
#include <opencv2/opencv.hpp>
#include "box_fitting.h"

using namespace std;
using namespace cv;

// float picScale = 30;
float roiM = 120.0f;
float picScale = 900/roiM;
int ramPoints = 80;
float lSlopeDist = 1.0; // 大于 1.5 米的对角线车
// int lnumPoints = 200;
// 找到十二个 lShapePoint 就使用我们的算法
int lnumPoints = 12; 

float sensorHeight = 0.1;
// float tHeightMin = 1.2;
float tHeightMin = 0.8;
float tHeightMax = 2.6;
// float tWidthMin = 0.5;
// float tWidthMin = 0.4;
float tWidthMin = 0.25;
float tWidthMax = 5.5;
float tLenMin = 0.5;
float tLenMax = 30.0;
float tAreaMax = 80.0;
float tRatioMin = 1.2;
float tRatioMax = 5.0;
float minLenRatio = 3.0;
float tPtPerM3 = 8;


void getPointsInPcFrame(Point2f rectPoints[], vector<Point2f>& pcPoints, int offsetX, int offsetY)
{
    // loop 4 rect points
    for (int pointI = 0; pointI < 4; pointI++){
        float picX = rectPoints[pointI].x;
        float picY = rectPoints[pointI].y;
        // reverse offset
        float rOffsetX = picX - offsetX;
        float rOffsetY = picY - offsetY;
        // reverse from image coordinate to eucledian coordinate
        float rX = rOffsetX;
        float rY = picScale*roiM - rOffsetY;
        // reverse to 30mx30m scale
        float rmX = rX/picScale;
        float rmY = rY/picScale;
        // reverse from (0 < x,y < 30) to (-15 < x,y < 15)
        float pcX = rmX - roiM/2;
        float pcY = rmY - roiM/2;
        Point2f point(pcX, pcY);
        pcPoints[pointI] = point;
    }
}

bool ruleBasedFilter(vector<Point2f> pcPoints, float maxZ, int numPoints)
{
    bool isPromising = false;
    //minnimam points thresh
    if(numPoints < 8) return isPromising;
    // length is longest side of the rectangle while width is the shorter side.
    float width, length, height, area, ratio;
    float mass;

    float x1 = pcPoints[0].x;
    float y1 = pcPoints[0].y;
    float x2 = pcPoints[1].x;
    float y2 = pcPoints[1].y;
    float x3 = pcPoints[2].x;
    float y3 = pcPoints[2].y;

    float dist1 = sqrt((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2));
    float dist2 = sqrt((x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2));
    if(dist1 > dist2)
    {
        length = dist1;
        width = dist2;
    }
    else
    {
        length = dist2;
        width = dist1;
    }
    // assuming ground = sensor height
    // assuming right angle
    area = dist1*dist2;
    mass = area * maxZ;
    ratio = length / width;

    //start rule based filtering
#if 0 //zhanghm 20180607 修改
    if(height > tHeightMin && height < tHeightMax){
        if(width > tWidthMin && width < tWidthMax){
            if(length > tLenMin && length < tLenMax){
                if(area < tAreaMax){
                    if(numPoints > mass*tPtPerM3){
                        if(length > minLenRatio){
                            if(ratio > tRatioMin && ratio < tRatioMax){
                                isPromising = true;
                                return isPromising;
                            }
                        }
                        else{
                            isPromising = true;
                            return isPromising;
                        }
                    }
                }
            }
        }
    }
    else 
        return isPromising;
#endif

    // if(height > 0.1 && height < tHeightMax)
    // {
    if(width < tWidthMax && length < tLenMax && area < tAreaMax)
    {
        // if(ratio < 6.0f)
        // {
        //     isPromising = true;
        //     return isPromising;
        // }
        // else
        // {
        //     return false;
        // }
        return true;
    }
    // }
    else
        return isPromising;
}

void getBoundingBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr>& bbPoints)
{
    for (size_t iCluster = 0; iCluster < clusteredPoints.size(); iCluster++)
    {//遍历每个物体
      // Check lidar points number
      if(clusteredPoints[iCluster]->size() == 0)
        continue;

        Mat m (picScale*roiM, picScale*roiM, CV_8UC1, Scalar(0));
        float initPX = (*clusteredPoints[iCluster])[0].x() + roiM/2;
        float initPY = (*clusteredPoints[iCluster])[0].y() + roiM/2;
        int initX = floor(initPX*picScale);
        int initY = floor(initPY*picScale);
        int initPicX = initX;
        int initPicY = picScale*roiM - initY;//图像像素坐标
        int offsetInitX = roiM*picScale/2 - initPicX;
        int offsetInitY = roiM*picScale/2 - initPicY;

        int numPoints = (*clusteredPoints[iCluster]).size();
        vector<Point> pointVec(numPoints);
        vector<Point2f> pcPoints(4);
        float minMx, minMy, maxMx, maxMy;
        float minM = 999; float maxM = -999; float maxZ = -99;
        // for center of gravity
        float sumX = 0; float sumY = 0;
        for (size_t iPoint = 0; iPoint < (*clusteredPoints[iCluster]).size(); iPoint++)
        {
            //遍历某个点云簇中的所有点
            float pX = (*clusteredPoints[iCluster])[iPoint].x();
            float pY = (*clusteredPoints[iCluster])[iPoint].y();
            float pZ = (*clusteredPoints[iCluster])[iPoint].z();
            // cast (-15 < x,y < 15) into (0 < x,y < 30)
            float roiX = pX + roiM/2;
            float roiY = pY + roiM/2;
            // cast 30mx30m into 900x900 scale
            int x = floor(roiX*picScale);
            int y = floor(roiY*picScale);
            // cast into image coordinate
            int picX = x;
            int picY = picScale*roiM - y;
            // offset so that the object would be locate at the center
            int offsetX = picX + offsetInitX;
            int offsetY = picY + offsetInitY;
            m.at<uchar>(offsetY, offsetX) = 255;
            pointVec[iPoint] = Point(offsetX, offsetY);
            // calculate min and max slope
            float m = pY/pX;
            if(m < minM) 
            {
                minM = m;
                minMx = pX;
                minMy = pY;
            }
            if(m > maxM) 
            {
                maxM = m;
                maxMx = pX;
                maxMy = pY;
            }

            //get maxZ
            if(pZ > maxZ) maxZ = pZ;

            sumX += offsetX;
            sumY += offsetY; 

        }
        // L shape fitting parameters
        float xDist = maxMx - minMx;
        float yDist = maxMy - minMy;
        float slopeDist = sqrt(xDist*xDist + yDist*yDist);//最大最小斜率对应的两点之间的距离
        float slope = (maxMy - minMy)/(maxMx - minMx);

        // random variable
        mt19937_64 mt(0);
        uniform_int_distribution<> randPoints(0, numPoints-1);

        // start l shape fitting for car like object
        // lSlopeDist = 30, lnumPoints = 300
        if(slopeDist > lSlopeDist && numPoints > lnumPoints)
        {
            float maxDist = 0;
            float maxDx, maxDy;

            // 80 random points, get max distance
            for(int i = 0; i < ramPoints; i++)
            {
                size_t pInd = randPoints(mt);
                assert(pInd >= 0 && pInd < (*clusteredPoints[iCluster]).size());
                float xI = (*clusteredPoints[iCluster])[pInd].x();
                float yI = (*clusteredPoints[iCluster])[pInd].y();

                // from equation of distance between line and point
                float dist = abs(slope*xI-1*yI+maxMy-slope*maxMx)/sqrt(slope*slope + 1);
                if(dist > maxDist) 
                {
                    maxDist = dist;
                    maxDx = xI;
                    maxDy = yI;
                }
            }

            // for center of gravity
            // maxDx = sumX/clusteredPoints[iCluster].size();
            // maxDy = sumY/clusteredPoints[iCluster].size();

            // vector adding
            float maxMvecX = maxMx - maxDx;
            float maxMvecY = maxMy - maxDy;
            float minMvecX = minMx - maxDx;
            float minMvecY = minMy - maxDy;
            float lastX = maxDx + maxMvecX + minMvecX;
            float lastY = maxDy + maxMvecY + minMvecY;

            pcPoints[0] = Point2f(minMx, minMy);
            pcPoints[1] = Point2f(maxDx, maxDy);
            pcPoints[2] = Point2f(maxMx, maxMy);
            pcPoints[3] = Point2f(lastX, lastY);
            // bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            // if(!isPromising) 
            //     continue;
        }
        else
        {
            //MAR fitting
            RotatedRect rectInfo = minAreaRect(pointVec);
            Point2f rectPoints[4]; 
            rectInfo.points(rectPoints);
            // covert points back to lidar coordinate
            getPointsInPcFrame(rectPoints, pcPoints, offsetInitX, offsetInitY);
        }

        // make pcl cloud for 3d bounding box
        Cloud::Ptr oneBboxPtr(new Cloud);
        for(int pclH = 0; pclH < 2; pclH++)
        {//底面四个点,上面四个点
            for(int pclP = 0; pclP < 4; pclP++)
            {
                point o;
                o.x() = pcPoints[pclP].x;
                o.y() = pcPoints[pclP].y;
                if(pclH == 0) 
                    o.z() = -1.73;//车体坐标系下点云,地面高度估计为0.1m
                else 
                    o.z() = maxZ;
                oneBboxPtr->emplace_back(o);
            }
        }
        bbPoints.emplace_back(oneBboxPtr);
    }
}

void getBBox(const vector<Cloud::Ptr> & clusteredPoints,
             vector<Cloud::Ptr>& bbPoints,
             Cloud::Ptr & markPoints,
             Cloud::Ptr & lShapePoints,
             std::unordered_map<int, int> & bboxToCluster,
             const float & lShapeHorizonResolution,
             const int & debugID)
{
    // fprintf(stderr, "getBBox------------------------\n");
    int numCorrect = 0;
    // BBox 是从 bbPoints 中保留出来的， 所以根据有效的 BBox 的数目可以推出来使用的是那个 ID 跟踪的
    // fprintf(stderr, "current choose iCluster %d\n", debugID);
    if (debugID != -1)
        fprintf(stderr, "clusteredPoints.size %d, debugID %d\n", clusteredPoints.size(), debugID);
    std::array<string, 4> shapeName = {"SYMETRIC", "LSHAPE", "ISHAPE", "MINAREA"};
    
    // float lshapeResRad = 25 * lShapeHorizonResolution / 180 * M_PI;
    // float lshapeResRad = 12 * lShapeHorizonResolution / 180 * M_PI;
    float lshapeResRad = 12.5 * lShapeHorizonResolution / 180 * M_PI;
    int numShapePoints = 2 * M_PI / (lshapeResRad);
    lshapeResRad += 1e-6;
    // 存储当前 seg 距离最小距离
    // std::vector<float> shapeToDist(numShapePoints + 1, 999.0f);
    std::vector<float> shapeToDist(numShapePoints, 999.0f);
    // 存储当前 seg 所属 cluster 的 ID 如 22222244444333338888555
    // std::vector<int> shapeToClusterID(numShapePoints + 1, -1);
    // std::vector<float> shapeAngleRef(numShapePoints + 1, 0.0f);
    std::vector<int> shapeToClusterID(numShapePoints, -1);
    std::vector<float> shapeAngleRef(numShapePoints, 0.0f);
    for (size_t iCluster = 0; iCluster < clusteredPoints.size(); iCluster++)
    {   
        // fprintf(stderr, "---iCluster %d, total %d\n", iCluster, clusteredPoints.size());
        bool debugBool = false;
        if (iCluster == debugID)
            debugBool = true;
        //遍历每个物体
        // Check lidar points number
        // random variable
        auto & cluster = (*clusteredPoints[iCluster]);
        int numPoints = cluster.size();
        if (numPoints == 0)
            continue;
        mt19937_64 mt(0);
        uniform_int_distribution<> randPoints(0, numPoints-1);

        // vector<Point> pointVec(numPoints);
        vector<cv::Point2f> pointVec(numPoints);
        vector<Point2f> pcPoints(4);
        // 修正跨过 pi ----> pi 这个位置的点
        bool needFixed = false;
        float minAngle = cluster.minAngle;
        float maxAngle = cluster.maxAngle;    
        if (maxAngle - minAngle > M_PI)
        {
            float minAngleFixed = 999.0f;
            float maxAngleFixed = -999.0f;
            // int minLPointIdx;
            // int maxLPointIdx;
            int iPoint = 0;
            for (auto ptIt = cluster.begin(); ptIt != cluster.end(); ++ptIt)
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
                    cluster.minLPoint = iPoint;
                }
                if (angle > maxAngleFixed)
                {
                    maxAngleFixed = angle;
                    cluster.maxLPoint = iPoint;
                }
                ++iPoint;
            }
            cluster.minAngle = minAngleFixed;
            cluster.maxAngle = maxAngleFixed;

            // 后面添加, 交换一下 max min L point 的索引值
            // std::swap(cluster.minLPoint, cluster.maxLPoint);
            needFixed = true;
        }
        // 结束修正
        float minMx, minMy, maxMx, maxMy;
        // float minM = 999; float maxM = -999; float maxZ = -999;
        if (debugBool)
        {
            fprintf(stderr, "maxMx = cluster[cluster.maxLPoint].x();");
            fprintf(stderr, "cluster size %d, cluster .maxLPOint %d, cluster minLPOint %d\n", 
                        cluster.size(), cluster.maxLPoint, cluster.minLPoint);
        }
        maxMx = cluster[cluster.maxLPoint].x();
        minMx = cluster[cluster.minLPoint].x();
        maxMy = cluster[cluster.maxLPoint].y();
        minMy = cluster[cluster.minLPoint].y();       

        // L shape fitting parameters
        float xDist = maxMx - minMx;
        float yDist = maxMy - minMy;
        float slopeDist = sqrt(xDist * xDist + yDist * yDist);//最大最小斜率对应的两点之间的距离
        float slope = (maxMy - minMy)/(maxMx - minMx);

        float maxDist = 0;
        float maxDx, maxDy;
        bool findDPoint = false;

        // 80 random points, get max distance
        int sizePoint = 0;
        if (numPoints < ramPoints)
            sizePoint = numPoints;
        else
            sizePoint = ramPoints;
        
        // float distToOrigin = -slope * maxMx / sqrt(slope * slope + 1);            
        float interceptB = maxMy - slope * maxMx;
        float distToOrigin = interceptB / sqrt(slope * slope + 1);
        for(int i = 0; i < sizePoint; i++)
        {
            size_t pInd = randPoints(mt);
            // assert(pInd >= 0 && pInd < (*clusteredPoints[iCluster]).size());
            // float xI = (*clusteredPoints[iCluster])[pInd].x();
            // float yI = (*clusteredPoints[iCluster])[pInd].y();
            float xI = cluster[pInd].x();
            float yI = cluster[pInd].y();

            // from equation of distance between line and point
            float dist = (slope * xI - yI + interceptB) / sqrt(slope * slope + 1);
            float distAbs = std::abs(dist);

            if (debugBool)
            {
                fprintf(stderr, "dist: %f, distToOrigin: %f, slope: %f, b:%f\n",
                    dist, distToOrigin, slope, interceptB);
            }

            // 与原点不再同一侧， 不考虑其为 拐点
            if (dist * distToOrigin < 0.0f)
                continue;
            if(distAbs > maxDist) 
            {
                findDPoint = true;
                maxDist = distAbs;
                maxDx = xI;
                maxDy = yI;

                // 存储拐点坐标索引
                cluster.minOPoint = pInd;
            }
        }

        if (!findDPoint)
        {
            maxDx = (maxMx + minMx) / 2;
            maxDy = (maxMy + minMy) / 2;
        }
        // 找到拐点后， 直接切割 旋转 LShape 点云
        // 等分， 或者调整大侧边点云
        getLShapePoints(cluster, maxDx, maxDy, needFixed , debugBool, lShapeHorizonResolution);
        cluster.detectID = iCluster;
        //---------------------------------------------------

        Cloud clusterTmp;
        for (int idx = 0; idx < cluster.size(); ++idx)
        {
            if (cluster[idx].isLShapePoint != 0)
            {
                clusterTmp.emplace_back(cluster[idx]);
                lShapePoints->emplace_back(cluster[idx]);
            }
        }
        clusterTmp.detectID = iCluster;
        // 这里分配是否是检测完全的边缘点
        // fprintf(stderr, "setShapeOcclusionCheck before\n");
        // setShapeOcclusionCheck(shapeToDist, shapeToClusterID, shapeAngleRef,clusterTmp, lshapeResRad);
        setShapeOcclusionCheck(shapeToDist, shapeToClusterID, shapeAngleRef, cluster, lshapeResRad);
        // fprintf(stderr, "setShapeOcclusionCheck after\n");
        if (debugBool)
        {
            fprintf(stderr, "clusterTmp mean lshape Points size %d\n", clusterTmp.size());
        }

        // clusterTmp.minLPoint = (*clusteredPoints[iCluster]).minLPoint;
        // clusterTmp.maxLPoint = (*clusteredPoints[iCluster]).maxLPoint;
        // // int numPoints = (*clusteredPoints[iCluster]).size();
        
        // // 点数太少， minAreRect 不能调用
        // if (numPoints == 0)
        // {
        //     // 有被删除过的聚类
        //     continue;
        // }
        // // vector<Point> pointVec(numPoints);
        // vector<cv::Point2f> pointVec(numPoints);
        // vector<Point2f> pcPoints(4);
        // float minMx, minMy, maxMx, maxMy;
        // float minM = 999; float maxM = -999; float maxZ = -999;
        // maxMx = (*clusteredPoints[iCluster])[(*clusteredPoints[iCluster]).maxLPoint].x();
        // minMx = (*clusteredPoints[iCluster])[(*clusteredPoints[iCluster]).minLPoint].x();
        // maxMy = (*clusteredPoints[iCluster])[(*clusteredPoints[iCluster]).maxLPoint].y();
        // minMy = (*clusteredPoints[iCluster])[(*clusteredPoints[iCluster]).minLPoint].y();
            // if (debugBool)
            //     fprintf(stderr, "222->(maxMx, maxMy), (minMx, minMy), (%f, %f), (%f, %f)\n", maxMx, maxMy, minMx, minMy);
        // }
        
        // L shape fitting parameters
        // float xDist = maxMx - minMx;
        // float yDist = maxMy - minMy;
        // float slopeDist = sqrt(xDist * xDist + yDist * yDist);//最大最小斜率对应的两点之间的距离
        // float slope = (maxMy - minMy)/(maxMx - minMx);

        

        // start l shape fitting for car like object
        // lSlopeDist = 30, lnumPoints = 300
        // int sizePoint;
        // fprintf(stderr, "slopeDist : %f, numPoints : %d\n", slopeDist, numPoints);
        bool hasRect = false;
        // 对称， 且不是侧边车辆的对称结构
        // fprintf(stderr, "has gone ----\n");
        if (debugBool)
        {
            fprintf(stderr, "slopeDist %f, numPoints %d\n", slopeDist, numPoints);
        }
        // 车宽 2.5m ？？？
        // if (debugBool)
        // {
        //     fprintf(stderr, "number of percent SymPoints %f, numSymPoints %d, numNoneEmptyLShapePoint %d\n", 
        //         cluster.numSymPoints / (cluster.numNoneEmptyLShapePoint + 1e-3),
        //         cluster.numSymPoints, 
        //         cluster.numNoneEmptyLShapePoint);
        // }
        // if (cluster.numSymPoints >= 30 && slopeDist < 2.5f && numPoints > lnumPoints)
        if (cluster.isSymCloud && slopeDist < 2.5f && numPoints > lnumPoints)
        {
            // fprintf(stderr, "has gone404 ----\n");
            float rectK;
            // if (debugBool)
            // {
            //     fprintf(stderr, "using direction, num %d Symmetric points\n", 
            //             cluster.numSymPoints);
            // }
            // 对称情况， 直接对全部点云使用 pac 方法  失败了
            // rectK = direction(*clusteredPoints[iCluster]);
            // rectK = direction(clusterTmp);  

            // 计算对称点云中心
            float centerX, centerY;
            float sumX = 0.0f, sumY = 0.0f;
            auto & pointsOrig = *clusteredPoints[iCluster];
            for (int idx = 0; idx < pointsOrig.size(); ++idx)
            {
                sumX += pointsOrig[idx].x();
                sumY += pointsOrig[idx].y();
            }
            centerX = sumX / pointsOrig.size();
            centerY = sumY / pointsOrig.size();
            markPoints->emplace_back(point(centerX, centerY, 0.0f, pointType::STANDARD));
            // 对点云对象排序， 根据距离， 从近到远
            // fprintf(stderr, "clusterTmp Size %d\n", clusterTmp.size());
            // fprintf(stderr, "------\n");
            // clusterTmp.sort();
            // std::vector<Point3f> frontKelemPoints(clusterTmp.size());
            // for (int idx = 0; idx < clusterTmp.size(); ++idx)
            // {
            //     frontKelemPoints[idx].x = clusterTmp[idx].x();
            //     frontKelemPoints[idx].y = clusterTmp[idx].y();
            //     frontKelemPoints[idx].z = clusterTmp[idx].toSensor2D;
            // } 
            // std::sort(frontKelemPoints.begin(), frontKelemPoints.end(), 
            //         [](const Point3f & a, const Point3f & b){return a.z < b.z;});
            // // 取对象的前百分之 10 的点求平均值
            // int numFrontPoints = std::floor(clusterTmp.size() / 2);
            
            // // fprintf(stderr, "numFrontPoints ");
            // std::vector<cv::Point2f> points;
            // Point2f frontMiddlePoint;
            // float sumFrontX = 0.0f, sumFrontY = 0.0f;
            // for (int idx = 0; idx < numFrontPoints; ++idx)
            // {
            //     sumFrontX += frontKelemPoints[idx].x;
            //     sumFrontY += frontKelemPoints[idx].y;
            //     points.emplace_back(Point2f(frontKelemPoints[idx].x, frontKelemPoints[idx].y));
            // }
            // frontMiddlePoint.x = sumFrontX / numFrontPoints;
            // frontMiddlePoint.y = sumFrontY / numFrontPoints;
            // markPoints->emplace_back(point(frontMiddlePoint.x, frontMiddlePoint.y, 0.0f));
            // float correctFitPercent = 0.0f;
            // // fprintf(stderr, "has gone1 !\n");
            // for (int idx = 0; idx < clusterTmp.size(); ++idx)
            // {
            //     points.emplace_back(Point2f(frontKelemPoints[idx].x, frontKelemPoints[idx].y));
            // }
            // rectK = fitLineRansac(points, 30, 0.045, correctFitPercent, debugBool);        
            // if (correctFitPercent < 0.8)
            //     rectK = (centerY - frontMiddlePoint.y) / (centerX - frontMiddlePoint.x + 1e-3);

            // 直接使用激光雷达中心到中心点的连线
            rectK = (centerY) / (centerX + 1e-6);
            // rectK = fitLine(points);
            fitRect(rectK, cluster,  pcPoints);                   
            if (debugBool)
                fprintf(stderr, "\ncurrent rectK : %f\n", rectK);
            hasRect = true;
            // 判断点云轮廓类型
            cluster.shape = shapeType::SYMETRIC;
            
            // fprintf(stderr, "has gone12 finish !\n");
        }
        if(!hasRect && slopeDist > lSlopeDist && numPoints > lnumPoints)
        {
            // fprintf(stderr, "has gone3 !\n");
            // float maxDist = 0;
            // float maxDx, maxDy;
            // bool findDPoint = false;

            // // 80 random points, get max distance
            // if (numPoints < ramPoints)
            //     sizePoint = numPoints;
            // else
            //     sizePoint = ramPoints;
            
            // // float distToOrigin = -slope * maxMx / sqrt(slope * slope + 1);            
            // float interceptB = maxMy - slope * maxMx;
            // float distToOrigin = interceptB / sqrt(slope * slope + 1);
            // for(int i = 0; i < sizePoint; i++)
            // {
            //     size_t pInd = randPoints(mt);
            //     // assert(pInd >= 0 && pInd < (*clusteredPoints[iCluster]).size());
            //     // float xI = (*clusteredPoints[iCluster])[pInd].x();
            //     // float yI = (*clusteredPoints[iCluster])[pInd].y();
            //     float xI = clusterTmp[pInd].x();
            //     float yI = clusterTmp[pInd].y();

            //     // from equation of distance between line and point
            //     float dist = (slope * xI - yI + interceptB) / sqrt(slope * slope + 1);
            //     float distAbs = std::abs(dist);

            //     if (debugBool)
            //     {
            //         fprintf(stderr, "dist: %f, distToOrigin: %f, slope: %f, b:%f\n",
            //             dist, distToOrigin, slope, interceptB);
            //     }

            //     // 与原点不再同一侧， 不考虑其为 拐点
            //     if (dist * distToOrigin < 0.0f)
            //         continue;
            //     if(distAbs > maxDist) 
            //     {
            //         findDPoint = true;
            //         maxDist = distAbs;
            //         maxDx = xI;
            //         maxDy = yI;
            //     }
            // }

            // if (!findDPoint)
            // {
            //     maxDx = (maxMx + minMx) / 2;
            //     maxDy = (maxMy + minMy) / 2;
            // }

            // for center of gravity
            // maxDx = sumX/clusteredPoints[iCluster].size();
            // maxDy = sumY/clusteredPoints[iCluster].size();

            // vector adding
            float maxMvecX = maxMx - maxDx;
            float maxMvecY = maxMy - maxDy;
            float minMvecX = minMx - maxDx;
            float minMvecY = minMy - maxDy;
            // // 夹角来个边的
            float theatAOBVal = maxMvecX * minMvecX + maxMvecY * minMvecY;
            // float lastX = maxDx + maxMvecX + minMvecX;
            // float lastY = maxDy + maxMvecY + minMvecY;

            // 添加三个可视化点
            markPoints->emplace_back(cluster[cluster.maxLPoint]);
            markPoints->emplace_back(cluster[cluster.minLPoint]);
            markPoints->emplace_back(point(maxDx, maxDy, 0.0f));
            // markPoints->emplace_back(point(maxMx, maxMy, 0.0f));
            // markPoints->emplace_back(point(minMx, minMy, 0.0f));

            // 求 垂线 俩边的点集合
            // float k = (maxMvecY - minMvecY) / (maxMvecX - minMvecX + 1e-6);
            // 这里出现过一个重大的 bug 求斜率时候， 写错了
            // float k_1 = -1 / (k_1 + 1e-6);
            // 正确的写法
            // float k_1 = -1 / (k + 1e-6);
            
            // Vertex vet(k_1, maxDy - k_1 * maxDx);
            // 存储较长边点云集合
            // vector<point> ptSet;
            vector<Point2f> ptSet;//, ptSet2;
            float distA = std::sqrt((maxMy- maxDy) * (maxMy - maxDy) + (maxMx - maxDx) * (maxMx - maxDx));
            float distB = std::sqrt((minMy- maxDy) * (minMy - maxDy) + (minMx - maxDx) * (minMx - maxDx));
            
            // 所成的夹角的值
            float cosThetaAOB = theatAOBVal / (distB * distA);
            // 旋转 30 度角分割 ------------------
            // 第一步 长边到短边的叉成 （x1, y1） x (x2, y2) = (x1 * y2 - x2 * y1) 正， 顺时针， 负， 逆时针
            // 利用 tan (A + B) = (tan A + tan B) / (1 - tan A * tan B)
            // 替换掉新的 斜率
            // a 是长边
            float crossProduct;
            float Xa, Ya, Xb, Yb;
            if (distA < distB)
            {
                
                Xa = minMx - maxDx;
                Ya = minMy - maxDy;

                Xb = maxMx - maxDx;
                Yb = maxMy - maxDy;
            }
            else
            {
                Xa = maxMx - maxDx;
                Ya = maxMy - maxDy;

                Xb = minMx - maxDx;
                Yb = minMy - maxDy;
            }
            
            crossProduct = Xa * Yb - Xb * Ya;
            // 逆时针旋转 加角度
            float k_curr  = Ya / (Xa + 1e-6);
            float k_1; // 旋转 30 度之后的斜率
            // 角度大于 130 接近直线， 选取全部点
            bool moreThanTheta = false;
            if (cosThetaAOB < std::cos(130.0f / 180 * M_PI))
            {
                if (debugBool)
                    fprintf(stderr, "cosTheatAOB %f度, cos(150) : %f, cosThetaAOB %f\n", 
                                std::acos(cosThetaAOB) / M_PI * 180, 
                                std::cos(150.0f / 180 * M_PI),
                                cosThetaAOB);
                k_1 = Yb / (Xb + 1e-6);
                moreThanTheta = true;
            }
            else
            {
                if (crossProduct > 0)
                {
                    k_1 = (0.2679 + k_curr) / (1 - k_curr * 0.2679);
                }
                else
                {
                    // 顺时针旋转， 减去一个角度
                    k_1 = (-0.2679 + k_curr) / (1 - k_curr * (-0.2679));
                }
            }
            
            // 旋转之后的直线
            Vertex vet(k_1, maxDy - k_1 * maxDx);
            if (debugBool)
            {
                fprintf(stderr, "k_curr %f, k_1 %f, crossProduct %f\n", k_curr, k_1, crossProduct);
                fprintf(stderr, "tan(15) :%f,  tan(30)%f\n", tanf(15.0f / 180 * M_PI), tanf(60.0f / 180 * M_PI));
            }
            if (debugBool)
            {
                for (int idx = 1; idx < 30; ++idx)
                {
                    markPoints->emplace_back(point(maxDx + 0.04 * idx, maxDy + 0.04 * idx * k_1, -1.7f));
                }
            }
            // 判断长边            
            //----------------------------------
            // 垂直线段

            // fprintf(stderr, "dist A -- > B(%f, %f)\n", distA, distB);            
            // float zero
            float lessZero;
            float longEdge;
            if (distA < distB)
            {
                lessZero = k_1 * minMx + vet.y - minMy; 
                markPoints->emplace_back(point((minMx + maxDx) / 2, (minMy + maxDy) / 2, cluster[cluster.minLPoint].z() / 2));
                longEdge = distB;
            }            
            else
            {
                lessZero = k_1 * maxMx + vet.y - maxMy;
                markPoints->emplace_back(point((maxMx + maxDx) / 2, (maxMy + maxDy) / 2, cluster[cluster.maxLPoint].z() / 2));
                longEdge = distA;
            }

            // fprintf(stderr, "k : %f\n", k);  
            // 相乘大于零， 说明处于同一侧， 否则不同侧
            // 搜集处于同一侧的点
            for (int i = 0; i < clusterTmp.size(); ++i)
            {
                float dist = k_1 * clusterTmp[i].x() + vet.y - clusterTmp[i].y();
                if (moreThanTheta)
                {
                    if (debugBool)
                        markPoints->emplace_back(clusterTmp[i]);
                    ptSet.emplace_back(Point2f(clusterTmp[i].x(), clusterTmp[i].y()));
                }
                else if (dist * lessZero >= 0)
                {
                    // 是否需要提出长边的部分点， 例如头部点
                    // if (debugBool)
                    //     fprintf(stderr, "K_1: %f,  dist : %f,  lessZero :%f \n", k_1, dist, lessZero);
                    // ptSet.emplace_back(clusterTmp[i]);
                    if (debugBool)
                        markPoints->emplace_back(clusterTmp[i]);
                    ptSet.emplace_back(Point2f(clusterTmp[i].x(), clusterTmp[i].y()));
                }              

                
                // else
                // {
                //     ptSet2.emplace_back(Point2f(clusterTmp[i].x(), clusterTmp[i].y()));
                // }
            }

            // 如果这条边的点数太少占据总 lShapePoint 点数的百分之 30 不到， 那么就换一条边

            // fprintf(stderr, "pointSetSize : %d\n", ptSet.size());  

            // 拟合直线 ， 并确定 bbox
            // float rectK = fitLine(ptSet);
            // 使用 RANSAC 与噪声干扰有关系
            float rectK;
            // if (ptSet.size() > ptSet2.size() * 0.4)
            // {
            if (iCluster == debugID)
                fprintf(stderr, "ptSet size : %d\n", ptSet.size());
            // 判断是否是对称的
            if (debugBool)
            {
                fprintf(stderr, "--- number symmetric points ---%d\n", 
                    (*clusteredPoints[iCluster]).numSymPoints);
            }
            // if ((*clusteredPoints[iCluster]).numSymPoints < 25)
            // {
            // 点多或者边足够长
            // if (ptSet.size() >= 10 || longEdge > 1.5f)
            if (ptSet.size() >= 10)
            {
                float correct = 0.0f;
                if (debugBool)
                    fprintf(stderr, "using fitLineRansac ptSet.size %d, longEdge %f\n", ptSet.size(), longEdge);
                rectK = fitLineRansac(ptSet, 100, 0.12f, correct, cluster.line,debugBool);
                // 对于一些拟合比较好的， 有可能设置的阈值 0.12f 过大， 这样会存在随机任意直线都会满足所有点的情况
                // 所以需要设置一个较小的阈值， 或者对于初步 ransac 筛选后的点用最小二乘法再次拟合一次
                if (correct > 0.85f)
                    rectK = fitLineRansac(ptSet, 100, 0.05f, correct, cluster.line,debugBool);
            }
            else
            {
                if (debugBool)
                    fprintf(stderr, "using fitLine\n");
                rectK = fitLine(ptSet, cluster.line);
            }
            // 保存当前斜率
            // }
            // else
            // {
            //     if (debugBool)
            //     {
            //         fprintf(stderr, "using direction, num %d Symmetric points\n", 
            //                 (*clusteredPoints[iCluster]).numSymPoints);
            //     }
            //     // 对称情况， 直接对全部点云使用 pac 方法  失败了
            //     // rectK = direction(*clusteredPoints[iCluster]);
            //     rectK = direction(ptSet);                
            // }
            
                // fprintf(stderr, "rectK : %f\n", rectK);  
                // 根据方向拟合 bbox 的                
            // }
            // else
            // {
            //     if (ptSet2.size() >= 10)
            //         rectK = fitLineRansac(ptSet2, 100, 0.13);
            //     else
            //         rectK = fitLine(ptSet2);
            //     // fprintf(stderr, "rectK : %f\n", rectK);                  
            // }
            // 根据方向拟合 bbox 的
            // 首先利用 轮廓点和指定方向来判断是 L 型 或者 I 型
            fitRect(rectK, clusterTmp, pcPoints);
            // 如果相邻俩边的比例 大于 5 或者 小于 1/5 判定为 I 型， 或则为 L 型
            float bboxLength1 = sqrt((pcPoints[2].x - pcPoints[1].x) * (pcPoints[2].x - pcPoints[1].x) + 
                                     (pcPoints[2].y - pcPoints[1].y) * (pcPoints[2].y - pcPoints[1].y));
            float bboxLength2 = sqrt((pcPoints[1].x - pcPoints[0].x) * (pcPoints[1].x - pcPoints[0].x) + 
                                     (pcPoints[1].y - pcPoints[0].y) * (pcPoints[1].y - pcPoints[0].y));

            // 比例不符合， 或者角 AOB 大于一定的角度时候， 比如 130 度， 判断其为 ISHAPE
            if (bboxLength1 / bboxLength2 > 5 || bboxLength1 / bboxLength2 < 1.0/5.0 ||
                cosThetaAOB < std::cos(130.0f / 180 * M_PI))
            {
                cluster.shape = shapeType::ISHAPE;
            }
            else
            {
                cluster.shape = shapeType::LSHAPE;
            }

            fitRect(rectK, *clusteredPoints[iCluster],  pcPoints);
            hasRect = true;
            if (debugBool)
            {
                fprintf(stderr, "bboxLength1 %f, bboxLength2 %f\n", bboxLength1, bboxLength2);
                fprintf(stderr, "\ncurrent 2 rectK : %f\n", rectK);
            }
            // fprintf(stderr, "pcPoints:\n");
            // for (int idx = 0; idx < 4; ++idx)
            //     fprintf(stderr, "(%f, %f)\n", pcPoints[idx].x, pcPoints[idx].y);
            // pcPoints[0] = Point2f(minMx, minMy);
            // pcPoints[1] = Point2f(maxDx, maxDy);
            // pcPoints[2] = Point2f(maxMx, maxMy);
            // pcPoints[3] = Point2f(lastX, lastY);
            // bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            // if(!isPromising) 
            //     continue;
        }
        if (!hasRect)
        {
            // fprintf(stderr, "has gone4 !\n");
            //MAR fitting
            // fprintf(stderr, "minAreaRect point size %d\n", pointVec.size());
            auto & cloud = (*clusteredPoints[iCluster]);
            std::vector<Point2f> pointVec(cloud.size());
            for (int idx = 0; idx < cloud.size(); ++idx)
            {
                pointVec[idx] = Point2f(cloud[idx].x(), cloud[idx].y());
            }

            RotatedRect rectInfo = minAreaRect(pointVec);
            // fprintf(stderr, "minAreaRect finised\n");
            Point2f rectPoints[4]; 
            rectInfo.points(rectPoints);
            // covert points back to lidar coordinate
            for (int idx = 0; idx < 4; ++idx)
            {
                pcPoints[idx].x = rectPoints[idx].x;
                pcPoints[idx].y = rectPoints[idx].y;
            }
            // 设置点云类型
            cluster.shape = shapeType::MINAREA;
        }

        // make pcl cloud for 3d bounding box
        // fprintf(stderr, "--------------757\n");
        bool saveIt = ruleBasedFilter(pcPoints, 
            clusteredPoints[iCluster]->maxZ - clusteredPoints[iCluster]->minZ, 
            clusteredPoints[iCluster]->size());
        
        sortClockWise(pcPoints);
        // 不进行保存 bbox
        if (!saveIt)
            continue;   
        // fprintf(stderr, "--------------765\n");
        Cloud::Ptr oneBboxPtr(new Cloud);
        for(int pclH = 0; pclH < 2; pclH++)
        {//底面四个点,上面四个点
            for(int pclP = 0; pclP < 4; pclP++)
            {
                point o;
                o.x() = pcPoints[pclP].x;
                o.y() = pcPoints[pclP].y;
                if(pclH == 0) 
                    o.z() = clusteredPoints[iCluster]->minZ;//车体坐标系下点云,地面高度估计为0.1m
                else 
                    o.z() = clusteredPoints[iCluster]->maxZ;
                oneBboxPtr->emplace_back(o);
            }
        }
        // fprintf(stderr, "--------------782\n");
        bbPoints.emplace_back(oneBboxPtr);
        // 正确的 bbox 新增一
        bboxToCluster[numCorrect] = iCluster;
        // fprintf(stderr, "--------------783\n");
        numCorrect++;
    }

    // 开始判断端点是否被遮挡    
    for (int iCluster = 0; iCluster < clusteredPoints.size(); ++iCluster)
    {
        // fprintf(stderr, "num %d cluster\n", clusteredPoints.size());
        auto &cluster = (*clusteredPoints[iCluster]);
        if (cluster.size() < 3)
            continue;
        // fprintf(stderr, "开始判断端点是否被遮挡: %d, cluster size %d\n", iCluster, cluster.size());
        // fprintf(stderr, "minLpoint %d, maxLPoint %d, shapeToClusterID size %d\n", 
                // cluster.minLPoint, cluster.maxLPoint, shapeToClusterID.size());
        // -pi ~ pi   =>   0 ~ 2 * pi
        // int minLIdx = (cluster[cluster.minLPoint].atan2Val + M_PI) / lshapeResRad + 1;
        // int maxLIdx = (cluster[cluster.maxLPoint].atan2Val + M_PI) / lshapeResRad - 1;
        int minLIdx = (cluster[cluster.minLPoint].atan2Val + M_PI) / lshapeResRad + 0.5 + 1;
        if (minLIdx >= numShapePoints) minLIdx-=numShapePoints;
        int maxLIdx = (cluster[cluster.maxLPoint].atan2Val + M_PI) / lshapeResRad + 0.5 - 1;
        if (maxLIdx < 0) minLIdx = 0;
        // fprintf(stderr, "minLIdx %d, maxLIdx %d\n", minLIdx, maxLIdx);
        // 当掐的端点记录的 检测目标索引 ID 已经被替换掉了， 所以被替换意味着被遮挡
        // 替换意味着其记录的值不再是当前 cluster 记录的 icluster.detectID
        assert(minLIdx >= 0);
        assert(maxLIdx < shapeToClusterID.size());
        if (iCluster == debugID)
        {
            fprintf(stderr, "cluster size %d\n", cluster.size()); 
            fprintf(stderr, "cluster.detectID %d\n", cluster.detectID);
            fprintf(stderr, "cluster[cluster.minLPoint] angle : %f, dist : %f\n", 
                        cluster[cluster.minLPoint].atan2Val * 180 / M_PI,
                        cluster[cluster.minLPoint].toSensor2D);
                        // cluster[cluster.minLPoint].toSensor2D);
            fprintf(stderr, "cluster[cluster.maxLPoint] angle : %f, dist : %f\n", 
                        cluster[cluster.maxLPoint].atan2Val * 180 / M_PI,
                        cluster[cluster.maxLPoint].toSensor2D); 
            fprintf(stderr, "minLIdx : %d, shapeToClusterID[minLIdx] %d, angle : %f, dist : %f\n", 
                        minLIdx,
                        shapeToClusterID[minLIdx],
                        shapeAngleRef[minLIdx] * 180 / M_PI,
                        shapeToDist[minLIdx]);
                        // cluster[cluster.minLPoint].toSensor2D);
            fprintf(stderr, "maxLIdx : %d, shapeToClusterID[maxLIdx] %d, angle : %f, dist : %f\n", 
                        maxLIdx,
                        shapeToClusterID[maxLIdx],
                        shapeAngleRef[maxLIdx] * 180 / M_PI,
                        shapeToDist[maxLIdx]);
                        // cluster[cluster.maxLPoint].toSensor2D);
        }
        if (shapeToClusterID[minLIdx] == cluster.detectID)
        {
            // 相等， 意味着检测未堵塞， 是可信赖的
            cluster.occlusionMin = false;
        }
        if (shapeToClusterID[maxLIdx] == cluster.detectID)
        {
            cluster.occlusionMax = false;
        }

        // 左右移动俩格， 然后比较其距离激光雷达远近即可判断
        // int compShapeIdx = (maxLIdx + 2) % (numShapePoints + 1);
        // if (shapeToDist[maxLIdx] < shapeToDist[compShapeIdx])
        // {
        //     cluster.occlusionMax = false;
        // }
        // compShapeIdx = minLIdx - 2;
        // if (compShapeIdx < 0) compShapeIdx += (numShapePoints + 1);

        // if (shapeToDist[minLIdx] < shapeToDist[compShapeIdx])
        // {
        //     cluster.occlusionMin = false;
        // }

        if (iCluster == debugID)
        {
            for (int idx = 0; idx < shapeToClusterID.size(); ++idx)
            {
                fprintf(stderr, "idx:%d [%d] [%f] [%f]\n", 
                    idx, shapeToClusterID[idx], shapeAngleRef[idx] / M_PI * 180, shapeToDist[idx]);
            }
            fprintf(stderr, "\n");
            fprintf(stderr, "current cluster occlustion:\n");
            fprintf(stderr, "min point: %d\n", cluster.occlusionMin);
            fprintf(stderr, "max point: %d\n", cluster.occlusionMax);
            fprintf(stderr, "shape tyep is : %s\n", shapeName[cluster.shape].c_str());
            fprintf(stderr, "numShape points %d\n", numShapePoints);
            fprintf(stderr, "-------------------\n", numShapePoints);
        }
    }      

    // 重叠部分 bbox 丢弃， 将 bboxToCluster 设置为 -1, 同时对于
    // -1 的部分， 不进行 ref 点的判断， 后续 CloudToBBoxs 函数对于
    // 没有 ref 点的 Cloud 不进行转换， 也不输入到跟踪部分
    removeIntersectBBox(bbPoints, bboxToCluster, debugID);
    // 开始判断跟踪点的选择问题
    getBBoxRefPoint(clusteredPoints, bbPoints, bboxToCluster, markPoints, debugID);
}

bool IsBBoxIntersecting(const BBox & boxA, const BBox & boxB, bool debug)
{
    // fprintf(stderr, "\n\n\n\n");
    
    // 迭代俩个 bbox 每个都选俩个边
    for (int bboxIdx = 0; bboxIdx < 2; ++bboxIdx)
    {
        // 迭代选择俩个边， 俩个 bbox 投影每个边的垂线
        for (int idx = 0; idx < 2; ++idx)
        {
            point p1, p2;
            if (bboxIdx == 0) // bboxA
            {
                p1 = boxA[idx];
                p2 = boxA[idx + 1];
            }
            else // bboB
            {
                p1 = boxB[idx];
                p2 = boxB[idx + 1];
            }            


            // 边的垂直投影向量
            Point2f normal(p2.y() - p1.y(), p1.x() - p2.x());
            if (debug)
            {
                fprintf(stderr, "normal %f,%f\n", normal.x, normal.y);
            }
            float minA = std::numeric_limits<float>::max();
            // float maxA = std::numeric_limits<float>::min(); 理解错误， 最小值是 0， 使用 lowest 最合适
            float maxA = std::numeric_limits<float>::lowest();

            float minB = std::numeric_limits<float>::max();
            float maxB = std::numeric_limits<float>::lowest();

            if (debug)
            {
                fprintf(stderr, "initial: minA , maxA, minB, maxB, %f, %f, %f, %f\n", minA, maxA, minB, maxB);
            }
            // 计算每个 bbox 在投影上的距离，有正负之分
            for (int j = 0; j < 4; ++j)
            {

                float projected = normal.x * boxA[j].x() + normal.y * boxA[j].y();
                if (projected < minA) minA = projected;
                if (projected > maxA) maxA = projected;
                if (debug)
                {
                    fprintf(stderr, "projected %f, boxA[j].x() %f,  boxA[j].y() %f\n", 
                            projected, boxA[j].x(), boxA[j].y());
                }
            }

            for (int j = 0; j < 4; ++j)
            {
                float projected = normal.x * boxB[j].x() + normal.y * boxB[j].y();
                if (projected < minB) minB = projected;
                if (projected > maxB) maxB = projected;
                if (debug)
                {
                    fprintf(stderr, "projected %f, boxB[j].x() %f,  boxB[j].y() %f\n", 
                            projected, boxB[j].x(), boxB[j].y());
                }
            }
            // 找到分离边， 不相交
            if (debug)
                fprintf(stderr, "minA , maxA, minB, maxB, %f, %f, %f, %f\n", minA, maxA, minB, maxB);
            if (maxA < minB || maxB < minA) return false;            
        }
    }
    // 是相交的
    return true;
}

void removeIntersectBBox(const vector<Cloud::Ptr> & bbPoints,
                               std::unordered_map<int, int> & bboxToCluster,
                               const int & debugID)
{
    int numBBoxs = bbPoints.size();
    std::vector<float> areaVec(numBBoxs, -1);
    for (int i = 0; i < numBBoxs; ++i)
    {
        Cloud bboxCurr = (*bbPoints[i]);
        float Area1 = -1;
        if (areaVec[i] < 0)
        {
            // 为了剔除车辆支架点云部分
            Area1 = getCloudBBoxArea(bboxCurr);
            if (bboxCurr[0].dist2D() < 2.5f && Area1 < 0.3f)
                bboxToCluster[i] = -1;
            areaVec[i] = Area1;
        }
        else
        {
            Area1 = areaVec[i];
        }
        
        for (int j = 0; j < numBBoxs; ++j)
        {
            bool debug = false;
            if (i == j) continue;
            if (bboxToCluster[j] == -1) continue;
            Cloud & bboxComp = (*bbPoints[j]);
            bool insertBool = IsBBoxIntersecting(bboxCurr, bboxComp, debug);
            if (insertBool)
            {
                float Area2 = -1;
                if (areaVec[j] < 0)
                {
                    // 为了剔除车辆支架点云部分
                    Area2 = getCloudBBoxArea(bboxComp);
                    if (bboxComp[0].dist2D() < 2.5f && Area2 < 0.3f)
                        bboxToCluster[j] = -1;
                    areaVec[j] = Area2;
                }
                else
                {
                    Area2 = areaVec[j];
                }

                if (Area1 < Area2) bboxToCluster[i] = -1;
                else               bboxToCluster[j] = -1;
            }
        }
    }
}

float getCloudBBoxArea(const Cloud & bbox)
{
    float len1 = distTwoPoint(bbox[0], bbox[1]);
    float len2 = distTwoPoint(bbox[1], bbox[2]);
    return len2 * len1;
}

void sortClockWise(std::vector<Point2f> & bbox)
{
    Point2f midPoint = (bbox[0] + bbox[2]) * 0.5;
    auto fun = [midPoint](const Point2f & p1, const Point2f & p2) {
        return std::atan2((p1 - midPoint).y, (p1 - midPoint).x) > std::atan2((p2 - midPoint).y , (p2 - midPoint).x);};
    std::sort(bbox.begin(), bbox.end(), fun);
}

void getBBoxRefPoint(const vector<Cloud::Ptr> & clusteredPoints, 
                           vector<Cloud::Ptr> & bbPoints,
                           std::unordered_map<int, int> & bboxToCluster, 
                           Cloud::Ptr & markPoints,
                           const int & debugID)
{
    // 避免重复计算， 我们将例如 bbox点存储时候的顺序为 0,1,2,3,4,5,6,7
    // 而我们只需要使用前 0, 1, 2, 3, 4 个点 所以这时候， 我们需要记录点为八个点, -1 表示还未求出跟踪点大小
    // 使用 10, 21， 32, 30, 20 表示前后点，这些复合点 都是由 前后俩个点的索引组成
    // 使用 如 320 表示依赖边为 32 但是没找到 ref 点， 俩端都堵的 ISHAPE 类型
    /*
            0__________10____________1
           |                         |
           |                         |
         30+           20            + 21
           |                         |
           3___________+_____________2
                       32 
    */
    
    // 判断 L I S 三个方向
    for (int bboxIdx = 0; bboxIdx < bbPoints.size(); ++bboxIdx)
    {
        auto & bbox = (*bbPoints[bboxIdx]);
        // 排序之后
        Cloud bboxTmp = bbox;  // 备份， 防止改值, 比如排序使用这种情况
        int clusterIdx = bboxToCluster[bboxIdx];
        // 表示多个 bbox 交叉的情况， 被剔除掉了
        if (clusterIdx == -1) continue;
        auto & cluster = (*clusteredPoints[clusterIdx]);
        
        if (cluster.shape == shapeType::LSHAPE)
        {
            // L shape 的规则为， 不管是那种堵的情况， 都是选择与拐点最近的 bbox 点作为参考点
            // 寻找离 拐点 最近的 bbox 点作为 ref 点
            point cornerPt = cluster[cluster.minOPoint];
            auto posIter = std::min_element(bbox.begin(), bbox.begin() + 4, 
                        [& cornerPt](const point & pt1, const point & pt2)
                            {return distTwoPoint(pt1, cornerPt) < distTwoPoint(pt2, cornerPt);});
            int posIdx = posIter - bbox.begin();
            auto pos = *posIter;
            bbox.refIdx = posIdx;    // 记录拐点索引
            markPoints->emplace_back(point(pos.x(), pos.y(), pos.z(), pointType::TRACK));
        }
        else if (cluster.shape == shapeType::ISHAPE)
        {
            // I shape 的规则
            std::vector<intToPoint> bboxID(4, intToPoint(0, point()));
            assert(bboxID.size() == 4);
            for (int idx = 0; idx < 4; ++idx) bboxID[idx] = intToPoint(idx, bbox[idx]);
            // 先根据 bbox 点距离拟合直线， 对 bboxTmp 点进行排序
            // 使用与拟合直线距离最近的俩个点
            Point2f line = cluster.line;
            std::sort(bboxID.begin(), bboxID.end(), 
                    [&line](const intToPoint & elem1, const intToPoint & elem2)
                    {return pointToLine(elem1.pt, line) < pointToLine(elem2.pt, line);});
            // std::sort(bboxTmp.begin(), bboxTmp.begin() + 4, 
            //             [&line](const point & pt1, const point & pt2)
            //             {return pointToLine(pt1, line) < pointToLine(pt2, line);});
            // 俩者皆堵， 使用预测点在该边上的投影作为 ref 点， 因为这时候的测量已经是不可信赖的点了
            if (cluster.occlusionMax && cluster.occlusionMin)  
            {
                // 没有参考点选择 do nothing
                int oneIdx = bboxID[0].id;
                int twoIdx = bboxID[1].id;
                // 设置可以依赖的边
                bbox.refIdx = std::max(oneIdx, twoIdx) * 100 + std::min(oneIdx, twoIdx) * 10 + 0;
                // if (debugID == clusterIdx)
                // {
                //     fprintf(stderr, "two occlusion refIDx is : %d, pos : (%f, %f)\n", 
                //                 bbox.refIdx, pos.x(), pos.y(), pos.z());
                // }
                // 俩端都堵塞， 并没有绘制点
            }
            else if (cluster.occlusionMin ^ cluster.occlusionMax)  // 俩则互异为真， 一端堵， 一端不堵
            {
                // 找到距离不堵点的最近的 bbox 点云
                point pt = (cluster.occlusionMax) ? cluster[cluster.minLPoint] : cluster[cluster.maxLPoint];
                // 距离拟合直线最近的俩个点， 然后根据堵塞的点找最近的 bbox 点作为 ref 点
                intToPoint res = *std::min_element(bboxID.begin(), bboxID.begin() + 2, 
                        [& pt](const intToPoint & elem1, const intToPoint & elem2)
                        {return distTwoPoint(elem1.pt, pt) < distTwoPoint(elem2.pt, pt);});
                point pos = res.pt;
                markPoints->emplace_back(point(pos.x(), pos.y(), pos.z(), pointType::TRACK));

                // 添加参考点索引
                bbox.refIdx = res.id;
            }
            else // (!cluster.occlusionMin && !cluster.occlustionMax) // 俩则都不堵
            {
                point pos = (bboxID[0].pt + bboxID[1].pt) * 0.5;
                markPoints->emplace_back(point(pos.x(), pos.y(), pos.z(), pointType::TRACK));
                
                int oneIdx = bboxID[0].id;
                int twoIdx = bboxID[1].id;
                // 添加参考点索引
                bbox.refIdx = std::max(oneIdx, twoIdx) * 10 + std::min(oneIdx, twoIdx);
            }
            
        }
        else if (cluster.shape == shapeType::SYMETRIC)
        {
            // 对称的情况也分三种， 不过主要是找寻面向激光雷达最近的 bbox 点， 而非寻找
            // 与 cluster 某个参考点的最近的 bbox 点， 因为特殊性
            // 有一端堵塞
            // 从小到大排列, 只排序前 4 个
            std::vector<intToPoint> bboxID(4, intToPoint(0, point()));
            assert(bboxID.size() == 4);
            for (int idx = 0; idx < 4; ++idx)  bboxID[idx] = intToPoint(idx, bbox[idx]);
            // 先根据 bbox 点距离拟合直线， 对 bboxTmp 点进行排序
            // 使用与拟合直线距离最近的俩个点
            Point2f line = cluster.line;
            std::sort(bboxID.begin(), bboxID.end(), 
                    [](const intToPoint & elem1, const intToPoint & elem2)
                    {return elem1.pt.dist2D() < elem2.pt.dist2D();});
            
            if (debugID == clusterIdx)
            {
                for (int i = 0; i < bboxTmp.size(); ++i)
                {
                    fprintf(stderr, "(%f, %f) --> %f\n", bboxTmp[i].x(), bboxTmp[i].y(), bboxTmp[i].toSensor2D);
                }
            }
            if (cluster.occlusionMin ^ cluster.occlusionMax)
            {
                point pt = (cluster.occlusionMax) ? cluster[cluster.minLPoint] : cluster[cluster.maxLPoint];
                auto res = *std::min_element(bboxID.begin(), bboxID.begin() + 2, 
                    [& pt](const intToPoint & elem1, const intToPoint & elem2)
                    {return distTwoPoint(elem1.pt, pt) < distTwoPoint(elem2.pt, pt);});
                point pos = res.pt;
                markPoints->emplace_back(point(pos.x(), pos.y(), pos.z(), pointType::TRACK));

                // 添加参考点
                bbox.refIdx = res.id;
            }
            else  // 俩端都不堵， 俩端都堵
            {
                int oneIdx = bboxID[0].id;
                int twoIdx = bboxID[1].id;
                point pos = (bboxID[0].pt + bboxID[1].pt) * 0.5;
                markPoints->emplace_back(point(pos.x(), pos.y(), pos.z(), pointType::TRACK));
                // 添加参考点
                bbox.refIdx = std::max(oneIdx, twoIdx) * 10 + std::min(oneIdx, twoIdx);
            }
            
        }
        else // cluster.shape = shapeType::MINAREA
        {
            // 以中心点为参考点
            point pos = (bbox[0] + bbox[2]) * 0.5;
            markPoints->emplace_back(point(pos.x(), pos.y(), pos.z(), pointType::TRACK));
            bbox.refIdx = 20;
        }

        if (debugID == clusterIdx)
        {
            fprintf(stderr, "refIdx is : %d\n", bbox.refIdx);
        }
        
    }
}

bool IsBBoxIntersecting(const Cloud & boxA, const Cloud & boxB, bool debug)
{
    for (int bboxIdx = 0; bboxIdx < 2; ++bboxIdx)
    {
        // 迭代选择俩个边， 俩个 bbox 投影每个边的垂线
        for (int idx = 0; idx < 2; ++idx)
        {
            point p1, p2;
            if (bboxIdx == 0) // bboxA
            {
                p1 = boxA[idx];
                p2 = boxA[idx + 1];
            }
            else // bboB
            {
                p1 = boxB[idx];
                p2 = boxB[idx + 1];
            }            


            // 边的垂直投影向量
            Point2f normal(p2.y() - p1.y(), p1.x() - p2.x());
            if (debug)
            {
                fprintf(stderr, "normal %f,%f\n", normal.x, normal.y);
            }
            float minA = std::numeric_limits<float>::max();
            // float maxA = std::numeric_limits<float>::min(); 理解错误， 最小值是 0， 使用 lowest 最合适
            float maxA = std::numeric_limits<float>::lowest();

            float minB = std::numeric_limits<float>::max();
            float maxB = std::numeric_limits<float>::lowest();

            if (debug)
            {
                fprintf(stderr, "initial: minA , maxA, minB, maxB, %f, %f, %f, %f\n", minA, maxA, minB, maxB);
            }
            // 计算每个 bbox 在投影上的距离，有正负之分
            for (int j = 0; j < 4; ++j)
            {

                float projected = normal.x * boxA[j].x() + normal.y * boxA[j].y();
                if (projected < minA) minA = projected;
                if (projected > maxA) maxA = projected;
                if (debug)
                {
                    fprintf(stderr, "projected %f, boxA[j].x() %f,  boxA[j].y() %f\n", 
                            projected, boxA[j].x(), boxA[j].y());
                }
            }

            for (int j = 0; j < 4; ++j)
            {
                float projected = normal.x * boxB[j].x() + normal.y * boxB[j].y();
                if (projected < minB) minB = projected;
                if (projected > maxB) maxB = projected;
                if (debug)
                {
                    fprintf(stderr, "projected %f, boxB[j].x() %f,  boxB[j].y() %f\n", 
                            projected, boxB[j].x(), boxB[j].y());
                }
            }
            // 找到分离边， 不相交
            if (debug)
                fprintf(stderr, "minA , maxA, minB, maxB, %f, %f, %f, %f\n", minA, maxA, minB, maxB);
            if (maxA < minB || maxB < minA) return false;            
        }
    }
    // 是相交的
    return true;
}

void setShapeOcclusionCheck(std::vector<float> & shapeToDist,
                std::vector<int> & shapeToClusterID,
                std::vector<float> & shapeAngleRef,
                const Cloud & cluster,
                const float & lshapeResRad
            )
{
    int detectID = cluster.detectID;
    for (int ptIdx = 0; ptIdx < cluster.size(); ++ptIdx)
    {
        int colIdx = (cluster[ptIdx].atan2Val + M_PI) / lshapeResRad;
        if (cluster[ptIdx].toSensor2D < shapeToDist[colIdx])
        {
            // if (detectID == 99)
            //     fprintf(stderr, "cluster[ptIdx].toSensor2D %f, shapeToDist[colIdx] %f\n",
            //         cluster[ptIdx].toSensor2D, shapeToDist[colIdx]);
            // fprintf(stderr, "colIdx: %d\n", colIdx);
            // fprintf(stderr, "detectID: %d\n", detectID);
            shapeToDist[colIdx] = cluster[ptIdx].toSensor2D;
            shapeToClusterID[colIdx] = detectID;
            shapeAngleRef[colIdx] = cluster[ptIdx].atan2Val;
        }
    }
}

// std::vector<Vertex> CloudToVertexs(const Cloud::Ptr & cloud, float & minZ, float & maxZ)
void CloudToVertexs(const Cloud::Ptr & cloud, 
            std::vector<Vertex> & bottomVertexs
            // std::vector<Vertex> & topVertexs
            )
{
    // std::vector<Vertex> res(cloud->size());
    bottomVertexs.clear();
    // topVertexs.clear();
    float & maxZ = cloud->maxZ;
    float & minZ = cloud->minZ;

    // float split = minZ + (maxZ - minZ) * 2 / 3;
    for (size_t idx = 0; idx < cloud->size(); ++idx)
    {
        Vertex vet;
        vet.x = (*cloud)[idx].x();
        vet.y = (*cloud)[idx].y();
        float & z = (*cloud)[idx].z();
        // if ((*cloud)[idx].z() < minZ)
        //     minZ = (*cloud)[idx].z();
        // if ((*cloud)[idx].z() > maxZ)
        //     maxZ = (*cloud)[idx].z();

        // res.emplace_back(vet);
        // res[idx] = vet;
        // 分割为上下俩个部分， 为了防止车耳朵造成偏差
        // if (z < split)
            bottomVertexs.emplace_back(vet);
        // else
        //     topVertexs.emplace_back(vet);        
    }

    // assert(cloud->size() == res.size());
    // fprintf(stderr, "point info:\n");
    // for (size_t idx = 0; idx < cloud->size(); ++idx)
    // {
    //     fprintf(stderr, "[%f, %f] <--> [%f, %f]\n", 
    //                 (*cloud)[idx].x(),
    //                 (*cloud)[idx].y(),
    //                 res[idx].x,
    //                 res[idx].y);
    // }
}

// ransac 算法
float fitLineRansac(const vector<cv::Point2f>& clouds, 
                    const int & num_iterations,
                    const float & tolerance,                    
                    float & correct,
                    Point2f & line,
                    const bool & debug)
{
    if (debug)
        fprintf(stderr, "num %d point fit ransac line \n", clouds.size());
    int max_num_fit = 0;
    // 最优直线斜率向量表示
    Point2f best_plane(0.0f, 0.0f);
    // 最优直线穿过的点
    Point2f best_point(0.0f, 0.0f);
    mt19937_64 mt(0);
    uniform_int_distribution<> randPoints(0, clouds.size() - 1);
    // if (debug)
    // {
    //     fprintf(stderr, "clouds.size() %d\n", clouds.size());
    // }
    
    for(int i = 0; i < num_iterations; ++i) 
    {
        // -- Get two random points.
        Point2f p;
        // int idx0 = rand() % clouds.size();
        size_t idx0 = randPoints(mt);
        p = clouds[idx0];

        Point2f q;
        // int idx1 = rand() % clouds.size();
        size_t idx1 = randPoints(mt);
        q = clouds[idx1];

        // if (debug)
        //     fprintf(stderr, "idx0 %d, idx1 %d\n", idx0, idx1);

        // 选择了相同的点
        // 异常点 排除
        // if (p.x > 300.0f || p.x < -300.0f ||
        //     p.y > 300.0f || p.y < -300.0f ||
        //     q.x > 300.0f || q.x < -300.0f ||
        //     q.y > 300.0f || q.y < -300.0f)
        //     continue;
        // if (!isfinite(p.x)||
        //     !isfinite(p.y)||
        //     !isfinite(q.x)||
        //     !isfinite(q.y))
        //     continue;

        if (p.x == q.x && p.y == q.y)
            continue;

        // -- Fit a line a'x = b.
        Point2f a;
        a.x = 1; //初始化其中一个轴为1, 后面归一化后， 就变成斜率向量了
        if(std::fabs(p.y - q.y) < 1e-4)
            continue;
        else
            a.y = -(p.x - q.x) / (p.y - q.y);        // 归一化斜率向量
        // if (debug)
        //     fprintf(stderr, "p.x %f, q.x %f, p.y %f, q.y %f \n", p.x, q.x, p.y, q.y);

        float normVal = std::sqrt(a.x * a.x + a.y * a.y);
        // if (debug)
        //     fprintf(stderr, "a.x %f, a.y %f, normVal = %f\n", 
        //     a.x, a.y,normVal);
        a.x = a.x / normVal;
        a.y = a.y / normVal;

        // a.normalize();
        float b = a.dot(p);
        // if (debug)
        //     fprintf(stderr, "a.x %f, a.y %f, fabs(a.x * a.x + a.y * a.y - 1) = %f\n", 
        //             a.x, a.y, fabs(a.x * a.x + a.y * a.y - 1));
        assert(fabs(a.x * a.x + a.y * a.y - 1) < 1e-3);
        assert(fabs(a.dot(p - q)) < 1e-3);
        assert(fabs(a.dot(q) - b) < 1e-3);

        // -- Count how many other points are close to the line.

        int num_fit = 0;
        for(int i = 0; i < clouds.size(); ++i) 
        {
            float adotx = a.dot(clouds[i]);
            if(fabs(adotx - b) <= tolerance)
                ++num_fit;
        }

        if(num_fit > max_num_fit) 
        { 
            max_num_fit = num_fit;
            best_plane = a;
            best_point = p;
        }
    }
    // return best_plane;
    // 返回 k

    correct = 1.0f * max_num_fit / clouds.size();
    if (debug)
    {
        fprintf(stderr, "Correct percent : %f\n", correct);
    }
    // if (debug)
    // {
    //     fprintf(stderr, "res line %f, max_num_fit %d\n", -1 * best_plane.x / best_plane.y, max_num_fit);
    //     fprintf(stderr, "fit %f point\n", 1.0f * max_num_fit / clouds.size());
    // }
    // 最优直线的斜率， 与拟合直线的方向相同
    float K = -1 * best_plane.x / (best_plane.y + 1e-6);
    // 最优直线的截距
    float b = best_point.y - K * best_point.x; 
    line.x = K;
    line.y = b;
    return K;
}

// 最小二乘法
float fitLine(const std::vector<Point2f> & points, Point2f & line)
{
    const unsigned int n_points = points.size();
    Eigen::MatrixXd X(n_points, 2);
    Eigen::VectorXd Y(n_points);

    unsigned int counter = 0;
    for (auto it = points.begin(); it != points.end(); ++it)
    {
        X(counter, 0) = it->x;
        X(counter, 1) = 1;
        Y(counter) = it->y;
        ++counter;
    }

    // y = k * x + b; 返回 k 和 b
    Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
    line.x = result(0);
    line.y = result(1);
    return result(0);
}

float direction(const Cloud & cloud)
{
    cv::Mat pcaSet(cloud.size(), 2, CV_32FC1, cv::Scalar::all(0));
    float *data = nullptr;
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        data = pcaSet.ptr<float>(i);
        *data++ = cloud[i].x();
        *data++ = cloud[i].y();
    }
    // 参数依次为：原始数据；原始数据均值，输入空会自己计算；
    // 每行/列代表一个样本；保留多少特征值，默认全保留
    cv::PCA pca(pcaSet, cv::Mat(), CV_PCA_DATA_AS_ROW, 2);
    cv::Mat directionVec = pca.eigenvectors;
    float k = directionVec.at<float>(0, 1) / 
            directionVec.at<float>(0, 0);
    return k;
}

float direction(const std::vector<Point2f> & cloud)
{
    cv::Mat pcaSet(cloud.size(), 2, CV_32FC1, cv::Scalar::all(0));
    float *data = nullptr;
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        data = pcaSet.ptr<float>(i);
        *data++ = cloud[i].x;
        *data++ = cloud[i].y;
    }
    // 参数依次为：原始数据；原始数据均值，输入空会自己计算；
    // 每行/列代表一个样本；保留多少特征值，默认全保留
    cv::PCA pca(pcaSet, cv::Mat(), CV_PCA_DATA_AS_ROW, 2);
    cv::Mat directionVec = pca.eigenvectors;
    float k = directionVec.at<float>(0, 1) / 
            directionVec.at<float>(0, 0);
    return k;
}

int ColFromAngle(float angle_cols, 
                    float minAngle, 
                    float maxAngle, 
                    const float & step,
                    const bool & debug)
{
    // 每一份的步长 step
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

int ColFromAngle(const float & angle_cols, 
                    const float & minAngle, 
                    const float & maxAngle, 
                    const float & OAngle,
                    const float & minToOStep,
                    const float & maxToOStep,
                    // 有个预先的值 0，1,2,3,4 --- （4 + 0, 4 + 1）第二段索引不是从零开始的
                    const int & numMinSample, 
                    const bool & debug)
{
    // fprintf(stderr, "angles_cols %f, OAngle %f, maxToOStep %f, minToOStep %f\n",
    //             angle_cols, OAngle, maxToOStep, minToOStep);
    if (angle_cols > OAngle)
    {
        return std::floor((angle_cols - OAngle) / (maxToOStep)) + numMinSample;
    }
    else
    {
        return std::floor((angle_cols - minAngle) / (minToOStep));
    }    
}

float distTwoPoint(const point & p1, const point & p2)
{
    return sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y()));
}

// 点到直线的距离
float pointToLine(const point & pt, const Point2f & line)
{
    return abs(line.x * pt.x() - pt.y() + line.y) / sqrt(line.x * line.x + 1);
}

void getLShapePoints(Cloud & cluster, 
                    const float & maxDx, 
                    const float & maxDy, 
                    const bool & needFixed,
                    const bool & debugBool,
                    const float & lShapeHorizonResolution)
{
    if (debugBool)
        fprintf(stderr, "current cluster has %d points\n", cluster.size());
    std::vector<int> colVec;
    int ptIdx = 0;
    if (cluster.size() < 10)
        return;
    // 是否需要修正跨越角度
    float minAngle = cluster.minAngle;
    float maxAngle = cluster.maxAngle;  
    // 拐点的角度
    float OAngle = atan2(maxDy, maxDx);   
    if (needFixed) OAngle = (OAngle > 0) ? (OAngle - M_PI) : (OAngle + M_PI);    
    // 计算俩侧的采样比例， 尝试一人一半， 或者长度比划分
    float distMinAngleToOPoint = distTwoPoint(point(maxDx, maxDy, 0.0f), cluster[cluster.minLPoint]);
    float distMaxAngleToOpoint = distTwoPoint(point(maxDx, maxDy, 0.0f), cluster[cluster.maxLPoint]);
    float distMinToOrigin = distTwoPoint(point(0.0f, 0.0f, 0.0f), cluster[cluster.minLPoint]);
    float distMaxToOrigin = distTwoPoint(point(0.0f, 0.0f, 0.0f), cluster[cluster.maxLPoint]);
    // 如果对称， 那么车宽的估值为这个数
    float carWidth = 2 * max(distMinToOrigin, distMaxToOrigin) * sin((maxAngle - minAngle) / 2);
    // 逻辑上， 角度对应的点的数量
    int numSampPoint = std::ceil((maxAngle - minAngle) / M_PI * 180 / lShapeHorizonResolution);
    if (carWidth > 1.30f && carWidth < 2.05f)
        numSampPoint /= 2;
    int numSampPointComp;
    if (debugBool)
        fprintf(stderr, "numSampPoint before %d\n", numSampPoint);
    // if (numSampPoint > 80 && numSampPoint < 150)
    //     numSampPoint = 80;
    // int nSegment = 10; //  分割为十等份
    int nSegment = 20;
    if (numSampPoint < 150)
    {
        numSampPoint = std::ceil(numSampPoint * 1.0f / nSegment) * nSegment - 1;
        // numSampPoint = numSampPoint - numSampPoint % nSegment - 1;
    }
    if (numSampPoint > 150)
        numSampPoint = 149;
    // 此处为赋值， 并非判断是否相等  
    if (debugBool)
        fprintf(stderr, "numSampPoint after %d\n", numSampPoint);
    numSampPointComp = numSampPoint;
    // colIdx 到 实际点云簇中的点
    std::vector<int> ToPointID(numSampPointComp + 1, -1);
    // 存储最小的距离
    std::vector<float> ToDist(numSampPointComp + 1, 999.0f);
    if (debugBool)
        fprintf(stderr, "cluster has numPoint: %d\n", cluster.size());
    if (debugBool)
    {
        fprintf(stderr, "numSampPointComp %d\nnumSampPoint %d\nlShapeHorizonResolution %f\n",
                         numSampPointComp,
                         numSampPoint,
                         lShapeHorizonResolution);
    }

    // 按俩边的长度比重旋转选点的密度
    int numMinSample = floor(distMinAngleToOPoint / (distMinAngleToOPoint + distMaxAngleToOpoint) * numSampPointComp);
    int numMaxSample = numSampPointComp - numMinSample;
    if (debugBool)
        fprintf(stderr, "distMinAngleToOPoint %f, distMaxAngleToOpoint %f\n", 
                        distMinAngleToOPoint, distMaxAngleToOpoint); 
    float minToOStep = (OAngle - minAngle) / (numMinSample);
    float maxToOStep = (maxAngle - OAngle) / (numMaxSample);
    if (debugBool)
        fprintf(stderr, "numMinSample %d, numMaxSample %d, (OAngle - minAngle) %f, (maxAngle - OAngle) %f\n",
                        numMinSample, numMaxSample, (OAngle - minAngle), (maxAngle - OAngle));
    if (debugBool)
    {    
        fprintf(stderr, "\ncluster size %d\n", cluster.size());
        fprintf(stderr, "maxAngle %f, minAngle %f, OAngle %f\n", maxAngle, minAngle, OAngle);
    }
    for (auto ptIt = cluster.begin(); ptIt != cluster.end(); ++ptIt)
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
        // colIdx = ColFromAngle(angle, minAngle, maxAngle, numSampPointComp, debugBool);
        // size_t colIdx;  // 此处因为使用 size_t 访问内存报错， 段核心已存储， 记坑坑坑坑坑
        int colIdx; // 可能有一边为 0 长度的情况
        if (numMinSample == 0 || numMaxSample == 0)
        {
            float stepTheta = (maxAngle - minAngle) / numSampPointComp;
            colIdx = ColFromAngle(angle, minAngle, maxAngle, stepTheta, debugBool);
        }
        else
        {
            colIdx = ColFromAngle(angle, minAngle, maxAngle, OAngle, minToOStep, maxToOStep, numMinSample, debugBool);
        }        

        // if (debugBool)
        // {
        //     fprintf(stderr, "colIdx : %d\n", colIdx);
        // }
        // if (colIdx >= 20)
        //     fprintf(stderr, "colIdx %d, angle %f, minAngle %f, maxAngle %f\n", 
        //                 colIdx, angle, minAngle, maxAngle);
        // assert(colIdx <= numSampPointComp); // 不能选取超过20个点的数目
        // 没有该索引， 就直接插入
        // if (debugBool)
            // fprintf(stderr, "colIdx %d, ToDist size %d, ptIt->toSensor2D %f, ToDist[colIdx] %f\n", 
            //         colIdx, ToDist.size(), ptIt->toSensor2D, ToDist[colIdx]);

        if (ptIt->toSensor2D < ToDist[colIdx])
        {
            ToDist[colIdx] = ptIt->toSensor2D;
            ToPointID[colIdx] = ptIdx;
        }
        ptIdx++;
    }   

    if (debugBool)
    {
        for (int idx = 0; idx < ToDist.size(); ++idx)
            fprintf(stderr, "%f ", ToDist[idx]);
        fprintf(stderr, "\n");
    } 

    int numPoints = ToDist.size();
    if (debugBool)
    {
        float minDistGlobal = *std::min_element(ToDist.begin(), ToDist.end());
        // 绘制*号， 相当于值方图
        for (int idx = 0; idx < numPoints/2; idx++)
        {
            float minDist = ToDist[2 * idx];
            for (int i = 1; i < 2;++i)
            {
                if (minDist > ToDist[2 * idx + i])
                    minDist = ToDist[2 * idx + i]; 
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

    for (int idx = 0; idx < ToPointID.size(); ++idx)
    {
        if (ToPointID[idx] == -1)
        {
            // fprintf(stderr, "idx %d\n", idx);
            continue;
        }
        // 可视化的点
        // points->emplace_back((*cluster)[ToPointID[idx]]);
        cluster[ToPointID[idx]].isLShapePoint = 1;
    }
    cluster.numNoneEmptyLShapePoint = ToDist.size();
    if (debugBool)
        fprintf(stderr, "distMinToOrigin %f, carWidth %f\n", distMinToOrigin, carWidth);
    if (debugBool)
        fprintf(stderr, "\n\n\n\n\n\n\n\n");
    
    // 先判断车宽度， 然后判断是否需要对称检测
    // 原先设置 carWidth 为 2.05 放宽检测条件为 2.30f
    std::array<float, 10> tolerence {0.04, 0.04, 0.04, 0.06, 0.06, 0.1, 0.1, 0.2,0.24, 0.3};
    if (carWidth > 1.30f && carWidth < 2.30f && numSampPoint >= nSegment)
    {
        // 重新求取一次 ToDist[], 不进行划分
        int colIdx = 0;
        float stepTheta = (maxAngle - minAngle) / numSampPointComp;
        for (auto ptIt = cluster.begin(); ptIt != cluster.end(); ++ptIt)
        {
            // float angle = std::atan2(ptIt->y(), ptIt->x());
            float angle = ptIt->atan2Val;
            if (needFixed)
            {
                if (angle > 0)    angle -= M_PI;
                else              angle += M_PI;            
            }            
            colIdx = ColFromAngle(angle, minAngle, maxAngle, stepTheta, debugBool);    
            if (ptIt->toSensor2D < ToDist[colIdx])   ToDist[colIdx] = ptIt->toSensor2D;
        }   
        //--------------------
        int NumSymSegs = 0;
        int nPointPreSeg = numSampPoint / nSegment; 
        // 俩部分的距离和， 用来求比值
        float frontSumDist = 0;
        float endSumDist = 0;
        // 最小距离
        float minDist = (*std::min_element(ToDist.begin(), ToDist.end()));
        // int numFrontDist = 0;
        // int numEndDist = 0;

        // int numDist = ToDist.size();
        // for (int idx = 0; idx < numDist / 2; ++idx)
        // {
        //     if (ToDist[idx] < 300.0f)
        //     {
        //         frontSumDist += ToDist[idx];
        //         ++numFrontDist;
        //     }

        //     if (ToDist[numDist - idx - 1] < 300.0f)
        //     {
        //         endSumDist += ToDist[numDist - idx - 1];
        //         ++numEndDist;
        //     }
        // }

        // frontSumDist = frontSumDist / numFrontDist - minDist;
        // endSumDist = endSumDist / numEndDist - minDist;

        for (int idx = 0; idx < nSegment / 2; ++idx)
        {
            if (debugBool)
                fprintf(stderr, "idx %d\n", idx);
            int frontIdx = idx;
            int backIdx = nSegment - frontIdx - 1;
            int NumFrontPts = 0, NumBackPts = 0;
            float SumFront = 0.0f, SumBack = 0.0f;
            for (int i = 0; i < nSegment; ++i)
            {
                // if (debugBool)
                //     fprintf(stderr, "Front point idx %d\n", frontIdx * nPointPreSeg + i);
                float currentValue = ToDist[frontIdx * nPointPreSeg + i];
                if (currentValue < 300.0f)
                {
                    ++NumFrontPts;
                    SumFront += currentValue;
                }
            }

            for (int i = 0; i < nSegment; ++i)
            {
                // if (debugBool)
                    // fprintf(stderr, "back point idx %d\n", backIdx * nPointPreSeg + i);
                float currentValue = ToDist[backIdx * nPointPreSeg + i];
                if (currentValue < 300.0f)
                {
                    ++NumBackPts;
                    SumBack += currentValue;
                }
            }
            float frontAveValue = SumFront / (1e-3 + NumFrontPts);
            float backAveValue = SumBack / (1e-3 + NumBackPts);
            frontSumDist += frontAveValue;
            endSumDist += backAveValue;
            // float frontAveValue = *std::min_element(ToDist.begin() + frontIdx * nPointPreSeg, 
            //     ToDist.begin() + (frontIdx + 1) * nPointPreSeg - 1);
            // float backAveValue = *std::min_element(ToDist.begin() + backIdx * nPointPreSeg,
            //     ToDist.begin() + (backIdx + 1) * nPointPreSeg - 1);
            // if (std::abs(frontAveValue - backAveValue) < 0.25f)
            // if (std::abs(frontAveValue - backAveValue) < 0.08 + (nSegment / 2 - idx) * 0.03)
            if (std::abs(frontAveValue - backAveValue) < tolerence[nSegment / 2 - idx - 1])
            {
                ++NumSymSegs;
            }
            if (debugBool)
                fprintf(stderr, "diff of dist : %f, stand diff :%f\n", 
                // std::abs(frontAveValue - backAveValue), 0.08 + (nSegment / 2 - idx) * 0.03);
                std::abs(frontAveValue - backAveValue), tolerence[nSegment / 2 - idx - 1]);
        }

        frontSumDist = 2 * frontSumDist / nSegment -  minDist;
        endSumDist = 2 * endSumDist / nSegment - minDist;
        
        cluster.SymPointPercent = NumSymSegs * 1.0f / nSegment;
        // if (cluster.SymPointPercent > 0.35 && (frontSumDist / endSumDist) < 1.1 && (frontSumDist / endSumDist) > 0.9)
        if (cluster.SymPointPercent >= 0.34 && std::abs(frontSumDist - endSumDist) <= 0.12)
        {
            cluster.isSymCloud = true;
        }

        if (debugBool)
        {
            fprintf(stderr, "NumSymSegs %d\n", NumSymSegs);
            fprintf(stderr, "isSymCloud %d\n", cluster.isSymCloud);
            fprintf(stderr, "SymPointPrecent %f\n", cluster.SymPointPercent);
            // fprintf(stderr, "(frontSumDist / endSumDist) %f, %f, %f, %f\n", 
            //             (frontSumDist / endSumDist), (endSumDist / frontSumDist),
            //             frontSumDist, endSumDist);
            fprintf(stderr, "abs(frontSumDist - endSumDist) =  %f,     %f,  %f\n", 
                std::abs(frontSumDist - endSumDist),
                frontSumDist, endSumDist);
        }
        /*
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
                cluster.numSymPoints++;
            }
            if (debugBool)
                fprintf(stderr, "\n");                
        }
        */
    }

    
}


void fitRect(const float & k, const Cloud & cloud, std::vector<Point2f> & rect)
{
    rect.reserve(4);
    float k_1 = -1 / k;

    // fprintf(stderr, "fitRect:\n");
    // fprintf(stderr, "cloud size : %d\n", cloud.size());
    // 斜率不为无穷时
    if(k_1 != -std::numeric_limits<float>::infinity() && 
        k_1 != std::numeric_limits<float>::infinity() &&
        k !=  -std::numeric_limits<float>::infinity() &&
        k !=  std::numeric_limits<float>::infinity())
    {
        // 初始化
        float b_1, b_2, b_3, b_4;
        b_1 = cloud[0].y() - k * cloud[0].x();
        b_2 = cloud[0].y() - k * cloud[0].x();
        b_3 = cloud[0].y() - k_1 * cloud[0].x();
        b_4 = cloud[0].y() - k_1 * cloud[0].x();

        for (size_t i = 0; i < cloud.size(); ++i)
        {
                  //点在直线的左边
            if(k * cloud[i].x() + b_1 <= cloud[i].y())
            {
                b_1 = cloud[i].y() - k * cloud[i].x();
            }

            if(k * cloud[i].x() + b_2 >= cloud[i].y())
            {
                b_2 = cloud[i].y() - k * cloud[i].x();
            }

            if(k_1 * cloud[i].x() + b_3 <= cloud[i].y())
            {
                b_3 = cloud[i].y() - k_1 * cloud[i].x();
            }

            if(k_1 * cloud[i].x() + b_4 >= cloud[i].y())
            {
                b_4 = cloud[i].y() - k_1 * cloud[i].x();
            }
        }

        // fprintf(stderr, "b_1, b_2, b_3, b_4, k, k_1--> (%f, %f, %f, %f, %f, %f)\n",
        //              b_1, b_2, b_3, b_4, k, k_1);
        Point2f vertex;
        vertex.x = (b_3 - b_1) / (k - k_1);
        vertex.y = vertex.x * k + b_1;
        rect[0] = vertex;

        vertex.x = (b_2 - b_3) / (k_1 - k);
        vertex.y = vertex.x * k + b_2;
        rect[1] = vertex;

        vertex.x = (b_4 - b_2) / (k - k_1);
        vertex.y = vertex.x * k + b_2;
        rect[2] = vertex;
        vertex.x = (b_1 - b_4) / (k_1 - k);
        vertex.y = vertex.x * k + b_1;
        rect[3] = vertex;
    }
    else
    {
        float max_x = -std::numeric_limits<float>::infinity();
        float max_y = -std::numeric_limits<float>::infinity();
        float min_x = std::numeric_limits<float>::infinity();
        float min_y = std::numeric_limits<float>::infinity();

        for(size_t i = 0; i < cloud.size(); i++)
        {
            max_x = (cloud[i].x() > max_x)?cloud[i].x():max_x;
            max_y = (cloud[i].y() > max_y)?cloud[i].y():max_y;
            min_x = (cloud[i].x() < min_x)?cloud[i].x():min_x;
            min_y = (cloud[i].y() < min_y)?cloud[i].y():min_y;
        }
        Point2f vertex;
        vertex.x = max_x;
        vertex.y = max_y;
        rect[0] = vertex;

        vertex.x = max_x;
        vertex.y = min_y;
        rect[1] = vertex;

        vertex.x = min_x;
        vertex.y = min_y;
        rect[2] = vertex;

        vertex.x = min_x;
        vertex.y = max_y;
        rect[3] = vertex;
    }
    
}

void getOrientedBBox(const vector<Cloud::Ptr> & clusteredPoints,
                    vector<Cloud::Ptr> & bbPoints)
{
    // 遍历所有 cloud
    // fprintf(stderr, "number of clusters %d\n", clusteredPoints.size());
    // fprintf(stderr, "------------------3.1\n");
    for (size_t idx = 0; idx < clusteredPoints.size(); ++idx)
    {
        // float minZ = 1000, maxZ = -1000;
        vector<Point2f> pcPoints(4);
        if (clusteredPoints[idx]->size() > 40)       
        { 
            // vector<Vertex> vertexs = CloudToVertexs(clusteredPoints[idx], minZ, maxZ);
            vector<Vertex> bottomVertexs;//, topVertexs;
            CloudToVertexs(clusteredPoints[idx], bottomVertexs);

            // if (bottomVertexs.size() > 20);
            // {
                // fprintf(stderr, "points about : %d\n", bottomVertexs.size());
                // fprintf(stderr, "------------------points size : %d\n", vertexs.size());
                // vector<Vertex> vertexs = generatePoints(100);
                // ConvexHull convex_hull(vertexs);
                std::shared_ptr<ConvexHull> convexHullBottom(new ConvexHull(bottomVertexs));
                vector<Vertex> pointsHullBottom = convexHullBottom->vertices_;
                pointsHullBottom = convexHullBottom->toRec1(); 

                // std::shared_ptr<ConvexHull> convexHullTop(new ConvexHull(topVertexs));
                // vector<Vertex> pointHullTop = convexHullTop->vertices_;
                // pointHullTop = convexHullTop->toRec1();

                // assert(points_hull.size() == 4);
                // for (size_t idx = 0; idx < points_hull.size(); ++idx)
                // {
                //     fprintf(stderr, "[%f, %f]\n", points_hull[idx].x, points_hull[idx].y);
                // }

                for (size_t pcIdx = 0; pcIdx < pcPoints.size(); ++pcIdx)
                {
                    pcPoints[pcIdx].x = pointsHullBottom[pcIdx].x;
                    pcPoints[pcIdx].y = pointsHullBottom[pcIdx].y;
                }
            // }
        }
        else
        {
            //MAR fitting
            Cloud & tmpCloud = (*clusteredPoints[idx]);
            vector<Point2f> pointVec(tmpCloud.size());

            for (int i = 0; i < tmpCloud.size(); ++i)
            {
                pointVec[i].x = tmpCloud[i].x();
                pointVec[i].y = tmpCloud[i].y();
                // if (tmpCloud[i].z() < minZ)
                //     minZ = tmpCloud[i].z();
                // if (tmpCloud[i].z() > maxZ)
                //     maxZ = tmpCloud[i].z();
            }

            RotatedRect rectInfo = minAreaRect(pointVec);
            Point2f rectPoints[4]; 
            rectInfo.points(rectPoints);
            // covert points back to lidar coordinate
            for (int j = 0; j < 4; ++j)
            {
                pcPoints[j].x = rectPoints[j].x;
                pcPoints[j].y = rectPoints[j].y;
            }
        }
        
        bool saveIt = ruleBasedFilter(pcPoints, 
        clusteredPoints[idx]->maxZ - clusteredPoints[idx]->minZ, 
        clusteredPoints[idx]->size());
        
        // 不进行保存 bbox
        if (!saveIt)
            continue;   

        Cloud::Ptr oneBboxPtr(new Cloud);
        for(int pclH = 0; pclH < 2; pclH++)
        {
            // 底面四个点,上面四个点
            for(int pclP = 0; pclP < 4; pclP++)
            {
                point o;
                o.x() = pcPoints[pclP].x;
                o.y() = pcPoints[pclP].y;
                if(pclH == 0) 
                    o.z() = clusteredPoints[idx]->minZ;//车体坐标系下点云,地面高度估计为0.1m
                    // o.z() = -1.73;
                else 
                    o.z() = clusteredPoints[idx]->maxZ;
                    // o.z() = 2;
                oneBboxPtr->emplace_back(o);
            }
        }
        bbPoints.emplace_back(oneBboxPtr);     
    }
}
