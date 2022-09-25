#include <iostream>
#include <array>
#include <atomic>
#include <vector>

#include "param.h"
#include <thread>
#include "bin.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "cloud.h"
#include "pixel_coords.h"

#include <unordered_map>
#include <mutex>
#include <thread>

typedef std::vector<std::vector<int>> mapGrid;
typedef Cloud PointCloud;
// 聚类方法一
class cluster
{
public:
    params params_;
    cluster();
    cluster(const float & roiM,
            const int & numGrid,
            const int & numCluster,
            const int & kernelSize,
            PointCloud & input,
            const params & inputParam = params()
            );

    void mapCloud2Grid();
    void mapCloud2GridThread(const unsigned int & start_idx,
                             const unsigned int & end_idx,
                             std::mutex * RectVecMutex);

    void findComponent();
    void componentClustering();
    void search(int Row, int Col);

    inline int getNumCluster(){return numCluster;}
    void getClusterImg(cv::Mat & img);

    ///////////////////  use pcl ///////////////////////////
    // Ptr 是智能指针， 应该不需要自己释放
    void makeClusteredCloud(PointCloud & clusterCloud, std::vector<Cloud::Ptr> & clusters);

    void insertCubeVec();
    inline std::vector<Rect2D> getRectVec() const {return rect2DVec;}

    void getLShapePoints(const std::vector<Cloud::Ptr> & clusters, 
                         Cloud::Ptr & points, 
                         const int & debugID,
                         const float & lShpaeHorizonResolution);
    
    size_t ColFromAngle(float angle_cols);
    // 根据每个对象， 平均分配多少份
    size_t ColFromAngle(float angle_cols, 
                        float minAngle, 
                        float maxAngle, 
                        const size_t & numSegment = 20,
                        const bool & debug = false);

private:
    int numCluster;
    int numGrid;
    // int roiM;
    float roiM;
    int kernelSize;
    mapGrid grid;
    std::atomic<int> hasPoint;
    PointCloud cloud;
    std::vector<int> clusterVecID;

    std::vector<Rect2D> rect2DVec;
    // float2DVec cubeVec;
    // std::vector<std::pair<float, float>> cubeVec;

    std::vector<float> _col_angles;
};


// 聚类方法二
class depth_clustering
{
public:
    enum class Direction {HORIZONTAL, VERTICAL};
    depth_clustering(const PointCloud & cloud, bool filter = false, float angle_threshold = 10);

    cv::Mat getVisualizeDepthImage();

    void createDepthImage();
    // 填充需要查找的表
    void fillVector();

    size_t RowFromAngle(float angle_rows);
    size_t ColFromAngle(float angle_cols);

    int mapToColor(float val);

    // 填充一些必要的计算
    void FillCosSin();

    cv::Mat CreateAngleImage(const cv::Mat & depth_image);
    
    // 修补深度图 深度间隔超过 depth_threshold 的将不再修补 step 是修补 kernel
    cv::Mat RepairDepth(const cv::Mat& no_ground_image, int step,
                                    float depth_threshold);
                                    
    void depthCluster();

    cv::Mat ApplySavitskyGolaySmoothing(const cv::Mat & image, int window_size);
    cv::Mat GetSavitskyGolayKernel(int window_size) const;
    cv::Mat visualzieDiffAngleImage();
    // cv::Mat ZeroOutGroundBFS(const cv::Mat & image,
    //                          const cv::Mat & angle_image,
    //                          const float & threshold,
    //                          int kernel_size) const;
    void LabelOneComponent(uint16_t label, const PixelCoord & start, const cv::Mat & angle_image);
    cv::Mat GetUniformKernel(int window_size, int type) const;

    void ComputeLabels();

    void LabelOneComponent(uint16_t label, const PixelCoord & start);
    int16_t WrapCols(int16_t col) const; 

    float DiffAt(const PixelCoord & from, const PixelCoord & to) const;

    cv::Mat visSegmentImage();
    cv::Mat visAngleImage();

    // 为点云打上标签
    void LabelCloud(Cloud & cloud);

    inline int getNumCluster() const {return numCluster;}


private:
    params _params;
    PointCloud _cloud;
    cv::Mat show_depth_image;

    std::vector<float> _row_angles;
    std::vector<float> _col_angles;    

    bool _filter;
    // 存储距离的
    cv::Mat _depthImage;

    // 避免二次计算， 存储映射过程
    // std::unordered_multimap<uint16_t, uint16_t> imageToPointID;    
    std::vector<std::pair<int, int>> pointToImageCoord;

    // 存储 Row 和 col 的各种值 避免重复计算
    std::vector<float> _row_angles_sines;
    std::vector<float> _row_angles_cosines;

    // 需要提前计算的变量

    // std::vector<float> _row_alphas;
    // std::vector<float> _col_alphas;
    // 这里我们有自己的固定的角度差
    // 水平方向为 _params.vehicle_step
    // 垂直方向为 _params.horizontal_step
    // 标记好的图像
    cv::Mat _label_image;
    cv::Mat _bata_image;

    float sinRowAlpha;
    float cosRowAlpha;
    float sinColAlpha;
    float cosColAlpha;

    // 四临域
    std::array<PixelCoord, 4> Neighborhood;
    float _angle_threshold;

    uint16_t numCluster;
};
