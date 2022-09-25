#ifndef SRC_QT_UTILS_CLOUD_H
#define SRC_QT_UTILS_CLOUD_H
#include <iostream>

#include <algorithm>
#include <list>
#include <vector>
#include <Eigen/Core>
#include "Eigen/Dense"

#include <memory>
#include <opencv2/core/core.hpp>
#include <string>
#include <array>

using std::string;

typedef std::pair<float, float> float2D;
typedef std::vector<float2D> float2DVec;
using cv::Point2f;

enum pointType : int
{
    STANDARD = 0, // 普通点
    TRACK = 1,    // 跟踪点
    MARK = 2,     // 标记点
    TRACKHIS = 3, // 历史跟踪 ref 点
};
enum shapeType : int
{
    SYMETRIC = 0, // 对称
    LSHAPE = 1,
    ISHAPE = 2,
    MINAREA = 3,
};

// 聚类显示的 2D 矩形
class Rect2D
{
public:
    explicit Rect2D(float x, float y, float hight):_x(x), _y(y), _hight(hight){}
    inline float x() {return _x;}
    
    inline float y() {return _y;}

    inline float hight() {return _hight;}

    inline void setSize(const float & size){_size = size;}

    inline float getSize() const{return _size;}

private:
    float _x;
    float _y;

    // hight 为离地高度， 暂时不确定使用最高还是最低高度
    float _hight;

    // 矩形大小
    float _size;
};


class point{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    point() {_point = Eigen::Vector3f::Zero();_intensity = 0;}
    explicit point(float x, float y, float z): _point(x, y, z) {}
    explicit point(float x, float y, float z, pointType ptType):_point(x, y, z), ptType(ptType){}
    // explicit point(const point & pt);

    virtual ~point(){}

    // 不需要改值的声明
    inline float x() const { return _point.x(); }
    inline float y() const { return _point.y(); }
    inline float z() const { return _point.z(); }
    inline float i() const { return _intensity; }

    // 需要改值的申明
    inline float& x() { return _point.x(); }
    inline float& y() { return _point.y(); }
    inline float& z() { return _point.z(); }
    inline float& i() { return _intensity; }
    
    inline float dist2D(){return sqrt(_point.x() * _point.x() + _point.y() * _point.y());}
    // const 对象，如果编译器认为可能会改变函数内部的值， 那么就需要 const 修饰函数使用
    inline float dist2D() const {return sqrt(_point.x() * _point.x() + _point.y() * _point.y());}
    inline const Eigen::Vector3f& AsEigenVector() const {return _point; }
    inline Eigen::Vector3f& AsEigenVector() { return _point; }

    inline float tan2() {return atan2(_point.y(), _point.x());}

    void operator=(const point& other);
    void operator=(const Eigen::Vector3f& other);
    bool operator==(const point& other) const;
    inline point operator-(const point& other){
        return point(this->x() - other.x(), this->y() - other.y(), this->z() - other.z());}
    
    inline point operator-(const point& other) const {
        return point(this->x() - other.x(), this->y() - other.y(), this->z() - other.z());}
    inline point operator+(const point& other){
        return point(this->x() + other.x(), this->y() + other.y(), this->z() + other.z());}

    inline point operator+(const point& other) const{
        return point(this->x() + other.x(), this->y() + other.y(), this->z() + other.z());}
    inline point add(const point & other) const {
        return point(this->x() + other.x(), this->y() + other.y(), this->z() + other.z());}
    inline point operator*(const float & factor){
        return point(this->x() * factor, this->y() * factor, this->z() * factor);}
public:
    int classID = -1;
    float toSensor2D = 0.0f;
    float atan2Val = 0.0f;
    // 是否是 lShapePoint
    int isLShapePoint = 0;

    pointType ptType = pointType::STANDARD;
    

private:
    Eigen::Vector3f _point;
    float _intensity;
};

class Cloud
{
public:
    Cloud():minZ(999.0f), maxZ(-999.0f), maxAngle(-999.0f), minAngle(999){}
    // explicit Cloud(const Cloud & cloud) {}
    // explicit Cloud(Cloud & cloud) {}
    inline size_t size() const { return _points.size(); }
    inline bool empty() const { return _points.empty(); }
    //  这里需要验证一下， 问题很大， 直接导致分配的点个数为 0 的情况， 在第一次显示 trackBBox 时间出现问题
    inline void reserve(size_t size) { _points.reserve(size);}
    inline void resize(size_t size) { _points.resize(size);}
    inline void push_back(const point& pt) { _points.push_back(pt); }
    inline void emplace_back(const point& pt) {_points.emplace_back(pt);}
    inline point & operator[](int idx) {return _points[idx];}
    inline const point & operator[](int idx) const { return _points[idx];}

    inline const std::vector<point> & points() const {return _points;}
    
    // inline std::vector<point>::iterator begin() {return _points.begin();}
    // inline std::vector<point>::iterator end() {return _points.end();}
    inline std::vector<point>::iterator begin() {return _points.begin();}
    inline std::vector<point>::iterator end() {return _points.end();}

    // std::vector<point> _points;
    inline void clear(){_points.clear();}
    inline void swap(){std::vector<point>().swap(_points);}

    // 根据距离激光雷达的距离对点进行排序， 从近到远
    inline void sort(){std::sort(_points.begin(), _points.end(), 
        [](const point & a, const point & b){return a.toSensor2D < b.toSensor2D;});}
    
    inline void sortClockWise() {        // 按逆时针对 bbox 进行排序， 以供后续使用
        auto midPoint = (_points[0] + _points[2]) * 0.5;
		auto compFun = [midPoint](const point & p1, const point & p2) {return (p1 - midPoint).tan2() < (p2 - midPoint).tan2();};
		std::sort(_points.begin(), _points.end(), compFun);}

    typedef std::shared_ptr<Cloud> Ptr;
    typedef std::shared_ptr<const Cloud> ConstPtr;
private:
    std::vector<point> _points;
public:
    float minZ;
    float maxZ;

    // 最大与最小角度
    float maxAngle;
    float minAngle;

    // 存储 L-shape 的俩个拐点的坐标的索引
    int minLPoint = -1;
    int maxLPoint = -1;
    // 拐角坐标索引
    int minOPoint = -1;
    // 处于最大点最小角度的边缘点， 
    // 其是否是观察完全的， 或者是阻塞的, 默认堵塞
    bool occlusionMin = true;
    bool occlusionMax = true;
    // 检测目标的 ID
    int detectID = -2;
    // 对称点的对数
    size_t numSymPoints = 0;
    size_t numNoneEmptyLShapePoint = 0;
    float SymPointPercent = 0.0f;
    bool isSymCloud = false;

    // 跟踪的类型
    int id = -1; 
    float velocity = 0.0f;
    float acceleration = 0.0f;
    float yaw = 0.0f;
    // 求出的 ref 点的索引
    int refIdx = -1;  
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
    // 为了跟踪而添加的属性
    shapeType shape = shapeType::LSHAPE;
    // 保存当前拟合的 bbox 的直线的斜率
    Point2f line = Point2f(0.0f, 0.0f);

    // bbox 的 ref 点的索引, 或者坐标
    float xRef = 0.0f;
    float yRef = 0.0f;
};

struct point3d
{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
};

struct Pose
{
    point3d position;
    float yaw = 0.0;
};


struct Velocity
{
    point3d linear;
    point3d angular;
};

struct Acceleration
{
    point3d linear;
    point3d angular;
};

class BBox
{
public:
    BBox(){}
    BBox(const point & x1,
         const point & x2,
         const point & x3,
         const point & x4);
    BBox(const std::vector<point> & bbox);
    ~BBox(){}
    inline point & operator[](const int & pointIdx){return points[pointIdx];}
    inline const point & operator[](const int & pointIdx) const {return points[pointIdx];}
    // golbel to local update yaw
    void updateCenterAndYaw();
    // fprintf(stderr, "pointIdx %d, points[pointIdx](%f, %f)\n", pointIdx, points[pointIdx].x(), points[pointIdx].y());
    point getRefPoint() const;
    // 设置 rp 点， 后续不需要多次装换
    void setRp();

public:
    Pose pose;
    std::array<point, 4> points;
    // std::vector<point> points;
    string label = "unknown";
    /*Behavior State of the Detected Object
    behavior_state # 
    FORWARD_STATE = 0, 
    STOPPING_STATE = 1, 
    BRANCH_LEFT_STATE = 2, 
    BRANCH_RIGHT_STATE = 3, 
    YIELDING_STATE = 4, 
    ACCELERATING_STATE = 5, 
    SLOWDOWN_STATE = 6
    */
    uint8_t behavior_state;
    uint32_t id;
    // velocity with angle and linear
    Velocity velocity;
    
    Acceleration acceleration;
    bool pose_reliable;
    bool velocity_reliable;
    bool acceleration_reliable;
    point3d dimensions;

    float angle = 0.0f; // 0 ~ 2 * pi 为 object msg 消息所定义
    float yaw = 0.0f;  // 0 ~ 2 * pi

    // 存储最高最低 z 值
    float minZ = 0.0f;
    float maxZ = 0.0f;

    // ref 点所在位置索引
    int refIdx = -1;
    // I L S MinArea
    shapeType shape = shapeType::LSHAPE;
    // isArrange
    point rp;
};

struct intToPoint
{
    intToPoint(const int & inputId, const point & inputPt):id(inputId), pt(inputPt){}
    int id = 0;
    point pt;
};

// 绘带方向的椭圆
enum EllipeType : int
{
    PREDICTE = 0, // 预测
    MEASURE = 1,  // 测量
    MERGE = 2, // kalman 合并后
};
struct rotateEllipse
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::MatrixXd state;
    Eigen::MatrixXd covariance;
    EllipeType type = EllipeType::MEASURE;
    rotateEllipse(Eigen::MatrixXd & s, 
        Eigen::MatrixXd & cov, 
        EllipeType typeIn):
        state(s), 
        covariance(cov), 
        type(typeIn){}

};
#endif

