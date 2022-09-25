// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
#include <algorithm>
#include <vector>
#include <limits>
#include <ctime>
#include <opencv2/core/core.hpp>
// #include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "convex_hull.h"
#include "box_type.h"
using namespace std;


vector<Vertex> generatePoints(const int num)
{
  vector<Vertex> points;
  srand((unsigned)time(NULL));
  for(int i = 0; i < num; i++)
  {
    Vertex point;
    point.x = 2 + 1*(rand()/double(RAND_MAX)+0.2);
    point.y = -5 + 2*(rand()/double(RAND_MAX)+0.2);
    points.push_back(point);
  }
  for(int i = 0; i < num; i++)
  {
    Vertex point;
    point.x = -2+2*(rand()/double(RAND_MAX)+0.2);
    point.y = 3+1*(rand()/double(RAND_MAX)+0.2);
    points.push_back(point);
  }
  return points;
}

// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
ConvexHull::ConvexHull(vector<Vertex> P)
{
  origin_points_ = P;
  size_t n = P.size(), k = 0;
  if (n <= 3) 
  {
    vertices_.assign(P.begin(), P.end());
    return;
  }

  vertices_.resize(2*n);
  // vector<Vertex> H(2*n);

  // Sort points lexicographically
  sort(P.begin(), P.end());

  // Build lower hull
  for (size_t i = 0; i < n; ++i) 
  {
    while (k >= 2 && cross(vertices_[k-2], vertices_[k-1], P[i]) <= 0) k--;
    vertices_[k++] = P[i];
  }
  // Build upper hull
  for (size_t i = n-1, t = k+1; i > 0; --i) {
    while (k >= t && cross(vertices_[k-2], vertices_[k-1], P[i-1]) <= 0) k--;
    vertices_[k++] = P[i-1];
  }
  vertices_.resize(k-1);
}

// must be big enough to hold 2*max(|coordinate|)^2

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
float ConvexHull::cross(const Vertex &O, const Vertex &A, const Vertex &B)
{
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

float ConvexHull::getDistance(const Vertex &A, const Vertex &B)
{
  return sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y));
}

// O在AB上的垂足
Vertex ConvexHull::foot(const Vertex &A, const Vertex &B, const Vertex &O)
{
  float k = (O.x * (B.x - A.x) + O.y * (B.y - A.y))/
            (pow((B.x - A.x), 2) + pow((B.y - A.y), 2));
  return k * (B - A);
}

//用PCA计算主方向，即朝向
cv::Mat ConvexHull::direction(vector<Vertex> points_input)
{
  cv::Mat pcaSet(points_input.size(), 2, CV_32FC1, cv::Scalar::all(0));

  float* data = nullptr;
  for(size_t i = 0; i < points_input.size(); i++)
  {
    data = pcaSet.ptr<float>(i);
    *data++ = points_input[i].x;
    *data++ = points_input[i].y;
  }
  cv::PCA pca(pcaSet, cv::Mat(), CV_PCA_DATA_AS_ROW, 2);
  //参数依次为：原始数据；原始数据均值，输入空会自己计算；每行/列代表一个样本；保留多少特征值，默认全保留

  cv::Mat direction_vector = pca.eigenvectors;
  return direction_vector;
}
// 给定朝向，拟合矩形
vector<Vertex> ConvexHull::fitRec(float k)
{
  std::vector<Vertex> rect;
  float k_1 = -1 / k;

  // 斜率为无穷时，分部计算
  if(k_1 != -std::numeric_limits<float>::infinity() && 
     k_1 != std::numeric_limits<float>::infinity() &&
     k !=  -std::numeric_limits<float>::infinity() &&
     k !=  std::numeric_limits<float>::infinity())
  {
    float b_1, b_2, b_3, b_4;
    b_1 = vertices_[0].y - k * vertices_[0].x;
    b_2 = vertices_[0].y - k * vertices_[0].x;
    b_3 = vertices_[0].y - k_1 * vertices_[0].x;
    b_4 = vertices_[0].y - k_1 * vertices_[0].x;

    for(size_t i = 0; i < vertices_.size(); i++)
    {
      //点在直线的左边
      if(k * vertices_[i].x + b_1 <= vertices_[i].y)
      {
        b_1 = vertices_[i].y - k * vertices_[i].x;
      }

      if(k * vertices_[i].x + b_2 >= vertices_[i].y)
      {
        b_2 = vertices_[i].y - k * vertices_[i].x;
      }

      if(k_1 * vertices_[i].x + b_3 <= vertices_[i].y)
      {
        b_3 = vertices_[i].y - k_1 * vertices_[i].x;
      }

      if(k_1 * vertices_[i].x + b_4 >= vertices_[i].y)
      {
        b_4 = vertices_[i].y - k_1 * vertices_[i].x;
      }
    }
    Vertex vertex;
    vertex.x = (b_3 - b_1) / (k - k_1);
    vertex.y = vertex.x * k + b_1;
    rect.push_back(vertex);

    vertex.x = (b_2 - b_3) / (k_1 - k);
    vertex.y = vertex.x * k + b_2;
    rect.push_back(vertex);

    vertex.x = (b_4 - b_2) / (k - k_1);
    vertex.y = vertex.x * k + b_2;
    rect.push_back(vertex);

    vertex.x = (b_1 - b_4) / (k_1 - k);
    vertex.y = vertex.x * k + b_1;
    rect.push_back(vertex);
  }
  else // 对于与坐标系平行的矩形，直接计算x、y轴的最大最小范围
  {
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    for(size_t i = 0; i < vertices_.size(); i++)
    {
      max_x = (vertices_[i].x > max_x)?vertices_[i].x:max_x;
      max_y = (vertices_[i].y > max_y)?vertices_[i].y:max_y;
      min_x = (vertices_[i].x < min_x)?vertices_[i].x:min_x;
      min_y = (vertices_[i].y < min_y)?vertices_[i].y:min_y;
    }
    Vertex vertex;
    vertex.x = max_x;
    vertex.y = max_y;
    rect.push_back(vertex);

    vertex.x = max_x;
    vertex.y = min_y;
    rect.push_back(vertex);

    vertex.x = min_x;
    vertex.y = min_y;
    rect.push_back(vertex);

    vertex.x = min_x;
    vertex.y = max_y;
    rect.push_back(vertex);
  }
  
  return rect;
}

// 根据PCA得到的方向绘制矩形
vector<Vertex> ConvexHull::toRec()
{
  std::vector<Vertex> rect;
  cv::Mat direct= direction(vertices_);
  float k = direct.at<float>(0,1)/direct.at<float>(0,0);
  rect = fitRec(k);
  return rect;
}

float ConvexHull::getArea(std::vector<Vertex> rect)
{
  return abs(cross(rect[1], rect[0], rect[2]));
}

// 得到面积最小fitting box
vector<Vertex> ConvexHull::Caliper()
{
  std::vector<Vertex> rect;
  float area = std::numeric_limits<float>::infinity();
  int len = vertices_.size();
  for(int i = 0; i < len; i++)
  {
    //计算斜率
    float k = (vertices_[(i+1) % len].y - vertices_[(i) % len].y)/
              (vertices_[(i+1) % len].x - vertices_[(i) % len].x);

    std::vector<Vertex> rect_temp = fitRec(k);
    float area_temp = getArea(rect_temp);
    if(area > area_temp)
    {
      area = area_temp;
      rect = rect_temp;
    }
  }
  return rect;
}


// Vertex ConvexHull::getMedianPoint()
// {
//   int n = origin_points_.size();
//   float w[n];
//   Vertex median;

//   for(int i = 0; i < 2; i++)
//   {
//     if(i == 0)
//     {
//       for(int j = 0; j < n; j++)
//       {
//         w[j] = 1;
//       }
//     }
//     else
//     {
//       for(int j = 0; j < n; j++)
//       {
//         w[j] =abs(median.x - origin_points_[j].x) + abs(median.y - origin_points_[j].y);
//       }
//     }

//     std::vector<float> pts_x;
//     std::vector<float> pts_y;
//     for(int j = 0; j < n; j++)
//     {
//       float pt_x_new = w[j] * origin_points_[j].x;
//       float pt_y_new = w[j] * origin_points_[j].y;
//       pts_x.push_back(pt_x_new);
//       pts_y.push_back(pt_y_new);
//     }
//     std::vector<float> pts_x_copy = pts_x;
//     std::vector<float> pts_y_copy = pts_y;
//     sort(pts_x.begin(), pts_x.end());
//     sort(pts_y.begin(), pts_y.end());

//     std::vector<float>::iterator it_x = find(pts_x_copy.begin(), pts_x_copy.end(), pts_x[n/2]);
//     std::vector<float>::iterator it_y = find(pts_y_copy.begin(), pts_y_copy.end(), pts_y[n/2]);

//     // median.x = pts_x[n/2] / w[it_x - pts_x_copy.begin()];
//     // median.y = pts_y[n/2] / w[it_y - pts_y_copy.begin()];
//     median.x = pts_x[n/2] / w[it_x - pts_x_copy.begin()];
//     median.y = pts_y[n/2] / w[it_y - pts_y_copy.begin()];
//   }
//   return median;
// }

float ConvexHull::minDisToRec(std::vector<Vertex> rect, Vertex point)
{
  float min_distance = std::numeric_limits<float>::infinity();
  for(int i = 0; i < 4; i++)
  {
    float dis1 = abs(cross(rect[i%4], rect[(i+1)%4], point)) / getDistance(rect[i%4], rect[(i+1)%4]);
    min_distance = (min_distance>dis1) ? dis1:min_distance;
  }
  return min_distance;
}
// 公式推导详见
// An Orientation Corrected Bounding Box Fit Based on the Convex Hull under Real Time Constraints
// Auther: Benjamin Naujoks and Hans-Joachim Wuensche
// 这个方法优先
vector<Vertex> ConvexHull::toRec1()
{
  static float lambda = 0.01;
  std::vector<Vertex> rect = Caliper();

    //   fprintf(stderr, "bbox -------\n");
    //   for (size_t idx = 0; idx < rect.size(); ++idx)
    //     fprintf(stderr, "[%f, %f]\n", rect[idx].x, rect[idx].y);

  Vertex higher_point, lower_point, third_point, median_point;
  
  //计算最高点和最低点
  float upper_min = std::numeric_limits<float>::infinity();
  float lower_min = std::numeric_limits<float>::infinity();;

  for(size_t i = 0; i < vertices_.size(); i++)
  {
    float upper_dis = abs(cross(vertices_[i], rect[0], rect[1]))/getDistance(rect[0], rect[1]);
    float lower_dis = abs(cross(vertices_[i], rect[2], rect[3]))/getDistance(rect[2], rect[3]);

    if(upper_dis < upper_min)
    {
      upper_min = upper_dis;
      higher_point = vertices_[i];
    }

    if(lower_dis < lower_min)
    {
      lower_min = lower_dis;
      lower_point = vertices_[i];
    }
  }

  //点到直线的距离减去垂足到两端点的最短距离和缩放系数的乘积
  float distance = 0;
  for(size_t i = 0; i < vertices_.size(); i++)
  {
    float min_cut = min(getDistance(foot(lower_point, higher_point, vertices_[i]) , lower_point),
                        getDistance(foot(lower_point, higher_point, vertices_[i]) , higher_point));

    float foot_distance = abs(cross(lower_point, higher_point, vertices_[i]))/
                      getDistance(lower_point, higher_point);
                      
    float dis_temp = foot_distance -  lambda * min_cut;

    if(dis_temp > distance)
    {
      third_point = vertices_[i];
      distance = dis_temp;
    }
  }


  // median_point = getMedianPoint();

  // float criteria_1 = abs(cross(lower_point, higher_point, median_point))/
  //                     getDistance(lower_point, higher_point);
  // float criteria_2 = abs(cross(third_point, higher_point, median_point))/
  //                   getDistance(third_point, higher_point);
  // float criteria_3 = abs(cross(third_point, lower_point, median_point))/
  //                 getDistance(third_point, lower_point);

  // if(criteria_1 <= criteria_2 && criteria_1 <= criteria_3)
  // {
  //   std::cout<<"1"<<std::endl;
  //   return fitRec((higher_point.y-lower_point.y)/(higher_point.x-lower_point.x));
  // }
  // else if(criteria_2 <=  criteria_1 && criteria_2 <= criteria_3)
  // {
  //   std::cout<<"2"<<std::endl;
   
  //   return fitRec((higher_point.y-third_point.y)/(higher_point.x-third_point.x));
  // }
  // else
  // {
  //   std::cout<<"3"<<std::endl;
  //   return fitRec((lower_point.y-third_point.y)/(lower_point.x-third_point.x));
  // }

  std::vector<Vertex> rect1 = fitRec((higher_point.y-lower_point.y)/(higher_point.x-lower_point.x));
  std::vector<Vertex> rect2 = fitRec((higher_point.y-third_point.y)/(higher_point.x-third_point.x));
  std::vector<Vertex> rect3 = fitRec((lower_point.y-third_point.y)/(lower_point.x-third_point.x));

  float dis_sum1 = 0;
  float dis_sum2 = 0;
  float dis_sum3 = 0;
  for(size_t i = 0; i < origin_points_.size(); i++)
  {
    dis_sum1 += minDisToRec(rect1, origin_points_[i]);
    dis_sum2 += minDisToRec(rect2, origin_points_[i]);
    dis_sum3 += minDisToRec(rect3, origin_points_[i]);
  }

  if(dis_sum1 < dis_sum2 && dis_sum1 < dis_sum3)
  {
    return rect1;
  }
  else if(dis_sum2 < dis_sum1 && dis_sum2 < dis_sum3)
  {
    return rect2;
  }
  else
  {
    return rect3;
  }
}


/*
int main()
{
    vector<Vertex> points_generated = generatePoints(100);
    ConvexHull* convex_hull (new ConvexHull(points_generated));
    // cv::Rect frame(0, 0, 800, 800);
    // // cv::Mat intensity_img = cv::Mat::zeros(frame.size(), CV_8UC3);
    // cv::Mat img = cv::Mat::zeros(frame.size(), CV_8UC3);

    // for(int i = 0; i < points_generated.size(); i++)
    // {
    //     img.at<cv::Vec3b>(cv::Point(points_generated[i].x, points_generated[i].y)) = cv::Vec3b(0, 0, 255);
    // }
    // cv::imshow("origin", img);

    vector<Vertex> points_hull = convex_hull->vertices_;
    points_hull = convex_hull->toRec1();

    fprintf(stderr, "bbox -------\n");
    for (size_t idx = 0; idx < points_hull.size(); ++idx)
        fprintf(stderr, "[%f, %f]\n", points_hull[idx].x, points_hull[idx].y);

    // for(int i = 0; i < points_hull.size()-1; i++)
    // {
    // cv::line(img, cv::Point(points_hull[i].x, points_hull[i].y),
    //                 cv::Point(points_hull[i+1].x, points_hull[i+1].y),
    //                 cv::Scalar(255,0,0),3);
    // }
    // cv::line(img, cv::Point((points_hull.end()-1)->x, (points_hull.end()-1)->y),
    //                 cv::Point(points_hull.begin()->x, points_hull.begin()->y),
    //                 cv::Scalar(255,0,0),3);

    //   Vertex point = convex_hull->getMedianPoint();
    //   img.at<cv::Vec3b>(cv::Point(point.x, point.y)) = cv::Vec3b(255, 0, 0);
    // cv::imshow("line", img);
    cv::waitKey(0);
}
*/