#ifndef CONVEX_HULL_H_
#define CONVEX_HULL_H_
#include <algorithm>
#include <vector>
#include <ctime>
#include "opencv2/core/core.hpp"
// #include <opencv2/core/eigen.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include "box_type.h"
#include "cloud.h"


using namespace std;

vector<Vertex> generatePoints(const int num);

class ConvexHull
{
public:
    std::vector<Vertex> vertices_;
    ConvexHull(vector<Vertex> P);
    vector<Vertex> toRec();
    vector<Vertex> toRec1();
    vector<Vertex> Caliper();
    // Vertex getMedianPoint();

private:
    std::vector<Vertex> origin_points_;
    float cross(const Vertex &O, const Vertex &A, const Vertex &B);
    float getDistance(const Vertex &A, const Vertex &B);
    Vertex foot(const Vertex &A, const Vertex &B, const Vertex &O);
    float minDisToRec(std::vector<Vertex> rect, Vertex point);
    vector<Vertex> fitRec(float k);
    float getArea(std::vector<Vertex> rect);
public:
    cv::Mat direction(vector<Vertex> points_input);
};
#endif