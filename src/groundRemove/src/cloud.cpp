#include "cloud.h"

// explicit point(const point & pt)
// {
//     _point(pt.x(), pt.y(), pt.z());
//     _intensity = pt.i();
// }

void point::operator=(const point & other)
{
    _point = other.AsEigenVector();
    _intensity = other.i();
}

void point::operator=(const Eigen::Vector3f& other) 
{
    _point = other;
}

bool point::operator==(const point & other) const
{
     return x() == other.x() && y() == other.y() &&
         z() == other.z() && i() == other.i();
}

//  bounding box with center and orientation
BBox::BBox(const point & x1,
            const point & x2,
            const point & x3,
            const point & x4)
{
    points[0] = x1;
    points[1] = x2;
    points[2] = x3;
    points[3] = x4;
    // 中心点
    pose.position.x = 0.5 * (x1.x() + x3.x());
    pose.position.y = 0.5 * (x1.y() + x3.y());
    pose.position.z = 0.0;

    // 角度
    pose.yaw = std::atan2((x2.y() - x3.y()), (x2.x() - x3.x()));

    // 维度
    dimensions.x = sqrt((x1.x() - x2.x()) * (x1.x() - x2.x()) + (x1.y() - x2.y()) * (x1.y() - x2.y()));
    dimensions.y = sqrt((x3.x() - x2.x()) * (x3.x() - x2.x()) + (x3.y() - x2.y()) * (x3.y() - x2.y()));
    dimensions.z = 0.0;
}

BBox::BBox(const std::vector<point> & bbox)
{
    // ------------------------------------------------------------------------
    points[0] = bbox[0];
    points[1] = bbox[1];
    points[2] = bbox[2];
    points[3] = bbox[3];

    // 中心点
    pose.position.x = 0.5 * (points[0].x() + points[2].x());
    pose.position.y = 0.5 * (points[0].y() + points[2].y());
    pose.position.z = 0.0;

    // 角度
    pose.yaw = std::atan2((points[1].y() - points[2].y()), (points[1].x() - points[2].x()));

    // 维度
    dimensions.x = sqrt((points[0].x() - points[1].x()) * (points[0].x() - points[1].x()) + 
                (points[0].y() - points[1].y()) * (points[0].y() - points[1].y()));
    dimensions.y = sqrt((points[2].x() - points[1].x()) * (points[2].x() - points[1].x()) + 
                (points[2].y() - points[1].y()) * (points[2].y() - points[1].y()));
    dimensions.z = 0.0;
}

void BBox::updateCenterAndYaw()
{
     // 中心点
    pose.position.x = 0.5 * (points[0].x() + points[2].x());
    pose.position.y = 0.5 * (points[0].y() + points[2].y());
    pose.position.z = 0.0;

    // 角度
    // pose.yaw = (points[0].y() - points[1].y()) / (points[0].x() -points[1].x() + 1e-6);
    pose.yaw = std::atan2((points[1].y() - points[2].y()), (points[1].x() - points[2].x()));

    dimensions.z = 0.0;
}

// 获取 ref 点
point BBox::getRefPoint() const
{
    // fprintf(stderr, "refIdx, refIdx / 10, refIdx % 10 : %d, %d, %d, \n", 
    //             refIdx, refIdx / 10, refIdx % 10);
    // 角点
    if (refIdx < 4)  // egg 2
    {
        return points[refIdx];
    } // 边点
    else if (refIdx > 9 && refIdx < 99) // egg 21
    {
        point firstPt = points[refIdx % 10];
        point secondPt = points[refIdx / 10];
        return (firstPt + secondPt) * 0.5;
    } // 参考边
    else // egg 210
    {
        // 暂时返回该边的中点
        point firstPt = points[refIdx / 100];
        point secondPt = points[refIdx / 10 % 10];
        return (firstPt + secondPt) * 0.5;
    }    
}

void BBox::setRp() {
        // fprintf(stderr, "refIdx, refIdx / 10, refIdx % 10 : %d, %d, %d, \n", 
    //             refIdx, refIdx / 10, refIdx % 10);
    // 角点
    if (refIdx < 4)  // egg 2
    {
        rp = points[refIdx];
    } // 边点
    else if (refIdx > 9 && refIdx < 99) // egg 21
    {
        point firstPt = points[refIdx % 10];
        point secondPt = points[refIdx / 10];
        rp = ((firstPt + secondPt) * 0.5);
    } // 参考边
    else // egg 210
    {
        // 暂时返回该边的中点
        point firstPt = points[refIdx / 100];
        point secondPt = points[refIdx / 10 % 10];
        rp = ((firstPt + secondPt) * 0.5);
    }   
}