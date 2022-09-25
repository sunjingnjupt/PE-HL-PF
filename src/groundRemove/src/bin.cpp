#include "bin.h"

#include <limits> // max , min val


// Bin::Bin() : min_z(std::numeric_limits<double>::max()), has_point(false) {}
Bin::Bin()
{
    std::atomic_init(&has_point, false);
    // min_z = std::numeric_limits<double>::max();
    std::atomic_init(&min_z, std::numeric_limits<double>::max());
    std::atomic_init(&pointID, -1);
    isGround = false;
}


Bin::Bin(const Bin& bin) 
{
    std::atomic_init(&min_z, std::numeric_limits<double>::max());
    std::atomic_init(&has_point, false);
    std::atomic_init(&isGround, false);
}

void Bin::addPoint(const PointXYZ & pointxyz, const int & ptID)
{
    has_point = true;
    const double d = sqrt(pointxyz.x * pointxyz.z + pointxyz.y * pointxyz.y);
    addPoint(d, pointxyz.z, ptID);
}

void Bin::addPoint(const double &d, const double &z, const int & ptID)
{
    // if(!has_point)
    // {
    //     std::cout << "before Bin::addPoint() " << has_point << std::endl;
    //     has_point = true;
    //     std::cout << "after Bin::addPoint() " << has_point << std::endl;
    // }

    has_point = true;
    // 只有在比但前 bin 的 min_z 小时， 才更新 ptID 和 其他信息
    // pointID = ptID;
    if (z < min_z)
    {
        min_z = z;
        min_z_range = d;
        pointID = ptID;
    }
    
}

Bin::MinZPoint Bin::getMinZPoint()
{
    MinZPoint pt;
    
    if (has_point)
    {
        pt.z = min_z;
        pt.d = min_z_range;
    }
    
    return pt;
}

// Bin& Bin::operator=(const Bin & bin)
// {
//     has_point = static_cast<int>(bin.has_point);
//     // has_point = bin.has_point;
//     min_z.store(bin.min_z);
//     min_z_range.store(bin.min_z_range);
//     return *this;
// }
void Bin::print()
{
    std::cout << "(" << min_z_range << "," << min_z << ")\n";
}




