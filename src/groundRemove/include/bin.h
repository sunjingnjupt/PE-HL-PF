#ifndef BIN_H_
#define BIN_H_

#include <atomic>
#include <stdio.h>
#include "cloud.h"
#include "math.h"
#include <iostream>

struct PointXYZ
{
    PointXYZ();
    PointXYZ(double x, double y, double z):x(x), y(y), z(z){}

    double x, y, z;
};

class Bin
{

public:
    struct MinZPoint
    {
        MinZPoint():z(0), d(0){}
        MinZPoint(const double& d, const double& z) : z(z), d(d) {}

        // friend Bin;
        bool operator==(const MinZPoint & comp)
        {
            return comp.z == z && comp.d == d;
        }

        void print(){fprintf(stderr, "(%f, %f) ", d, z);}
        double z, d;
    };
        // inline bool hasPoint(){return has_point_;}

public:
    Bin();
    Bin(const Bin& bin);

    void addPoint(const PointXYZ & pointxyz, const int & ptID);
    void addPoint(const double &d, const double &z, const int & ptID);

    MinZPoint getMinZPoint();

    // inline bool hasPoint(){return mzpoint.hasPoint();}
    inline bool hasPoint() {return has_point;}

    // Bin & operator=(const Bin & bin);

    void print();

    inline double getMinZ(){double minZ = min_z;return minZ;}
    // inline double getMinD(){double minD = min_z_range;return min_z_range;}
    inline double getMinD(){return min_z_range;}
    inline void updateHeight(const double & z){height = z;}
    inline void updateMinZ(const double & z){min_z = z;}
    inline void updateSmoothed(const double & s){smoothed = s;}
    inline void updateSmoothedZ(const double & s){min_z = s;}
    inline double getHeight(){return height;}
    inline void updateHDiff(const double & h){hDiff = h;}
    inline double getSmoothed() const {return smoothed;}
    inline double getHDiff() const {return hDiff;}
    inline void updateGround(){isGround = true;hGround = height;}
    // 第二个版本的跟新是否是地面点
    inline void updateGround2(){isGround = true;hGround = min_z;}
    inline bool isThisGround()const {return isGround;}
    inline double getHGround()const {return hGround;}
public:
    std::atomic<int> pointID;
private:
    std::atomic<bool> has_point;
    std::atomic<double> min_z;
    std::atomic<double> min_z_range;

    // 后面添加的s
    // double z;
    // double h;
    // double m;
    // double h_diff;
    // bool isGround;
    // double h_ground;

    float smoothed;
    float height;
    float hDiff;
    float hGround;
    //float minZ;  min_z;
    std::atomic<bool> isGround;
    // std::atomic<int> pointID;
};

#endif
