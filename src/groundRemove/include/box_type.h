#ifndef BOX_TYPE_H_
#define BOX_TYPE_H_
#include <iostream>
#include <string>
#include <vector>

struct Vertex{
  float x;
  float y;
  Vertex(){}
  Vertex(float a, float b)
  {
    x = a;
    y = b;
  }
  bool operator <(const Vertex &p) const {
    return x < p.x || (x == p.x && y < p.y);
  }

  Vertex operator +(const Vertex &p) const{
    Vertex vertex;
    vertex.x = x + p.x;
    vertex.y = y + p.y;
    return vertex;
  }

  Vertex operator -(const Vertex &p) const{
    Vertex vertex;
    vertex.x = x - p.x;
    vertex.y = y - p.y;
    return vertex;
  }
  
  friend Vertex operator *(const float& k, const Vertex &p){
    Vertex vertex;
    vertex.x = k * p.x;
    vertex.y = k * p.y;
    return vertex;
  }

  friend std::ostream& operator<<(std::ostream& out, const Vertex &p){
    out<<"["<<p.x<<" "<<p.y<<"]";
    return out;
  }
};

// 棱柱类
class MyPrism{
public:
  std::vector<Vertex> vertices;
  float top;
  float bottom;
};

struct Obstacle{
  float x;
  float y;
  float orientation;
  float length;
  float width;
};

#endif