#ifndef SRC_DRAWABLELINE_H
#define SRC_DRAWABLELINE_H

#include "drawable.h"
#include <memory>

// 绘制点云
class DrawableLine : public Drawable
{
public:
    using Ptr = std::shared_ptr<DrawableLine>;
    explicit DrawableLine(const Cloud::ConstPtr& cloud,
                            const Eigen::Vector3f& color = Eigen::Vector3f::Ones())
            : _cloud_ptr{cloud}, _color{color}{}
    
    void Draw() const override;

    static DrawableLine::Ptr FromCloud(
      const Cloud::ConstPtr& cloud,
      const Eigen::Vector3f& color = Eigen::Vector3f::Ones());

    ~DrawableLine() override {}

private:
    Cloud::ConstPtr _cloud_ptr = nullptr;
    Eigen::Vector3f _color = Eigen::Vector3f::Ones();
};
#endif