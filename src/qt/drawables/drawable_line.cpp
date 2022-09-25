#include "drawable_line.h"

void DrawableLine::Draw() const
{
    // fprintf(stderr, "DrawableCloud::Draw()\n");
    if (!_cloud_ptr)
    {
        throw std::runtime_error("DrawableLine has no cloud to draw.");
    }

    // fprintf(stderr, "DrawableCloud::Draw() after _cloud_ptr check\n");

    glPushMatrix();
    glBegin(GL_LINES);
    glColor3f(_color[0], _color[1], _color[2]);
    for (const auto & point : _cloud_ptr->points())
    {
        auto real_point = point.AsEigenVector();
        glVertex3f(real_point.x(), real_point.y(), real_point.z());
    }
    glEnd();
    glPopMatrix();    
}

DrawableLine::Ptr DrawableLine::FromCloud(const Cloud::ConstPtr& cloud,
                                            const Eigen::Vector3f& color)
{
    return std::make_shared<DrawableLine>(DrawableLine(cloud, color));
}
