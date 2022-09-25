#include "drawable_selectable_cloud.h"

DrawSelectAbleCloud::DrawSelectAbleCloud(const Cloud::ConstPtr& cloud,
                            const Eigen::Vector3f& color,
                            const GLfloat & pointSize): 
                            _cloud_ptr{cloud}, 
                            _color{color},
                            _pointSize(_pointSize)
{
    objects.clear();
    // 构造对象
    for (const auto & point : _cloud_ptr->points())
    {
        auto real_point = point.AsEigenVector();
        // 注意 make_shared 的用法
        auto obj = std::make_shared<Object>();
        obj->frame.setPosition(real_point.x(), real_point.y(), real_point.z());
        objects.push_back(obj);
    }
}

// void DrawSelectAbleCloud::DrawSeletable() const
// {
//     // 写入带名字的对象
//     for (size_t idx = 0; idx < _objects.size(); ++idx)
//     {
//         glPushName(idx);
//         _objects.at(idx)->draw();
//         glPopName();
//     }
// }

void DrawSelectAbleCloud::Draw() const
{
    // fprintf(stderr, "DrawableCloud::Draw()\n");
    if (!_cloud_ptr)
    {
        throw std::runtime_error("DrawableCloud has no cloud to draw.");
    }

    // fprintf(stderr, "DrawableCloud::Draw() after _cloud_ptr check\n");

    glPushMatrix();
    glPointSize(_pointSize);
    glBegin(GL_POINTS);
    glColor3f(_color[0], _color[1], _color[2]);    
    // fprintf(stderr, "there has about %ld points\n", _cloud_ptr->size());
    for (const auto & point : _cloud_ptr->points())
    {
        auto real_point = point.AsEigenVector();
        // fprintf(stderr, "(%f, %f, %f)\n", real_point.x(), real_point.y(), real_point.z());
        glVertex3f(real_point.x(), real_point.y(), real_point.z());
    }
    glEnd();
    glPopMatrix();    
}

DrawSelectAbleCloud::Ptr DrawSelectAbleCloud::FromCloud(const Cloud::ConstPtr& cloud,
                                            const Eigen::Vector3f& color,
                                            const GLfloat & pointSize)
{
    return std::make_shared<DrawSelectAbleCloud>(DrawSelectAbleCloud(cloud, color, pointSize));
}

// selected BBox
DrawSelectAbleBBox::DrawSelectAbleBBox(const std::vector<Cloud::Ptr> & posVec, 
                                       bool drawZAxis,
                                       const Eigen::Vector3f& color)
{
    rectPosVec.assign(posVec.begin(), posVec.end());
    _drawZAxis = drawZAxis;
    _color = color;
    objects.clear();
    // 构造对象
    for (const auto & rect : rectPosVec)
    {
        point centerPoint;
        for (int idx = 0; idx < 4; ++idx)
        {
            centerPoint.x() += (*rect)[idx].x();
            centerPoint.y() += (*rect)[idx].y();
            centerPoint.z() += (*rect)[idx].z();
        }
        // 注意 make_shared 的用法
        auto obj = std::make_shared<Object>();
        obj->frame.setPosition(centerPoint.x() / 4, centerPoint.y() / 4, centerPoint.z() / 4);
        objects.push_back(obj);
    }
}

void DrawSelectAbleBBox::Draw() const
{
    point centerPoint;
    glPushMatrix();
    glLineWidth(1.8f);
    for (int iBBox = 0; iBBox < rectPosVec.size(); ++iBBox)
    {
        Cloud bboxPt = (*rectPosVec[iBBox]);
        if (_drawZAxis)
        {
            // glColor3f(0.4112f, 0.412f, 0.412f);
            // glBegin(GL_QUAD_STRIP);
//            glColor3f(1.0f, 1.0f, 1.0f);
            glColor3f(_color.x(), _color.y(), _color.z());
            glBegin(GL_LINE_STRIP);
            // std::array<int, 4> bottom = {1, 2, 0, 3};
            std::array<int, 5> bottom = {0, 1, 2, 3, 0};
            for (int idx = 0; idx < 5; ++idx)
            {
                glVertex3f(bboxPt[bottom[idx]].x(), 
                            bboxPt[bottom[idx]].y(), 
                            bboxPt[bottom[idx]].z());
                centerPoint.x() += bboxPt[bottom[idx]].x();
                centerPoint.y() += bboxPt[bottom[idx]].y();
                centerPoint.z() += bboxPt[bottom[idx]].z();
            }
            glEnd();
        }
        else
        {
            // glColor3f(1.0f, 1.0f, 1.0f); 
            // glColor3f(0.0f, 0.0f, 0.0f);
            glColor3f(_color[0], _color[1], _color[2]);
            glBegin(GL_LINE_STRIP);
            std::array<int, 4> bottom = {0, 1, 2, 3};
            for (int idx = 0; idx < 4; ++idx)
            {
                // glVertex3f(bboxPt[bottom[idx]].x(), 
                //             bboxPt[bottom[idx]].y(), 
                //             bboxPt[bottom[idx]].z());
                glVertex3f(bboxPt[bottom[idx]].x(), 
                    bboxPt[bottom[idx]].y(), 
                    -1.72f);
                centerPoint.x() += bboxPt[bottom[idx]].x();
                centerPoint.y() += bboxPt[bottom[idx]].y();
                centerPoint.z() += bboxPt[bottom[idx]].z();
            }
            glVertex3f(bboxPt[0].x(), bboxPt[0].y(), -1.721f);
            glEnd();
        }
        
        // 绘制中心点
        // glColor3f(1.0f, 1.0f, 1.0f);    
        // glBegin(GL_POINT);
        // glVertex3f(centerPoint.x() / 4, centerPoint.y() / 4, centerPoint.z() / 4 + 0.2);
        // glEnd();
        // --------
        if (_drawZAxis)
        {
            // glColor3f(0.80f, 0.745f, 0.448f);     
//            glColor3f(1.0f, 1.0f, 1.0f);
            glColor3f(_color.x(), _color.y(), _color.z());
            glBegin(GL_LINE_STRIP);
            for (int idx = 4; idx < 8;++idx)
            {
                glVertex3f(bboxPt[idx].x(), bboxPt[idx].y(), bboxPt[idx].z());
            } 
            glVertex3f(bboxPt[4].x(), bboxPt[4].y(), bboxPt[4].z());           
            glEnd();

            // 绘制四边
            glBegin(GL_LINES);
            glVertex3f(bboxPt[0].x(), bboxPt[0].y(), bboxPt[0].z());
            glVertex3f(bboxPt[4].x(), bboxPt[4].y(), bboxPt[4].z());

            glVertex3f(bboxPt[1].x(), bboxPt[1].y(), bboxPt[1].z());
            glVertex3f(bboxPt[5].x(), bboxPt[5].y(), bboxPt[5].z());

            glVertex3f(bboxPt[2].x(), bboxPt[2].y(), bboxPt[2].z());
            glVertex3f(bboxPt[6].x(), bboxPt[6].y(), bboxPt[6].z());

            glVertex3f(bboxPt[3].x(), bboxPt[3].y(), bboxPt[3].z());
            glVertex3f(bboxPt[7].x(), bboxPt[7].y(), bboxPt[7].z());           
            glEnd();
        }
    }
}

DrawSelectAbleBBox::Prt DrawSelectAbleBBox::FromCloud(
                        const std::vector<Cloud::Ptr> & posVec, 
                        bool drawZAxis,
                        const Eigen::Vector3f& color)
{
    return std::make_shared<DrawSelectAbleBBox>(DrawSelectAbleBBox(posVec, drawZAxis, color));   
}