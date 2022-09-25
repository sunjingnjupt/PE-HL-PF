#include "drawable_cloud.h"
#include <math.h>

void DrawableTracking::Draw() const
{
    // fprintf(stderr, "DrawableTracking::Draw()\n");
    if (!_cloud_ptr)
    {
        throw std::runtime_error("DrawableTracking has no cloud to draw.");
    }

    // fprintf(stderr, "DrawableTracking::Draw() after _cloud_ptr check\n");
    glPushMatrix();
    glEnable(GL_LINE_STIPPLE);
    glLineWidth(4.0f);
    glLineStipple(2, 0x3F3F);
    glPointSize(_pointSize);
    // 绘制虚线
    glBegin(GL_LINES);
    bool multiColor = (_numCluster != -1);
    // 点数是二的倍数
    auto & points = _cloud_ptr->points();
    // assert(points.size() % 6 == 0);
    assert(points.size() % 4 == 0);
    for (int idx = 0; idx < points.size() / 4; ++idx)
    {   
        auto real_point1 = points[4 * idx    ].AsEigenVector();
        auto real_point2 = points[4 * idx + 1].AsEigenVector();
        auto real_point3 = points[4 * idx + 2].AsEigenVector();
        auto real_point4 = points[4 * idx + 3].AsEigenVector();
        // auto real_point1 = points[6 * idx    ].AsEigenVector();
        // auto real_point2 = points[6 * idx + 1].AsEigenVector();
        // auto real_point3 = points[6 * idx + 2].AsEigenVector();
        // auto real_point4 = points[6 * idx + 3].AsEigenVector();
        // auto real_point5 = points[6 * idx + 4].AsEigenVector();
        // auto real_point6 = points[6 * idx + 5].AsEigenVector();       

        // 二维显示
        glColor3f(1.0f, 0.0f, 0.0f);  // cv
        glVertex3f(real_point1.x(), real_point1.y(), -1.72f);
        // glVertex3f(real_point4.x(), real_point4.y(), -1.72f);
        glVertex3f(real_point3.x(), real_point3.y(), -1.72f);

        glColor3f(0.0f, 1.0f, 0.0f);  // ctrv, or 4 point for x_merge
        glVertex3f(real_point2.x(), real_point2.y(), -1.72f);
        // glVertex3f(real_point5.x(), real_point5.y(), -1.72f);
        glVertex3f(real_point4.x(), real_point4.y(), -1.72f);

        // glColor3f(0.0f, 0.0f, 1.0f);  // rm
        // glVertex3f(real_point3.x(), real_point3.y(), -1.72f);
        // glVertex3f(real_point6.x(), real_point6.y(), -1.72f);

    }
    glEnd();
    glDisable(GL_LINE_STIPPLE);
    glPopMatrix();    

    // 绘制点 虚线俩个端点
    glPushMatrix();
    glPointSize(12.0f);
    glBegin(GL_POINTS);   

    for (int idx = 0; idx < points.size() / 4; ++idx)
    { 
        auto real_point1 = points[4 * idx    ].AsEigenVector();
        auto real_point2 = points[4 * idx + 1].AsEigenVector();
        auto real_point3 = points[4 * idx + 2].AsEigenVector();
        auto real_point4 = points[4 * idx + 3].AsEigenVector();  
        // auto real_point1 = points[6 * idx    ].AsEigenVector();
        // auto real_point2 = points[6 * idx + 1].AsEigenVector();
        // auto real_point3 = points[6 * idx + 2].AsEigenVector();
        // auto real_point4 = points[6 * idx + 3].AsEigenVector();
        // auto real_point5 = points[6 * idx + 4].AsEigenVector();
        // auto real_point6 = points[6 * idx + 5].AsEigenVector();       

        // 二维显示
        glColor3f(1.0f, 0.0f, 0.0f);  // cv
        glVertex3f(real_point1.x(), real_point1.y(), -1.72f);
        glColor3f(1.0f, 1.0f, 1.0f);  // 预测值
        // glVertex3f(real_point4.x(), real_point4.y(), -1.72f);
        glVertex3f(real_point3.x(), real_point3.y(), -1.72f);

        glColor3f(0.0f, 1.0f, 0.0f);  // ctrv
        glVertex3f(real_point2.x(), real_point2.y(), -1.72f);
        glColor3f(1.0f, 1.0f, 1.0f);  // 预测 merge 之后的值
        // glVertex3f(real_point5.x(), real_point5.y(), -1.72f);
        glVertex3f(real_point4.x(), real_point4.y(), -1.72f);

        // glColor3f(0.0f, 0.0f, 1.0f);  // rm
        // glVertex3f(real_point3.x(), real_point3.y(), -1.72f);
        // glColor3f(1.0f, 1.0f, 1.0f);
        // glVertex3f(real_point6.x(), real_point6.y(), -1.72f);

    }
    glEnd();
    glPopMatrix();  
}

DrawableTracking::Ptr DrawableTracking::FromCloud(const Cloud::ConstPtr& cloud,
                                            const Eigen::Vector3f& color,
                                            const GLfloat & pointSize,
                                            const int numCluster)
{
    return std::make_shared<DrawableTracking>(DrawableTracking(cloud, color, pointSize, numCluster));
}


void DrawableCloud::Draw() const
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
    // glColor3f(_color[0], _color[1], _color[2]);    
    // fprintf(stderr, "there has about %ld points\n", _cloud_ptr->size());
    // bool multiColor = ((*_cloud_ptr)[0].classID != -1);
    bool multiColor = (_numCluster != -1);
    for (const auto & point : _cloud_ptr->points())
    {
        if (!multiColor)
        {
            glColor3f(_color[0], _color[1], _color[2]);
        }
        else
        {
            // int classID = point.classID % _param.RANDOM_COLORS.size();
            // // fprintf(stderr, "classID[%d]\n", classID);
            // glColor3f((float)_param.RANDOM_COLORS[classID][0] / 255,
            //           (float)_param.RANDOM_COLORS[classID][1] / 255,
            //           (float)_param.RANDOM_COLORS[classID][2] / 255
            //         );
            glColor3f(0.835f, 0.031f, 0.043f); // 浅红色
        }   

        auto real_point = point.AsEigenVector();
        // 其他的显示点的类型
        if (point.ptType != pointType::TRACK || point.ptType != pointType::TRACKHIS)
        {
            // glVertex3f(real_point.x(), real_point.y(), -1.73f); // 点云二维显示
            glVertex3f(real_point.x(), real_point.y(), real_point.z()); // 点云二维显示
            // fprintf(stderr, "real_point.z() %f\n", real_point.z());
        }

    }
    glEnd();
    glPopMatrix();

    glColor3f(1.0f, 0.83f, 0.38f);
    glLineWidth(3.0f);

    for (const auto & point : _cloud_ptr->points())
    {
        if (point.ptType == pointType::TRACK)
        {
            auto real_point = point.AsEigenVector();
            glPushMatrix();
            glTranslatef(real_point.x(), real_point.y(), 0.0f);
            drawCircle(0.24, 12);
            glPopMatrix();  
        }
    }

    glColor3f(0.0f, 1.0f, 0.0f);
    glLineWidth(3.0f);

    for (const auto & point : _cloud_ptr->points())
    {
        if (point.ptType == pointType::TRACKHIS)
        {
            auto real_point = point.AsEigenVector();
            glPushMatrix();
            glTranslatef(real_point.x(), real_point.y(), 0.0f);
            drawCircle(0.24, 12);
            // fprintf(stderr, "has draw his ref points\n");
            glPopMatrix();  
        }
    }

}

void DrawableCloud::drawCircle(const float & radius, const int & numPoints) const
{
    // glVertex2f(R*cos(2*Pi/n*i), R*sin(2*Pi/n*i));
    glBegin(GL_LINE_STRIP);
    // 实心圆
    // glBegin(GL_TRIANGLE_FAN);
    for (size_t idx = 0; idx < numPoints; ++idx)
    {
        glVertex3f(radius * cos(2 * M_PI / numPoints * idx), radius * sin(2 * M_PI / numPoints * idx), -1.71f);
    }
    glVertex3f(radius * cos(0), radius * sin(0), -1.73);
    glEnd();
}

DrawableCloud::Ptr DrawableCloud::FromCloud(const Cloud::ConstPtr& cloud,
                                            const Eigen::Vector3f& color,
                                            const GLfloat & pointSize,
                                            const int numCluster)
{
    return std::make_shared<DrawableCloud>(DrawableCloud(cloud, color, pointSize, numCluster));
}




// 矩形繪制
DrawableRect::DrawableRect(const std::vector<Rect2D> & posVec, const float & z)
{
    rectPosVec = posVec;
    hightToGround = z;
}

// 可视化 voxel 的 Rect 大小
void DrawableRect::Draw() const
{
    // fprintf(stderr, "void DrawableRect::Draw() const\n");

    glPushMatrix();
    // fprintf(stderr, "rectPosVec size : %d\n", rectPosVec.size());
    glLineWidth(0.5f);
    glColor3f(0.8f, 0.8f, 0.8f);
    for (int idx = 0; idx < rectPosVec.size(); ++idx)
    {
        Rect2D rect = rectPosVec[idx];
        float size = rect.getSize();
        // float x1 = rect.x() - size / 2;
        // float y1 = rect.y() - size / 2;

        // float x2 = rect.x() + size / 2;
        // float y2 = rect.y() + size / 2;
        float x1 = rect.x();
        float y1 = rect.y();

        float x2 = rect.x() + size;
        float y2 = rect.y() + size;

        // glRectf(x1, y1, x2, y2);
        float hight = -1.73f;
        glBegin(GL_LINE_STRIP);
        glVertex3f(x1, y1, hight);
        glVertex3f(x1, y2, hight);
        glVertex3f(x2, y2, hight);
        glVertex3f(x2, y1, hight);
        glVertex3f(x1, y1, hight);
        glEnd();
        // fprintf(stderr, "[%.3f] [%.3f, %.3f, %.3f, %.3f]\n", size, x1, y1, x2, y2);
    }

    glPopMatrix();
}

DrawableRect::Prt DrawableRect::FromRectVec(const std::vector<Rect2D> & posVec, const float & z)
{
    return std::make_shared<DrawableRect>(DrawableRect(posVec, z));
}

// -------------------------------------------------------------------------------------
DrawableBBox::DrawableBBox(const std::vector<Cloud::Ptr> & posVec, bool drawZAxis, int color)
{
    rectPosVec.assign(posVec.begin(), posVec.end());
    _drawZAxis = drawZAxis;
    _color = color;
}

void DrawableBBox::Draw() const
{
    glPushMatrix();
    glLineWidth(2.5f);
    glPointSize(6.0f);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //指定混合函数
    // glEnable(GL_BLEND);
    // glDisable(GL_LIGHTING);
    for (int iBBox = 0; iBBox < rectPosVec.size(); ++iBBox)
    {
        // fprintf(stderr, "void DrawableBBox::Draw() const start\n");
        // fprintf(stderr, "rect PosVec size %d\n", rectPosVec.size());
        Cloud bboxPt = (*rectPosVec[iBBox]);
        // fprintf(stderr, "bboxPt size %d\n", bboxPt.size());
        // glColor3f(0.0f, 1.0f, 0.0f); // 绿色
        // glColor3f(181.0/255, 124.0/255, 7.0/255);  // 金色
        if (_color == 0)
            glColor3f(181.0/255, 124.0/255, 7.0/255);
        else
            glColor3f(215.0f / 255, 46.0f / 255, 202.0f / 255);
        // glBegin(GL_QUAD_STRIP);
        // std::array<int, 4> bottom = {1, 2, 0, 3};
        // for (int idx = 0; idx < 4; ++idx)
        // {
        //     glVertex3f(bboxPt[bottom[idx]].x(), 
        //                 bboxPt[bottom[idx]].y(), 
        //                 bboxPt[bottom[idx]].z());
        // }
        // glVertex3f(bboxPt[0].x(), bboxPt[0].y(), bboxPt[0].z());

        //  ----------------------------------------------------
        glBegin(GL_LINE_STRIP);
        // std::array<int, 4> bottom = {1, 2, 0, 3};
        std::array<int, 5> bottom = {0, 1, 2, 3, 0};
        // int colorSize = _param.RANDOM_COLORS.size();
        // int trackingID = bboxPt.id % colorSize;
        // glColor3f((float)_param.RANDOM_COLORS[trackingID][0] / 255,
        //           (float)_param.RANDOM_COLORS[trackingID][1] / 255,
        //           (float)_param.RANDOM_COLORS[trackingID][2] / 255);
        for (int idx = 0; idx < 5; ++idx)
        {
            glVertex3f(bboxPt[bottom[idx]].x(), 
                        bboxPt[bottom[idx]].y(), 
                        -1.721f);
        }
        glEnd();
        glBegin(GL_POINTS);
        glVertex3f(0.5 * (bboxPt[0].x() + bboxPt[2].x()), 
                    0.5 * (bboxPt[0].y() + bboxPt[2].y()), 
                    -1.7f);
        glEnd();
        // fprintf(stderr, "void DrawableBBox::Draw() const end\n");
        //  ----------------------------------------------------
        if (_drawZAxis)
        {
            if (_color == 0)
                glColor3f(181.0/255, 124.0/255, 7.0/255);
            else
                glColor3f(215.0f / 255, 46.0f / 255, 202.0f / 255);
                
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

        /*
        glBegin(GL_QUAD_STRIP);
        std::array<int, 4> bottom = {1, 2, 0, 3};

        for (auto it = bottom.begin(); it != bottom.end(); ++it)
        {
            glVertex3f(bboxPt[*it].x(), bboxPt[*it].y(), bboxPt[*it].z());
        }
        glEnd();

        if (_drawZAxis)
        {
            std::array<int, 4> top = {5, 6, 4, 7};
            std::array<int, 8> surround = {0, 4, 3, 7, 2, 6, 1, 5};
            glBegin(GL_QUAD_STRIP);
            for (auto it = top.begin(); it != top.end(); ++it)
            {
                glVertex3f(bboxPt[*it].x(), bboxPt[*it].y(), bboxPt[*it].z());
            }
            glEnd();           

            glBegin(GL_QUAD_STRIP);
            for (auto it = surround.begin(); it != surround.end(); ++it)
            {
                glVertex3f(bboxPt[*it].x(), bboxPt[*it].y(), bboxPt[*it].z());
            }
            glEnd(); 
        }
        */
    }

    // glEnable(GL_LIGHTING);
    // glDisable(GL_BLEND);
}

DrawableBBox::Prt DrawableBBox::FromCloud(
                        const std::vector<Cloud::Ptr> & posVec, 
                        bool drawZAxis,
                        int color)
{
    return std::make_shared<DrawableBBox>(DrawableBBox(posVec, drawZAxis, color));    
}

DrawableEllipse::Prt DrawableEllipse::FromEllipseVec(const std::vector<rotateEllipse> & ellipeVec)
{
    return std::make_shared<DrawableEllipse>(DrawableEllipse(ellipeVec));
}

void GetEllipseValue(const float & a, 
    const float & b, 
    float & meanX, 
    float & meanY,
    float & theta, 
    const float & roateRad,
    float & x, 
    float & y)
{
	float nx = 0.0f;
	float ny = 0.0f;
	nx = a * cosf(theta);
	ny = b * sinf(theta);
	x = nx * cos(roateRad) - ny * sin(roateRad) + meanX;
	y = nx * sin(roateRad) + ny * cos(roateRad) + meanY;
}

void DrawableEllipse::Draw() const
{
    // 测量的为白色
    glPushMatrix();
    glLineWidth(3.0f);
    Eigen::MatrixXd stateMatrix;
    Eigen::MatrixXd corrMatrix;
    float DX = 0.0f;
    float DY = 0.0f;
    float meanX = 0.0f;
    float meanY = 0.0f;

    const float thetaResolution = 2.0 * M_PI / 30.0f;

    // fprintf(stderr, "size of ellipse: %d\n", rotateEllipseVec.size());

    for (size_t ellipseIdx = 0; ellipseIdx < rotateEllipseVec.size(); ++ellipseIdx)
    {
        auto & curr = rotateEllipseVec[ellipseIdx];
        if (curr.type == EllipeType::MEASURE)
        {
            glColor3f(1.0f, 1.0f, 1.0f);
        }
        else if (curr.type == EllipeType::PREDICTE)
        {
            glColor3f(1.0f, 0.0f, 0.0f);
        }
        else  // MEAGER
        {
            glColor3f(0.0f, 1.0f, 0.0f);
        }                
        
        // std::cout << curr.state << std::endl << std::flush;
        // std::cout << curr.covariance << std::endl << std::flush;


        stateMatrix = curr.state;
        corrMatrix = curr.covariance;
        if (std::isnan(stateMatrix(0, 0)) ||
            std::isnan(stateMatrix(1, 0)) ||
            std::isnan(corrMatrix(0, 0)) ||
            std::isnan(corrMatrix(1, 1)) ||
            std::isnan(corrMatrix(0, 1)))
        {
            std::cout << curr.state << std::endl << std::flush;
            std::cout << curr.covariance << std::endl << std::flush;
            continue;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(corrMatrix);
        Eigen::VectorXd eigVec;

        if (eigenSolver.info() == Eigen::Success)
        {
            eigVec = eigenSolver.eigenvectors().block<1, 2>(1, 0);
        }
        else
        {
            fprintf(stderr, "corrMatrix has no eig vector!!\n");
            exit(1);
        }

        glBegin(GL_LINE_STRIP);
        float angleRad = 0.0f;

        const float rotateRad = atan2(eigVec(1),  eigVec(0));
        DX = corrMatrix(0, 0);
        DY = corrMatrix(1, 1);
        const float dx = 2 * sqrtf(DX);
        const float dy = 2 * sqrtf(DY);

        meanX = stateMatrix(0, 0);
        meanY = stateMatrix(1, 0);

        while (angleRad < 2 * M_PI)
        {
            float x = 0.0f;
            float y = 0.0f;
            GetEllipseValue(dx, dy, meanX, meanY, angleRad, rotateRad, x, y);
            glVertex3f(x, y, -1.73f);
            angleRad += thetaResolution;
        }
        glEnd();
    }
}