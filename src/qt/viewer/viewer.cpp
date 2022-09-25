#include "viewer.h"
#include <QDebug>
using std::mutex;
using std::lock_guard;

void Viewer::AddDrawable(Drawable::Prt drawable, std::string name)
{
    // fprintf(stderr,  "%s\n", name.c_str());
    // drawable->Draw();
    // fprintf(stderr, "Viewer::AddDrawable()\n");
    lock_guard<mutex> guard(_cloud_mutex);
    _drawables.push_back(drawable);
    // fprintf(stderr, "drawable size : %d\n", _drawables.size());
}

void Viewer::Clear()
{
    lock_guard<mutex> guard(_cloud_mutex);
    _drawables.clear();
    bboxs.clear();
    // 清楚历史 ref point
    // hisRefPts->clear();
}

void Viewer::draw() 
{
    // 坐标原点
    QGLViewer::drawAxis(sceneRadius() / 40.0);
    // 绘制自车
    // glEnable(GL_LINE_STIPPLE);
    // glLineStipple(2, 0x3F3F);
    // glColor3f(1.0f, 0.0f, 0.0f);
    // glLineWidth(2.0f);
    // glBegin(GL_LINE_STRIP);
    // glVertex3f(-2.5f, -1.4f, -1.73f);
    // glVertex3f(2.5f, -1.4f, -1.73f);
    // glVertex3f(2.5f, 1.4f, -1.73f);
    // glVertex3f(-2.5f, 1.4f, -1.73f);
    // glVertex3f(-2.5f, -1.4f, -1.73f);   
    // glEnd();
    // glDisable(GL_LINE_STIPPLE);

    // 绘制坐标系
    // glColor3f(0.0f, 1.0f, 0.0f);
    // glBegin(GL_LINE_STRIP);
    // glVertex3f(0.3f, 0.0f, -1.73f);
    // glVertex3f(0.0f, 0.0f, -1.73f);
    // glVertex3f(0.0f, 0.3f, -1.73f);  
    // glEnd();
    // 参考前进方向
    // glLineWidth (0.8f);    
    // glLineStipple (1, 0x0F0F);
    // glBegin(GL_LINES);  
    // glVertex3f(-100.0f, 0.0f, -1.72f);
    // glVertex3f(100.0f, 0.0f, -1.72f);
    // glVertex3f(0.0f, -100.0f, -1.72f);
    // glVertex3f(0.0f, 100.0f, -1.72f);
    // glEnd();
    // glLineWidth (1.5f);    
    // glLineStipple (1, 0x0F0F);
    // glBegin(GL_LINES);  
    // glVertex3f(-100.0f, -1.0f, -1.72f);
    // glVertex3f(100.0f, -1.0f, -1.72f);
    // glVertex3f(-100.0f, 1.0f, -1.72f);
    // glVertex3f(100.0f, 1.0f, -1.72f);
    // glEnd();
    ////////------------------   绘制参考线圆线 白色----------------    
    // float radius = 10;
    // glColor3f(1.0f, 0.0f, 0.0f);
    // while (radius <= 100)
    // {
    //     /* code */
    //     drawRefCircle(radius, radius * 100);
    //     radius += 10;
    // }
    ////////--------------------- 绘制距离圆结束  ------------------
    
    // ---------------------------- 绘制可视化网格 ----
    // float radius = 10;
    // glColor3f(1.0f, 1.0f, 1.0f);
    // float minSize = 0.2f;
    // float maxSize = 0.7f;

    // // 每次增加多少
    // float deta = 0.003f;
    // float minR = 1.0f;
    // float maxR = 120.0f;

    // int numGrid = 200;
    // float radius = minR;
    // for (int idx = 0; idx < numGrid && radius < maxR; ++idx)
    // {
    //     drawRefCircle(radius, int(radius * 100));
    //     radius += minSize;
    //     minSize += deta;
    // }

    // // while (radius <= 100)
    // // {
    // //     /* code */
    // //     drawRefCircle(radius, radius * 100);
    // //     radius += 10;
    // // }
    // //  paramsViewer.row_angles.size()
    // // float minR = 1.73f / atan(std::abs(paramsViewer.end_angle));
    // // float maxR = 0;
    // // for (int idx = 0; idx < paramsViewer.row_angles.size(); ++idx)
    // // {
    // //     float angle = paramsViewer.row_angles[idx];
    // //     // fprintf(stderr, "%f\n", angle);
    // //     // if (angle > -2 / M_PI * 180)
    // //     //     continue;
    // //     if (angle > 0.0f)
    // //         continue;
    // //     float d = 1.73f / atan(std::abs(angle));
    // //     if (d > 130)
    // //         continue;
    // //     drawRefCircle(d - minR, int(100 * d));
    // //     if (d > maxR)
    // //         maxR = d;
    // // }

    // // float distCurr = 1.73f / atan(std::abs(paramsViewer.min_angle_two));
    // // for (int idx = 0; idx < 200; ++idx)
    // // {
    // //     distCurr += 0.5f;
    // //     drawRefCircle(distCurr, int(100 * distCurr));
    // // }
    
    // int numPoints = 600;
    // glBegin(GL_LINES);
    // for (int idx = 0; idx < numPoints; ++idx)
    // {     
    //     glVertex3f(0.5 * cos(2 * M_PI / numPoints * idx), 0.5 * sin(2 * M_PI / numPoints * idx), -1.73); 
    //     glVertex3f(maxR * cos(2 * M_PI / numPoints * idx), maxR * sin(2 * M_PI / numPoints * idx), -1.73); 
    // }
    // glEnd();

    // ----------------------------------------------
    //
    // qDebug() << "window clicked .\n" << endl;
    // fprintf(stderr, "Viewer::draw()\n");    
    // fprintf(stderr, "\n\n\ncloud total number : %d\n", drawSelectableCloud.objects.size());
    // fprintf(stderr, "cloud selected id :\n");
    // for (auto it = selection.begin(); it != selection.end(); ++it)
    // {
    //     fprintf(stderr, "%d ", *it);
    // }

    // fprintf(stderr, "bbox selected id :\n");
    // for (auto it = bboxSelection.begin(); it != bboxSelection.end(); ++it)
    // {
    //     fprintf(stderr, "%d ", *it);
    // }

    glPointSize(4);
    glColor3f(0.9f, 0.3f, 0.3f);
    size_t CloudNum = drawSelectableCloud.objects.size();
    size_t BBoxNum = drawSelectableBBox.objects.size();
    if (selection.size() != 0 && CloudNum != 0)
    {
        for (auto it = selection.begin(); it != selection.end(); ++it)
        {
            if (*it < CloudNum)
                drawSelectableCloud.objects[*it]->draw();
            else if (CloudNum <= (*it) && (*it) <= (CloudNum + BBoxNum))
            {
                drawSelectableBBox.objects[*it - CloudNum]->draw();
            }
        }
    }

    lock_guard<mutex> guard(_cloud_mutex);
    for (auto& drawable : _drawables)
    {
        drawable->Draw();
    }

    // fprintf(stderr, "Viewer::draw()\n");
    // Draws rectangular selection area. Could be done in postDraw() instead.
    if (selectionMode_ != NONE)
        drawSelectionRectangle();

    
    // // yyg add
    // // Draws selected objects only.
    // glColor3f(0.9f, 0.3f, 0.3f);
    // // for (QList<int>::const_iterator it = selection.begin(),
    // //                                 end = selection.end();
    // //     it != end; ++it)
    // //     objects_.at(*it)->draw();
    // for (auto it = selection.begin(); it != selection.end(); ++it)
    // {
    //     objects_[*it]->draw();
    // }

    // // Draws all the objects. Selected ones are not repainted because of GL depth
    // // test.
    // glColor3f(0.8f, 0.8f, 0.8f);
    // for (int i = 0; i < int(objects_.size()); i++)
    //     objects_.at(i)->draw();
    // test draw Text function
    displayText();

}

void Viewer::displayText()
{
    // 绘制 bboxs 的索引 
    // 设置颜色
    // QPalette pa;
    // pa.setColor(QPalette::WindowText, Qt::red);
    // setPalette(pa);
    // QFont ft("Helvetica [Cronyx]", 10, QFont::Bold);
    QFont ft("Arial", 10);
    QFont ftID("Arial", 12, QFont::Bold);
    // QFont ft("Time New Roman", 12, QFont::Bold);
    // QFont ft("Comic Sans MS", 12);
    // QFont ft("Fixed", 8, QFont::Bold);
    qglColor(Qt::black);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    for (int idx = 0; idx < bboxs.size(); ++idx)
    {
        glColor3f(1.0f, 1.0f, 1.0f);
        auto &bbox = (*bboxs[idx]);
        // renderText(bbox[0].x(), bbox[1].y(), bbox[2].z(),
        //     QString::number(idx), ft);
        point pt1 = bbox[0];
        point pt2 = bbox[2];
        QString strID = QString("ID:") + QString::number(bboxs[idx]->id);// + "m/s";
        float x = 0.5 * (pt1.x() + pt2.x());
        float y = 0.5 * (pt1.y() + pt2.y());
        // float z = 0.5 * (bbox.minZ + bbox.maxZ);
        float z = -1.90f;
        float v = bbox.velocity * 3.6;
        float yaw = bbox.yaw;
        const qglviewer::Vec point2dID = camera()->projectedCoordinatesOf(qglviewer::Vec(x, y + 0.1, z));
        QGLViewer::drawText(point2dID.x, point2dID.y, strID, ftID);

        QString velocityStr = QString::number(v, 'g', 2) + " km/h";
        const qglviewer::Vec point2d = camera()->projectedCoordinatesOf(qglviewer::Vec(x - 0.3, y - 0.3, z));
        QGLViewer::drawText(point2d.x, point2d.y, velocityStr, ft);

        // 显示角度信息        
        point ArrowTo;
        ArrowTo.x() = x + 0.3 * v * cos(yaw);
        ArrowTo.y() = y + 0.3 * v * sin(yaw);
        glColor3f(0.0f, 1.0f, 0.0f);
        QGLViewer::drawArrow(qglviewer::Vec(x, y, z), qglviewer::Vec(ArrowTo.x(), ArrowTo.y(), z), 0.045);

        glColor3f(1.0f, 1.0f, 1.0f);
        if (showCorner_) {
            for (size_t ptIdx = 0; ptIdx < 4U; ++ptIdx) {
                auto& pos = bbox[ptIdx];
                QString cornerNumStr = QString::number(ptIdx * 2);
                const qglviewer::Vec &cornerPos = camera()->projectedCoordinatesOf(qglviewer::Vec(pos.x(), pos.y(), -1.73F));
                QGLViewer::drawText(cornerPos.x, cornerPos.y, cornerNumStr, ft);
            }
        }
    }    
    
//    glEnable(GL_DEPTH_TEST);
//    glDisable(GL_LIGHTING);
}

void Viewer::drawWithNames() 
{
    // for (int i = 0; i < int(objects_.size()); i++) 
    // {
    //     glPushName(i);
    //     objects_.at(i)->draw();
    //     glPopName();
    // }
    // fprintf(stderr, "Viewer::drawWithNames()\n");
    // fprintf(stderr, "drawSelectableCloud.objects.size : %d\n",
    //                 drawSelectableCloud.objects.size());
    // 非空 画图
    if (drawSelectableCloud.objects.size() != 0)
    {
        // this->objects.assign(drawSelectableCloud.objects.begin(), 
        //         drawSelectableCloud.objects.end());
        int idx = 0;
        for (auto & elem : drawSelectableCloud.objects)
        {
            // fprintf(stderr, "Cloud pushName with %d\n", idx);
            glPushName(idx);
            elem->draw();
            glPopName();
            ++idx;
        }
    }

    // 对象减去点云对象数目， 等于 bbox 的 ID
    // fprintf(stderr, "drawSelectableBBox.objects.size : %d\n", drawSelectableBBox.objects.size());
    if(drawSelectableBBox.objects.size() != 0)
    {
        int idx = drawSelectableCloud.objects.size();
        for (auto & elem : drawSelectableBBox.objects)
        {
            // fprintf(stderr, "BBox pushName with %d\n", idx);
            glPushName(idx);
            elem->draw();
            glPopName();
            ++idx;
        }
    }

    

}

// 显示按压事件
void Viewer::mousePressEvent(QMouseEvent *e) {
    // Start selection. Mode is ADD with Shift key and TOGGLE with Alt key.
    //   qDebug() << e->pos() << std::endl;
    rectangle_ = QRect(e->pos(), e->pos());

    if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::ShiftModifier))
    {
        selectionMode_ = ADD;
    }
        // else if ((e->button() == Qt::LeftButton) &&
        //          (e->modifiers() == Qt::AltModifier))
    else if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::AltModifier))
    {
        selectionMode_ = REMOVE;    
    }
    else 
    {
        // if (e->modifiers() == Qt::ControlModifier)
        //   startManipulation();  //不需要操作
        QGLViewer::mousePressEvent(e);
    }
}

void Viewer::mouseMoveEvent(QMouseEvent *e) 
{
    if (selectionMode_ != NONE) 
    {
        // Updates rectangle_ coordinates and redraws rectangle
        // qDebug() << "curren pose :" << e->pos() << endl;
        rectangle_.setBottomRight(e->pos());
        update();
    } 
    else
    {
        QGLViewer::mouseMoveEvent(e);
    }
}

void Viewer::mouseReleaseEvent(QMouseEvent *e) 
{
    if (selectionMode_ != NONE) 
    {
        // Actual selection on the rectangular area.
        // Possibly swap left/right and top/bottom to make rectangle_ valid.
        rectangle_ = rectangle_.normalized();
        // Define selection window dimensions
        setSelectRegionWidth(rectangle_.width());
        setSelectRegionHeight(rectangle_.height());
        // Compute rectangle center and perform selection
        select(rectangle_.center());
        // Update display to show new selected objects
        update();
    } 
    else
        QGLViewer::mouseReleaseEvent(e);
}

void Viewer::endSelection(const QPoint &) 
{
    // Flush GL buffers
    // fprintf(stderr, "Viewer::endSelection(const QPoint &)\n");
    glFlush();

    // Get the number of objects that were seen through the pick matrix frustum.
    // Reset GL_RENDER mode.
    GLint nbHits = glRenderMode(GL_RENDER);
    // fprintf(stderr, "nbHits %d has hited\n", nbHits);

    if (nbHits > 0) 
    {
        // Interpret results : each object created 4 values in the selectBuffer().
        // (selectBuffer())[4*i+3] is the id pushed on the stack.
        for (int i = 0; i < nbHits; ++i)
        {
            switch (selectionMode_) 
            {
            case ADD:
                addIdToSelection((selectBuffer())[4 * i + 3]);
                break;
            case REMOVE:
                removeIdFromSelection((selectBuffer())[4 * i + 3]);
                break;
            default:
                break;
            }
        }

        // for (auto & ele : selection)
        // {
        //     // std::cout << ele << " ";
        //     fprintf(stderr, "%d ", ele);
        // }
        // fprintf(stderr, "\nhas been selected!\n\n\n\n");
    }

    // 打印添加后的 ID

    selectionMode_ = NONE;
}



void Viewer::removeIdFromSelection(int id) 
{ 
    // selection.removeAll(id); 
    selection.clear();
}

void Viewer::drawRefCircle(const float & radius, const int & numPoints)
{
    // glVertex2f(R*cos(2*Pi/n*i), R*sin(2*Pi/n*i));
    glLineWidth(1.0f);
    glBegin(GL_LINE_STRIP);
    for (size_t idx = 0; idx < numPoints; ++idx)
    {
        glVertex3f(radius * cos(2 * M_PI / numPoints * idx), radius * sin(2 * M_PI / numPoints * idx), -1.73);
    }
    glEnd();
}

void Viewer::init()
{    
    // glClearColor(1.0f,1.0f,1.0f,1.0f);
    // glClear(GL_COLOR_BUFFER_BIT);
    // fprintf(stderr, "Viewer::init()\n");
    // Clear Screen And Depth Buffer
    // glClearColor(1.0, 1.0, 1.0, 0.0);
    // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   // Reset The Current Modelview Matrix
    // glLoadIdentity();
    // 如果想要黑色 设置为 0.0, 0.0, 0.0,  如果想要白色为 255, 255, 255
    // setBackgroundColor(QColor(255, 255, 255));
    glClear(GL_COLOR_BUFFER_BIT);
    setBackgroundColor(QColor(1, 0, 1));
    setSceneRadius(50.0);
    camera()->showEntireScene();
//    glDisable(GL_LIGHTING);
//    glBlendFunc(GL_ONE, GL_ONE);
}

void Viewer::addIdToSelection(int id) 
{
    // if (!selection.contains(id))
    //     selection.push_back(id);
    int cloudNum = drawSelectableCloud.objects.size();
    int bboxNum = drawSelectableBBox.objects.size();
    auto it = std::find(selection.begin(), selection.end(), id);
    if (it == selection.end())
    {
        if (id < cloudNum)
            selection.emplace_back(id);
        if (id >= cloudNum && 
            id < (cloudNum + bboxNum))
        {
            bboxSelection.emplace_back(id - cloudNum);
        }
    }
}


// void Viewer::postSelection(const QPoint &point)
// {
//     bool found;
//     selectedPoint = camera()->pointUnderPixel(point, found);
//     // qDebug() << "point (" << QString(point.x()) + "," + QString(point.y()) + ")" << endl;
//     // std::cout << "selectedPoint " << selectedPoint << std::endl;
//     // qDebug() <<"selectedPoint " << "(" + QString(selectedPoint.x) + "," +
//     //                                      QString(selectedPoint.y) + "," +
//     //                                      QString(selectedPoint.z) +")" << endl;

//     fprintf(stderr, "(%f, %f , %f ) cilicked\n", selectedPoint.x, selectedPoint.y, selectedPoint.z);
// }

void Viewer::getClickedPoint(double &x, double &y)
{
    // auto a = selectedPoint.x;
    // qreal c = selectedPoint.y;
    // x = reinterpret_cast<double>(selectedPoint.x);
    // y = reinterpret_cast<double>(selectedPoint.y);
    x = (double)(selectedPoint.x);
    y = (double)(selectedPoint.y);
}


void Viewer::drawSelectionRectangle() const 
{
    startScreenCoordinatesSystem();
    glDisable(GL_LIGHTING);

    glEnable(GL_BLEND);
    glColor4f(0.0, 0.0, 0.3f, 0.3f);
    glBegin(GL_QUADS);
    glVertex2i(rectangle_.left(), rectangle_.top());
    glVertex2i(rectangle_.right(), rectangle_.top());
    glVertex2i(rectangle_.right(), rectangle_.bottom());
    glVertex2i(rectangle_.left(), rectangle_.bottom());
    glEnd();

    glLineWidth(2.0);
    glColor4f(0.4f, 0.4f, 0.5f, 0.5f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(rectangle_.left(), rectangle_.top());
    glVertex2i(rectangle_.right(), rectangle_.top());
    glVertex2i(rectangle_.right(), rectangle_.bottom());
    glVertex2i(rectangle_.left(), rectangle_.bottom());
    glEnd();

    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    stopScreenCoordinatesSystem();
}

void Viewer::setBBoxs(const std::vector<Cloud::Ptr> & inputBBox)
{
    bboxs.assign(inputBBox.begin(), inputBBox.end());
    // hisRefPts.reset(inputRefPoints)
    // hisRefPts.reset(inputRefPoints.get());
}

void Viewer::SetShowCornerNumBool(const bool & showCorner)
{
    showCorner_ = showCorner;
}