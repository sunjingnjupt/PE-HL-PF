#ifndef SRC_INCLUDE_VIEWER_H
#define SRC_INCLUDE_VIEWER_H

#include <memory>
#include <GL/gl.h>
#include <GL/glu.h>
#include <QGLViewer/qglviewer.h>
#include <mutex>
#include <vector>
#include "drawable.h"

#include <QMouseEvent>
#include <QPoint>

#include "object.h"
#include "drawable_selectable_cloud.h"
#include "groundRemove/include/param.h"

class Viewer : public QGLViewer
{
    // Q_OBJECT  这里一定不能定义 Q_OBJECT 这个选项， 很容易报错、
    // 根据 depth_clustering 的逻辑来， 就不会报错
public:
    explicit Viewer(QWidget *parent = 0) : QGLViewer(parent), isFullScreen(false) {}
    void AddDrawable(Drawable::Prt drawable, std::string  name = "null");
    void Clear();
    ~Viewer() override {}
    void getClickedPoint(double & x, double & y);
    void setBBoxs(const std::vector<Cloud::Ptr> & inputBBox);
    
    void SetShowCornerNumBool(const bool & showCorner);
protected:
    void draw() override;
    void init() override;

    // 绘制文本信息
    void displayText();
    // 多选框
    // Selection functions
    virtual void drawWithNames();
    virtual void endSelection(const QPoint &);

      // Mouse events functions
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);
    // virtual void mouseDoubleClickEvent(QMouseEvent *e);

    // 寻找我们点击的点
    // virtual void postSelection(const QPoint &point);

    
public:
    DrawSelectAbleCloud drawSelectableCloud;
    DrawSelectAbleBBox drawSelectableBBox;
    std::vector<int> selection;
    std::vector<int> bboxSelection;
private:
    std::vector<Drawable::Prt> _drawables;
    mutable std::mutex _cloud_mutex;

    qglviewer::Vec selectedPoint;

    // Current rectangular selection
    QRect rectangle_;

    void drawSelectionRectangle() const;
    void addIdToSelection(int id);
    void removeIdFromSelection(int id);

    void drawRefCircle(const float & radius, const int & numPoints);
    // Different selection modes
    enum SelectionMode { NONE, ADD, REMOVE };
    SelectionMode selectionMode_;
    // 可视化使用的
    // 调用的 drawSelectableCloud 的 objects 对象
    // 所以此处暂时不需要定义自己的 objects
    // std::vector<Object::Ptr> objects

    // 是否显示 cornelNum
    bool showCorner_ = false;

public:
    bool isFullScreen;
    params paramsViewer;
    std::vector<Cloud::Ptr> bboxs;
    // 历史 bbox 的参考点
    Cloud::Ptr hisRefPts;
};
#endif



