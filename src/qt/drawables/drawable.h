#ifndef SRC_DRAWABLES_H
#define SRC_DRAWABLES_H

#include <memory>
#include <GL/gl.h>
#include <GL/glu.h>

#include <QOpenGLWidget>
#include "QOpenGLFunctions"

#include "groundRemove/include/cloud.h"

class Drawable
{
public:
    using Prt = std::shared_ptr<Drawable>;
    virtual void Draw() const = 0;
    virtual ~Drawable() {}
};


#endif
