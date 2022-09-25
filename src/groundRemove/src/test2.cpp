#include <iostream>
#include <fstream>
#include "param.h"
#include <vector>
#include <array>
#include <sstream>
#include <QWidget>
#include <QMainWindow>
#include <qapplication.h>

#include "Eigen/Dense"

#include <GL/gl.h>
#include <GL/glu.h>
#include <QGLViewer/qglviewer.h>

using std::string;
using std::vector;
using std::stringstream;

// 备注 椭圆朝向应该为协方差矩阵的最大特征值朝向方向

class Viewer : public QGLViewer {
protected:
  virtual void draw();
  virtual void init();
};

void getFunZValue(const float & i, const float & j, float & z)
{
	// 球体
	if (i * i + j * j > 1.0f)
	{
		z = 0.0f;
		return;
	}
	const static float r = 1.0f;
	z = std::sqrt(r - i * i - j * j);
}

void GetGaussianValue(const float &x, 
	const float & y,
	const float & meanX, 
	const float & meanY,
	const float & dx,
	const float & dy,
	const float & r,
	const float & part1,
	const float & p1,
	float & res)
{
	const float px = (x - meanX) * (x - meanX) / (dx * dx);
	const float py = (y - meanY) * (y - meanY) / (dy * dy);
	const float pxy = 2 * r * (x - meanX) * (y - meanY) / (dx * dy);
	res = part1 * std::exp(p1 * (px - pxy + py));
	// fprintf(stderr, "x %f\n", x);
	// fprintf(stderr, "y %f\n", y);
	// fprintf(stderr, "px %f\n", px);
	// fprintf(stderr, "py %f\n", py);
	// fprintf(stderr, "pxy %f\n", pxy);
	// fprintf(stderr, "part1 %f\n", part1);
	// fprintf(stderr, "res %f\n", res);
}

void GetEllipseValue(const float & a, const float & b, const float & theta, const float & roateRad,float & x, float & y)
{
	float nx = 0.0f;
	float ny = 0.0f;
	nx = a * cosf(theta);
	ny = b * sinf(theta);
	x = nx * cos(roateRad) - ny * sin(roateRad); // + Cx;
	y = nx * sin(roateRad) + ny * cos(roateRad); // + Cy;
}

void display(void)
{
    const float nbSteps = 200.0;

    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i < nbSteps; ++i) {
        const float ratio = i / nbSteps;
        const float angle = 21.0 * ratio;
        const float c = cos(angle);
        const float s = sin(angle);
        const float r1 = 1.0 - 0.8f * ratio;
        const float r2 = 0.8f - 0.8f * ratio;
        const float alt = ratio - 0.5f;
        const float nor = 0.5f;
        const float up = sqrt(1.0 - nor * nor);
        glColor3f(1.0 - ratio, 0.2f, ratio);
        glNormal3f(nor * c, up, nor * s);
        glVertex3f(r1 * c, alt, r1 * s);
        glVertex3f(r2 * c, alt + 0.05f, r2 * s);
    }
    glEnd();
    return;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f(1.0f, 1.0f, 1.0f);
	glPushMatrix();
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	// 参数设置
	Eigen::Matrix2f corrMatrix;
	corrMatrix << 0.5f, 0.000175975f,
				  0.000175975f , 0.5f;
    float DX = corrMatrix(0, 0); // 方差 X
	float DY = corrMatrix(1, 1); // 方差 Y
	const float dx = sqrtf(DX);
	const float dy = sqrtf(DY);
	// const float r = -0.3f; // 协方差
	// const float r = 0.2f; // 不相关
	const float r = corrMatrix(0, 1) / (dx * dy);
	// 最大特征值的特征向量方向
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigenSolver(corrMatrix);
	Eigen::Vector2f eigVec;
	if (eigenSolver.info() == Eigen::Success)
	{
		eigVec = eigenSolver.eigenvectors().block<1, 2>(1, 0);
		// std::cout << "eightValue and vector :\n" <<
		// 			eigenSolver.eigenvalues() << "\n" <<
		// 			eigenSolver.eigenvectors() << std::endl;
	}
	else
	{
		fprintf(stderr, "corrMatrix has no eig vector!!\n");
		exit(1);
	}
	
	const float meanX = 0.0f; // 均值
	const float meanY = 0.0f; // 均值
	const float part1 = 1.0f / (2 * M_PI * dx * dy * sqrtf(1 - r * r));
	const float p1 = -1.0f / (2 * (1 - r * r));

	const float gridResolution = 0.2f;
	float zVal = 0.0f;
	float posX = 0.0f; 
	float posY = 0.0f;

	for (int j = -100; j < 100; ++j)
	{
		glBegin(GL_QUAD_STRIP);
		for (int i = -100; i <= 100; ++i)
		{
			posX = i * gridResolution;
			posY = j * gridResolution;

			GetGaussianValue(posX, posY, meanX, meanY, dx, dy, r, part1, p1, zVal);
			glColor3f(10 * zVal, 0.0f, 0.0f);
			glVertex3f(posX, posY, 200 * zVal);

			GetGaussianValue(posX, posY + gridResolution, meanX, meanY, dx, dy, r, part1, p1, zVal);
			glColor3f(10 * zVal, 0.0f, 0.0f);
			glVertex3f(posX, posY + gridResolution, 200 * zVal);
			// fprintf(stderr, "zVal %f\n", zVal);
		}
		glEnd();

	}

	glLineWidth(3.0f);
	glColor3f(0.0f, 1.0f, 0.0f);
	glBegin(GL_LINE_STRIP);
	const float theatResolution = 2.0f * M_PI / 60;
	float angleRad = 0.0f;
	// const float rotateRad = atanf(r);
	const float rotateRad = atan2(eigVec(1),  eigVec(0));
	while(angleRad < 2 * M_PI)
	{
		float x = 0.0f;
		float y = 0.0f;
		GetEllipseValue(3 * dx, 3 * dy, angleRad, rotateRad, x, y);
		glVertex3f(x, y, 0.0f);
		angleRad += theatResolution;
	}
	glEnd();

	// 2 theta
	glLineWidth(3.0f);
	glColor3f(0.0f, 0.0f, 1.0f);
	glBegin(GL_LINE_STRIP);
	angleRad = 0.0f;
	// const float rotateRad = atanf(r);
	while(angleRad < 2 * M_PI)
	{
		float x = 0.0f;
		float y = 0.0f;
		GetEllipseValue(2 * dx, 2 * dy, angleRad, rotateRad, x, y);
		glVertex3f(x, y, 0.0f);
		angleRad += theatResolution;
	}
	glEnd();
	glPopMatrix();
	glFlush();
		
}

void Viewer::draw()
{
	display();
}

void Viewer::init()
{
	glClear(GL_COLOR_BUFFER_BIT);
    // 如果想要黑色 设置为 0.0, 0.0, 0.0,  如果想要白色为 255, 255, 255
    setBackgroundColor(QColor(0, 0 ,0));
    // setBackgroundColor(QColor(1, 0, 1));
    setSceneRadius(2.0);
    // fprintf(stderr, "setBackgroundColor(QColor(1, 1, 1));\n");
    camera()->showEntireScene();
    glDisable(GL_LIGHTING);
    glBlendFunc(GL_ONE, GL_ONE);

	// glClearColor(1.0, 1.0, 1.0, 0.0);

	//下行的代码用控制点定义Bezier曲面函数
	// glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 4, 0, 1, 12, 4, &ctrlpoints[0][0][0]);
	/*
	void glMap2f(GLenum target, GLfloat u1, GLfloat u2, GLint ustride, GLint uorder, GLfloat v1, GLfloat v2, GLint vstride, GLint vorder, const GLfloat * points);
	二维求值器
	target：求值器生成的值的类型，指出了控制顶点的意义以及在points参数中需要提供多少值。
			GL_MAP2_VERTEX_3: xyz坐标
			GL_MAP2_VERTEX_4：xyzw坐标
			GL_MAP2_COLOR_4:  RGBA
			GL_MAP2_INDEX：   颜色索引
			GL_MAP2_NORMAL：  法线坐标
			GL_MAP2_TEXTURE_COORD_{}:纹理坐标，｛1,2,3,4｝
	u1、u2：生成x方向上的n个数，通常是从0变化到1。
	ustride：表示跨度（在每块存储区内浮点数或双精度数的个数，即两个控制点间的偏移量，对于三维的点，不管其坐标值是单精度还是双精度，stride的值都是3）。
	uorder： 阶数，等于Bernstein函数的阶次加1，与控制点数相等，对3次双项式，order的值为4。
	(ustride, uorder):生成x方向上，uorder个控制点，每个控制点由ustride个数表示。

	v1、v2： 生成y方向上的n个数，通常是从0变化到1。
	vstride：度为4的情况下，u方向上的Stride为3，因此3*4=12
	(vstride, vorder): 生成y方向上，vorder个控制点，控制点之间相隔ustride*uorder个数字

	points：用于传递控制点数组的起始地址，可以指向控制点集、RGBA颜色值或是纹理坐标串等。

	*/
	glEnable(GL_MAP2_VERTEX_3);   //激活该曲面函数
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// glOrtho(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0);   //构造平行投影矩阵
}

int main(int argc, char** argv)
{
	fprintf(stderr, "code start!\n");
	QApplication application(argc, argv);

	// Instantiate the viewer.
	Viewer viewer;

	viewer.setWindowTitle("drawGaussian");

	// Make the viewer window visible on screen.
	viewer.show();

	// Run main loop.
	return application.exec();
	fprintf(stderr, "code finished!\n");
}
