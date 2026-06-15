#include "MiniGL.h"

#ifdef WIN32
#include "windows.h"
#else
#include <cstdio>
#include <limits.h>
#ifndef MAX_PATH
#define MAX_PATH PATH_MAX
#endif
#define sprintf_s(buffer, format, ...) snprintf(buffer, sizeof(buffer), format, __VA_ARGS__)
#endif

#include <glad/gl.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#ifdef __APPLE__
#include <OpenGL/GL.h>
#include <OpenGL/GLU.h>
#else
#include "GL/gl.h"
#include "GL/glu.h"
#endif

#define _USE_MATH_DEFINES

#include "math.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Demos/Visualization/TextureAssetDiagnostics.h"
#include "Demos/Common/LiveHapticToolControl.h"
#include "Utils/Logger.h"
#include "Utils/FileSystem.h"

#include "Demos/Visualization/MyTimer.hpp"

using namespace PBD;

float MiniGL::fovy = 45;
float MiniGL::znear = 0.5f;
float MiniGL::zfar = 1000;
MiniGL::SceneFct MiniGL::scenefunc = nullptr;
MiniGL::IdleFct MiniGL::idlefunc = nullptr;
MiniGL::DestroyFct MiniGL::destroyfunc = nullptr;
void (*MiniGL::exitfunc)(void) = NULL;
int MiniGL::m_width = 0;
int MiniGL::m_height = 0;
Quaternionr MiniGL::m_rotation;
Real MiniGL::m_zoom = 1.0;
Vector3r MiniGL::m_translation;
Real MiniGL::movespeed = 1.0;
Real MiniGL::turnspeed = 0.01;
int MiniGL::mouse_button = -1;
double MiniGL::mouse_wheel_pos = 0;
int MiniGL::modifier_key = 0;
double MiniGL::mouse_pos_x_old = 0;
double MiniGL::mouse_pos_y_old = 0;
std::vector<MiniGL::KeyFunction> MiniGL::keyfunc;
int MiniGL::drawMode = GL_FILL;
unsigned char MiniGL::texData[IMAGE_ROWS][IMAGE_COLS][3];
unsigned int MiniGL::m_texId = 0;
unsigned int MiniGL::texId[6] = { 0,0,0,0,0,0 };
void(*MiniGL::selectionfunc)(const Vector2i&, const Vector2i&, void*) = NULL;
void(*MiniGL::hapticselectionfunc) (const Vector3r&, void*) = NULL;
void* MiniGL::selectionfuncClientData = NULL;
void(*MiniGL::mousefunc)(int, int, void*) = NULL;
int MiniGL::mouseFuncButton;
Vector2i MiniGL::m_selectionStart;
GLint MiniGL::m_context_major_version = 0;
GLint MiniGL::m_context_minor_version = 0;
GLint MiniGL::m_context_profile = 0;
bool MiniGL::m_breakPointActive = true;
bool MiniGL::m_breakPointLoop = false;
GLUquadricObj* MiniGL::m_sphereQuadric = nullptr;
std::vector<MiniGL::ReshapeFct> MiniGL::m_reshapeFct;
std::vector<MiniGL::KeyboardFct> MiniGL::m_keyboardFct;
std::vector<MiniGL::CharFct> MiniGL::m_charFct;
std::vector<MiniGL::MousePressFct> MiniGL::m_mousePressFct;
std::vector<MiniGL::MouseMoveFct> MiniGL::m_mouseMoveFct;
std::vector<MiniGL::MouseWheelFct> MiniGL::m_mouseWheelFct;
GLFWwindow* MiniGL::m_glfw_window = nullptr;
std::vector<MiniGL::Triangle> MiniGL::m_drawTriangle;
std::vector<MiniGL::Line> MiniGL::m_drawLines;
std::vector<MiniGL::Point> MiniGL::m_drawPoints;
bool MiniGL::m_vsync = false;
double MiniGL::m_lastTime;

// Initialize haptic related variables
#ifdef PBD_ENABLE_HAPTICS
HHD MiniGL::ghHD = HD_INVALID_HANDLE;
HHLRC MiniGL::ghHLRC = NULL;

//static const double kPI = 3.1415926535897932384626433832795;

HLuint MiniGL::gAxisId = 0;
hduVector3Dd MiniGL::gAxisCenter = hduVector3Dd(0, 0, 0);

HLuint MiniGL::gSphereShapeId = 0;

HLuint MiniGL::m_effectName = 0;

/* Position and orientation of proxy at start of drag. */
hduVector3Dd MiniGL::gStartDragProxyPos;
hduQuaternion MiniGL::gStartDragProxyRot;
#endif
bool	MiniGL::gButtonDownState = false;
bool MiniGL::gButton1DownState = false;
bool MiniGL::gButton2DownState = false;
bool MiniGL::gHapticAvailable = false;
Vector3r MiniGL::gHapticPos;
HLdouble MiniGL::gHapticXform[16] = { 0.0 };
double MiniGL::gHapticWorkspaceScale = 1.0;
hduMatrix MiniGL::gDeltaTMat;


/* Position and orientation of drag object at start of drag. */
#ifdef PBD_ENABLE_HAPTICS
hduMatrix MiniGL::gStartDragObjTransform;
#endif

/* Flag for enabling/disabling axis snap on drag. */
bool MiniGL::gAxisSnap = true;

/* flag for enabling/disabling rotation. */
bool MiniGL::gRotate = true;

double MiniGL::gCursorScale = 0;
GLuint MiniGL::gCursorDisplayList = 0;

void MiniGL::bindTexture()
{
	glBindTexture(GL_TEXTURE_2D, MiniGL::m_texId);
}

void MiniGL::bindTexture(const unsigned int id)
{
	glBindTexture(GL_TEXTURE_2D, MiniGL::texId[id]);
}

void MiniGL::unbindTexture()
{
	glBindTexture(GL_TEXTURE_2D, 0);
}

void MiniGL::getOpenGLVersion(int& major_version, int& minor_version)
{
	sscanf((const char*)glGetString(GL_VERSION), "%d.%d", &major_version, &minor_version);
}


void MiniGL::coordinateSystem()
{
	Eigen::Vector3f a(0, 0, 0);
	Eigen::Vector3f b(2, 0, 0);
	Eigen::Vector3f c(0, 2, 0);
	Eigen::Vector3f d(0, 0, 2);

	float diffcolor[4] = { 1,0,0,1 };
	float speccolor[4] = { 1,1,1,1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glLineWidth(2);

	glBegin(GL_LINES);
	glVertex3fv(&a[0]);
	glVertex3fv(&b[0]);
	glEnd();

	float diffcolor2[4] = { 0, 1, 0, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor2);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor2);

	glBegin(GL_LINES);
	glVertex3fv(&a[0]);
	glVertex3fv(&c[0]);
	glEnd();

	float diffcolor3[4] = { 0, 0, 1, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor3);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor3);

	glBegin(GL_LINES);
	glVertex3fv(&a[0]);
	glVertex3fv(&d[0]);
	glEnd();
	glLineWidth(1);
}

void MiniGL::drawVector(const Vector3r& a, const Vector3r& b, const float w, float* color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glLineWidth(w);

	glBegin(GL_LINES);
	glVertex3v(&a[0]);
	glVertex3v(&b[0]);
	glEnd();

	glLineWidth(1);
}

void MiniGL::drawCylinder(const Vector3r& a, const Vector3r& b, const float* color, const float radius, const unsigned int subdivisions)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	Real vx = (b.x() - a.x());
	Real vy = (b.y() - a.y());
	Real vz = (b.z() - a.z());
	//handle the degenerate case with an approximation
	if (vz == 0)
		vz = .00000001;
	Real v = sqrt(vx * vx + vy * vy + vz * vz);
	Real ax = static_cast<Real>(57.2957795) * acos(vz / v);
	if (vz < 0.0)
		ax = -ax;
	Real rx = -vy * vz;
	Real ry = vx * vz;

	GLUquadricObj* quadric = gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);

	glPushMatrix();
	glTranslatef((float)a.x(), (float)a.y(), (float)a.z());
	glRotatef((float)ax, (float)rx, (float)ry, 0.0f);
	//draw the cylinder
	gluCylinder(quadric, radius, radius, v, subdivisions, 1);
	gluQuadricOrientation(quadric, GLU_INSIDE);
	//draw the first cap
	gluDisk(quadric, 0.0, radius, subdivisions, 1);
	glTranslatef(0, 0, (float)v);
	//draw the second cap
	gluQuadricOrientation(quadric, GLU_OUTSIDE);
	gluDisk(quadric, 0.0, radius, subdivisions, 1);
	glPopMatrix();

	gluDeleteQuadric(quadric);
}

void MiniGL::drawSphere(const Vector3r& translation, float radius, float* color, const unsigned int subDivision)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	if (m_sphereQuadric == nullptr)
	{
		m_sphereQuadric = gluNewQuadric();
		gluQuadricNormals(m_sphereQuadric, GLU_SMOOTH);
	}

	glPushMatrix();
	glTranslated((translation)[0], (translation)[1], (translation)[2]);

	gluSphere(m_sphereQuadric, radius, subDivision, subDivision);
	glPopMatrix();
}

void MiniGL::drawPoint(const Vector3r& translation, const float pointSize, const float* const color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glPointSize(pointSize);

	glBegin(GL_POINTS);
	glVertex3v(&translation[0]);
	glEnd();

	glPointSize(1);
}

void MiniGL::drawMesh(const std::vector<Vector3r>& vertices, const std::vector<unsigned int>& faces,
	const std::vector<Vector3r>& vertexNormals, const float* const color)
{
	// draw mesh 
	if (MiniGL::checkOpenGLVersion(3, 3))
	{
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_REAL, GL_FALSE, 0, &vertices[0][0]);
		if (vertexNormals.size() > 0)
		{
			glEnableVertexAttribArray(2);
			glVertexAttribPointer(2, 3, GL_REAL, GL_FALSE, 0, &vertexNormals[0][0]);
		}
	}
	else
	{
		float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0f);
		glColor3fv(color);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_REAL, 0, &vertices[0][0]);
		if (vertexNormals.size() > 0)
		{
			glEnableClientState(GL_NORMAL_ARRAY);
			glNormalPointer(GL_REAL, 0, &vertexNormals[0][0]);
		}
	}

	glDrawElements(GL_TRIANGLES, (GLsizei)faces.size(), GL_UNSIGNED_INT, faces.data());

	if (MiniGL::checkOpenGLVersion(3, 3))
	{
		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(2);
	}
	else
	{
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
	}
}

void MiniGL::drawQuad(const Vector3r& a, const Vector3r& b, const Vector3r& c, const Vector3r& d, const Vector3r& norm, float* color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glBegin(GL_QUADS);
	glNormal3v(&norm[0]);
	glVertex3v(&a[0]);
	glVertex3v(&b[0]);
	glVertex3v(&c[0]);
	glVertex3v(&d[0]);
	glEnd();
}

void MiniGL::drawTriangle(const Vector3r& a, const Vector3r& b, const Vector3r& c, const Vector3r& norm, float* color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glBegin(GL_TRIANGLES);
	glNormal3v(&norm[0]);
	glVertex3v(&a[0]);
	glVertex3v(&b[0]);
	glVertex3v(&c[0]);
	glEnd();
}

void MiniGL::drawTetrahedron(const Vector3r& a, const Vector3r& b, const Vector3r& c, const Vector3r& d, float* color)
{
	Vector3r normal1 = (b - a).cross(c - a);
	Vector3r normal2 = (b - a).cross(d - a);
	Vector3r normal3 = (c - a).cross(d - a);
	Vector3r normal4 = (c - b).cross(d - b);
	drawTriangle(a, b, c, normal1, color);
	drawTriangle(a, b, d, normal2, color);
	drawTriangle(a, c, d, normal3, color);
	drawTriangle(b, c, d, normal4, color);
}

void MiniGL::drawGrid_xz(float* color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	const int size = 5;

	glBegin(GL_LINES);
	for (int i = -size; i <= size; i++)
	{
		glVertex3f((float)i, 0.0f, (float)-size);
		glVertex3f((float)i, 0.0f, (float)size);
		glVertex3f((float)-size, 0.0f, (float)i);
		glVertex3f((float)size, 0.0f, (float)i);
	}
	glEnd();

	glLineWidth(3.0f);
	glBegin(GL_LINES);
	glVertex3f((float)-size, 0.0f, 0.0f);
	glVertex3f((float)size, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, (float)-size);
	glVertex3f(0.0f, 0.0f, (float)size);
	glEnd();
}

void MiniGL::drawGrid_xy(float* color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	const int size = 5;

	glBegin(GL_LINES);
	for (int i = -size; i <= size; i++)
	{
		glVertex3f((float)i, (float)-size, 0.0f);
		glVertex3f((float)i, (float)size, 0.0f);
		glVertex3f((float)-size, (float)i, 0.0f);
		glVertex3f((float)size, (float)i, 0.0f);
	}
	glEnd();

	glLineWidth(3.0f);
	glBegin(GL_LINES);
	glVertex3f((float)-size, 0.0f, 0.0f);
	glVertex3f((float)size, 0.0f, 0.0f);
	glVertex3f(0.0f, (float)-size, 0.0f);
	glVertex3f(0.0f, (float)size, 0.0f);
	glEnd();
}

void MiniGL::setViewport(float pfovy, float pznear, float pzfar, const Vector3r& peyepoint, const Vector3r& plookat)
{
	fovy = pfovy;
	znear = pznear;
	zfar = pzfar;

	glLoadIdentity();
	gluLookAt(peyepoint[0], peyepoint[1], peyepoint[2], plookat[0], plookat[1], plookat[2], 0, 1, 0);

	Matrix4r transformation;
	Real* lookAtMatrix = transformation.data();
	glGetRealv(GL_MODELVIEW_MATRIX, &lookAtMatrix[0]);

	Matrix3r rot;
	Vector3r scale;

	rot.row(0) = Vector3r(transformation(0, 0), transformation(0, 1), transformation(0, 2));
	rot.row(1) = Vector3r(transformation(1, 0), transformation(1, 1), transformation(1, 2));
	rot.row(2) = Vector3r(transformation(2, 0), transformation(2, 1), transformation(2, 2));
	scale[0] = rot.col(0).norm();
	scale[1] = rot.col(1).norm();
	scale[2] = rot.col(2).norm();
	m_translation = Vector3r(transformation(0, 3), transformation(1, 3), transformation(2, 3));

	rot.col(0) = 1.0 / scale[0] * rot.col(0);
	rot.col(1) = 1.0 / scale[1] * rot.col(1);
	rot.col(2) = 1.0 / scale[2] * rot.col(2);

	m_zoom = scale[0];
	m_rotation = Quaternionr(rot);

	glLoadIdentity();
}

void MiniGL::setViewport(float pfovy, float pznear, float pzfar)
{
	fovy = pfovy;
	znear = pznear;
	zfar = pzfar;
}

void MiniGL::setClientSceneFunc(SceneFct func)
{
	scenefunc = func;
}

void MiniGL::init(int argc, char** argv, const int width, const int height, const char* name, const bool vsync, const bool maximized)
{
	fovy = 60;
	znear = 0.5f;
	zfar = 1000;

	m_width = width;
	m_height = height;
	m_vsync = vsync;

	scenefunc = nullptr;

	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	initDevIL();

	glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GL_FALSE);
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	//glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	if (maximized)
		glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

	if (m_vsync)
		glfwWindowHint(GLFW_DOUBLEBUFFER, GL_TRUE);
	else
		glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE);

	m_glfw_window = glfwCreateWindow(width, height, name, NULL, NULL);
	if (!m_glfw_window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(m_glfw_window);
	gladLoadGL(glfwGetProcAddress);
	glfwSwapInterval(0);

	glfwSetFramebufferSizeCallback(m_glfw_window, reshape);

	getOpenGLVersion(m_context_major_version, m_context_minor_version);
	glGetIntegerv(GL_CONTEXT_PROFILE_MASK, &m_context_profile);

	LOG_INFO << "OpenGL version " << m_context_major_version << "." << m_context_minor_version;
	LOG_INFO << "Vendor: " << glGetString(GL_VENDOR);
	LOG_INFO << "Renderer: " << glGetString(GL_RENDERER);
	LOG_INFO << "Version: " << glGetString(GL_VERSION);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glClearColor(0.4f, 0.4f, 0.4f, 1.0f);

	glfwSetKeyCallback(m_glfw_window, keyboard);
	glfwSetCharCallback(m_glfw_window, char_callback);
	glfwSetMouseButtonCallback(m_glfw_window, mousePress);
	glfwSetCursorPosCallback(m_glfw_window, mouseMove);
	glfwSetScrollCallback(m_glfw_window, mouseWheel);

	int w, h;
	glfwGetWindowSize(m_glfw_window, &w, &h);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	m_lastTime = glfwGetTime();
}

//void MiniGL::HandleDevILErrors()
//{
//	ILenum error = ilGetError();
//
//	if (error != IL_NO_ERROR) {
//		do {
//			printf("\n\n%s\n", iluErrorString(error));
//		} while ((error = ilGetError()));
//
//		exit(1);
//	}
//}

void MiniGL::initDevIL()
{
	// Needed to initialize DevIL.
	ilInit();
}

void MiniGL::initTexture()
{
	ILuint	ImgId;
	ILuint	Width, Height, bpp;

	// Generate the main image name to use.
	ilGenImages(1, &ImgId);

	// Bind this image name.
	ilBindImage(ImgId);

	// Loads the image specified by File into the image named by ImgId.
	const std::string texturePath = Utilities::FileSystem::normalizePath(
		Utilities::FileSystem::getProgramPath() + "/resources/models/scene1/Head.bmp");
	if (!ilLoadImage(texturePath.c_str())) {
		if (looksLikeGitLfsPointerFile(texturePath))
			LOG_WARN << "Texture asset is a Git LFS pointer, not image data: " << texturePath
				<< ". Install git-lfs and run 'git lfs pull'. Using fallback texture.";
		else
			LOG_WARN << "Could not load texture: " << texturePath << ". Using fallback texture.";
		const unsigned char fallback[3] = { 180u, 180u, 180u };
		glGenTextures(1, &m_texId);
		glBindTexture(GL_TEXTURE_2D, m_texId);
		glTexImage2D(GL_TEXTURE_2D, 0, 3, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, fallback);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glEnable(GL_TEXTURE_2D);
		glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
		glBindTexture(GL_TEXTURE_2D, 0);
		return;
	}

	Width = ilGetInteger(IL_IMAGE_WIDTH);
	Height = ilGetInteger(IL_IMAGE_HEIGHT);
	bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

	printf("Width: %d  Height: %d  Bpp: %d\n",
		ilGetInteger(IL_IMAGE_WIDTH),
		ilGetInteger(IL_IMAGE_HEIGHT),
		ilGetInteger(IL_IMAGE_BITS_PER_PIXEL));

	ILubyte* pBytes = ilGetData();

	glGenTextures(1, &m_texId);
	glBindTexture(GL_TEXTURE_2D, m_texId);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, Width, Height, 0, GL_BGR,
		GL_UNSIGNED_BYTE, pBytes);  // Create texture from image data
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	glEnable(GL_TEXTURE_2D);  // Enable 2D texture 

	// Correct texture distortion in perpective projection
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glBindTexture(GL_TEXTURE_2D, 0);
}
void MiniGL::initTexture4pic() {
	ILuint ImgId[6];
	ILuint Width, Height, bpp;
	const std::string modelPath = Utilities::FileSystem::normalizePath(
		Utilities::FileSystem::getProgramPath() + "/resources/models");
	const std::string externalModelPath = Utilities::FileSystem::normalizePath(
		Utilities::FileSystem::getProgramPath() + "/../../models");
	const std::string paths[] = {	//皮肤，骨头，脊椎神经，椎间盘,肌肉,韧带
		modelPath + "/scene/Female_Body_Diffuse.bmp",
		modelPath + "/scene/Skeleton_Spine_Diffuse.bmp",
		modelPath + "/scene/Spinal_Cord_Diffuse.bmp",
		modelPath + "/scene/Tissue_Discs_Torso_Diffuse.bmp",
		modelPath + "/scene/Muscles_UpperLimb_Diffuse.bmp",
		modelPath + "/scene/Teeth_Top_Diffuse.bmp"

	};
	const std::string externalPaths[] = {
		externalModelPath + "/scene/Female_Body_Diffuse.bmp",
		externalModelPath + "/scene/Skeleton_Spine_Diffuse.bmp",
		externalModelPath + "/scene/Spinal_Cord_Diffuse.bmp",
		externalModelPath + "/scene/Tissue_Discs_Torso_Diffuse.bmp",
		externalModelPath + "/scene/Muscles_UpperLimb_Diffuse.bmp",
		externalModelPath + "/scene/Teeth_Top_Diffuse.bmp"
	};
		// modelPath + "/scene/Female_Body_Diffuse.bmp",
		// modelPath + "/scene/Skeleton_Spine_Diffuse.bmp",
		// modelPath + "/scene/Spinal_Cord_Diffuse.bmp",
		// modelPath + "/scene/Tissue_Discs_Torso_Diffuse.bmp"
	
	// 遍历加载每个纹理
	ilGenImages(6, ImgId);
	glGenTextures(6, texId);
	for (unsigned int i = 0; i <6; i++) {
		const std::string texturePath =
			Utilities::FileSystem::fileExists(paths[i]) ? paths[i] : externalPaths[i];
		// 生成一个新的图像ID
		ilBindImage(ImgId[i]);

		// 加载图片文件
		if (!ilLoadImage(texturePath.c_str())) {
			if (looksLikeGitLfsPointerFile(texturePath))
				LOG_WARN << "Texture asset is a Git LFS pointer, not image data: " << texturePath
					<< ". Install git-lfs and run 'git lfs pull'. Using fallback texture.";
			else
				LOG_WARN << "Could not load texture: " << texturePath << ". Using fallback texture.";
			const unsigned char fallback[3] = { 180u, 180u, 180u };
			glBindTexture(GL_TEXTURE_2D, texId[i]);
			glTexImage2D(GL_TEXTURE_2D, 0, 3, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, fallback);
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
			continue;
		}

		Width = ilGetInteger(IL_IMAGE_WIDTH);
		Height = ilGetInteger(IL_IMAGE_HEIGHT);
		bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		printf("Texture %d - Width: %d  Height: %d  Bpp: %d\n", i,
			ilGetInteger(IL_IMAGE_WIDTH),
			ilGetInteger(IL_IMAGE_HEIGHT),
			ilGetInteger(IL_IMAGE_BITS_PER_PIXEL));

		ILubyte* pBytes = ilGetData();

		// 为每个图片生成一个纹理ID
		
		glBindTexture(GL_TEXTURE_2D, texId[i]);
		//if(i==2)glTexImage2D(GL_TEXTURE_2D, 0, 3, Width, Height, 0, GL_RGB, GL_UNSIGNED_BYTE, pBytes);
		glTexImage2D(GL_TEXTURE_2D, 0, 3, Width, Height, 0, GL_BGR, GL_UNSIGNED_BYTE, pBytes);//GL_BGR

		// 设置纹理参数
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	}
	glEnable(GL_TEXTURE_2D);  // Enable 2D texture 

	// Correct texture distortion in perpective projection
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glBindTexture(GL_TEXTURE_2D, 0);
}

void MiniGL::setMouseMoveFunc(int button, void(*func) (int, int, void*))
{
	mousefunc = func;
	mouseFuncButton = button;
}


void MiniGL::setSelectionFunc(void(*func) (const Vector2i&, const Vector2i&, void*), void* clientData)
{
	selectionfunc = func;
	selectionfuncClientData = clientData;
}

void MiniGL::setHapticSelectionFunc(void(*func) (const Vector3r&, void*), void* clientData)
{
	hapticselectionfunc = func;
	selectionfuncClientData = clientData;
}

void MiniGL::destroy()
{
	if (m_sphereQuadric != nullptr)
	{
		gluDeleteQuadric(m_sphereQuadric);
		m_sphereQuadric = nullptr;
	}
}

void MiniGL::shutdown()
{
	if (destroyfunc != nullptr)
		destroyfunc();
	destroy();
	if (m_glfw_window != nullptr)
	{
		glfwDestroyWindow(m_glfw_window);
		m_glfw_window = nullptr;
	}
	glfwTerminate();
}

bool MiniGL::writeFramebufferPPM(const std::string& path, unsigned int& width, unsigned int& height, unsigned int& nonBackgroundPixelCount)
{
	width = 0u;
	height = 0u;
	nonBackgroundPixelCount = 0u;
	if (m_glfw_window == nullptr)
		return false;

	int framebufferWidth = 0;
	int framebufferHeight = 0;
	glfwGetFramebufferSize(m_glfw_window, &framebufferWidth, &framebufferHeight);
	if ((framebufferWidth <= 0) || (framebufferHeight <= 0))
		return false;

	width = static_cast<unsigned int>(framebufferWidth);
	height = static_cast<unsigned int>(framebufferHeight);
	std::vector<unsigned char> pixels(static_cast<size_t>(framebufferWidth) * static_cast<size_t>(framebufferHeight) * 3u);
	while (glGetError() != GL_NO_ERROR)
	{
	}
	glFinish();
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadBuffer(m_vsync ? GL_BACK : GL_FRONT);
	glReadPixels(0, 0, framebufferWidth, framebufferHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
	const GLenum readError = glGetError();
	if (readError != GL_NO_ERROR)
		return false;

	const unsigned char background = static_cast<unsigned char>(0.4f * 255.0f);
	for (size_t i = 0; i + 2u < pixels.size(); i += 3u)
	{
		const int dr = std::abs(static_cast<int>(pixels[i]) - static_cast<int>(background));
		const int dg = std::abs(static_cast<int>(pixels[i + 1u]) - static_cast<int>(background));
		const int db = std::abs(static_cast<int>(pixels[i + 2u]) - static_cast<int>(background));
		if ((dr + dg + db) > 24)
			nonBackgroundPixelCount++;
	}

	std::ofstream out(path.c_str(), std::ios::binary);
	if (!out)
		return false;
	out << "P6\n" << framebufferWidth << " " << framebufferHeight << "\n255\n";
	for (int y = framebufferHeight - 1; y >= 0; --y)
	{
		const size_t rowOffset = static_cast<size_t>(y) * static_cast<size_t>(framebufferWidth) * 3u;
		out.write(reinterpret_cast<const char*>(pixels.data() + rowOffset), static_cast<std::streamsize>(framebufferWidth * 3));
	}
	return static_cast<bool>(out);
}

void MiniGL::reshape(GLFWwindow* glfw_window, int w, int h)
{
	if ((w > 0) && (h > 0))
	{
		m_width = w;
		m_height = h;

		for (auto i = 0; i < m_reshapeFct.size(); i++)
			m_reshapeFct[i](m_width, m_height);
		glViewport(0, 0, m_width, m_height);
	}
}

void MiniGL::setClientIdleFunc(IdleFct func)
{
	idlefunc = func;
}

void MiniGL::setClientDestroyFunc(DestroyFct func)
{
	destroyfunc = func;
}

void MiniGL::addKeyFunc(unsigned char k, std::function<void()> const& func)
{
	if (func == nullptr)
		return;
	else
		keyfunc.push_back({ func, k });
}

void MiniGL::keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	// Check if registered listener wants the event
	for (auto i = 0; i < m_keyboardFct.size(); i++)
	{
		if (m_keyboardFct[i](key, scancode, action, mods))
			return;
	}

	if (key == GLFW_KEY_ESCAPE)
	{
		m_breakPointLoop = false;
		m_breakPointActive = false;
#ifndef __APPLE__
		leaveMainLoop();
#else
		exit(0);
#endif
		return;
	}
	else if (key == GLFW_KEY_A)
		move(0, 0, movespeed);
	else if (key == GLFW_KEY_Y)
		move(0, 0, -movespeed);
	else if (key == GLFW_KEY_UP)
		move(0, -movespeed, 0);
	else if (key == GLFW_KEY_DOWN)
		move(0, movespeed, 0);
	else if (key == GLFW_KEY_LEFT)
		move(movespeed, 0, 0);
	else if (key == GLFW_KEY_RIGHT)
		move(-movespeed, 0, 0);
	else if (key == GLFW_KEY_F5)
		m_breakPointLoop = false;
}

void MiniGL::char_callback(GLFWwindow* window, unsigned int codepoint)
{
	// Check if registered listener wants the event
	for (auto i = 0; i < m_charFct.size(); i++)
	{
		if (m_charFct[i](codepoint, GLFW_PRESS))
			return;
	}

	for (int i = 0; i < keyfunc.size(); i++)
	{
		if (codepoint == keyfunc[i].key)
			keyfunc[i].fct();
		else if (codepoint == GLFW_KEY_1)
			rotateX(-turnspeed);
		else if (codepoint == GLFW_KEY_2)
			rotateX(turnspeed);
		else if (codepoint == GLFW_KEY_3)
			rotateY(-turnspeed);
		else if (codepoint == GLFW_KEY_4)
			rotateY(turnspeed);
	}
}

void MiniGL::setProjectionMatrix(int width, int height)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fovy, (Real)width / (Real)height, znear, zfar);
}

void MiniGL::viewport()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glRenderMode(GL_RENDER);
	glfwGetFramebufferSize(m_glfw_window, &m_width, &m_height);
	glViewport(0, 0, m_width, m_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	setProjectionMatrix(m_width, m_height);
	glMatrixMode(GL_MODELVIEW);

	glTranslatef((float)m_translation[0], (float)m_translation[1], (float)m_translation[2]);
	Matrix3r rot;
	rot = m_rotation.toRotationMatrix();
	Matrix4r transform(Matrix4r::Identity());
	Vector3r scale(m_zoom, m_zoom, m_zoom);
	transform.block<3, 3>(0, 0) = rot;
	transform.block<3, 1>(0, 3) = m_translation;
	transform(0, 0) *= scale[0];
	transform(1, 1) *= scale[1];
	transform(2, 2) *= scale[2];
	Real* transformMatrix = transform.data();
	glLoadMatrix(&transformMatrix[0]);
}

void MiniGL::initLights()
{
	float t = 0.9f;
	float a = 0.2f;
	float amb0[4] = { a,a,a,1 };
	float diff0[4] = { t,0,0,1 };
	float spec0[4] = { 1,1,1,1 };
	float pos0[4] = { -10,10,10,1 };
	glLightfv(GL_LIGHT0, GL_AMBIENT, amb0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diff0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, spec0);
	glLightfv(GL_LIGHT0, GL_POSITION, pos0);
	glEnable(GL_LIGHT0);

	float amb1[4] = { a,a,a,1 };
	float diff1[4] = { 0,0,t,1 };
	float spec1[4] = { 1,1,1,1 };
	float pos1[4] = { 10,10,10,1 };
	glLightfv(GL_LIGHT1, GL_AMBIENT, amb1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diff1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, spec1);
	glLightfv(GL_LIGHT1, GL_POSITION, pos1);
	glEnable(GL_LIGHT1);

	float amb2[4] = { a,a,a,1 };
	float diff2[4] = { 0,t,0,1 };
	float spec2[4] = { 1,1,1,1 };
	float pos2[4] = { 0,10,10,1 };
	glLightfv(GL_LIGHT2, GL_AMBIENT, amb2);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, diff2);
	glLightfv(GL_LIGHT2, GL_SPECULAR, spec2);
	glLightfv(GL_LIGHT2, GL_POSITION, pos2);
	glEnable(GL_LIGHT2);


	glEnable(GL_LIGHTING);
	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

}

void MiniGL::move(Real x, Real y, Real z)
{
	m_translation[0] += x;
	m_translation[1] += y;
	m_translation[2] += z;
}

void MiniGL::rotateY(Real y)
{
	AngleAxisr angleAxis(y, Vector3r(0, 1, 0));
	Quaternionr quat(angleAxis);
	m_rotation = m_rotation * quat;
}

void MiniGL::rotateX(Real x)
{
	AngleAxisr angleAxis(x, Vector3r(1, 0, 0));
	Quaternionr quat(angleAxis);
	m_rotation = quat * m_rotation;
}

void MiniGL::mousePress(GLFWwindow* window, int button, int action, int mods)
{
	//getting cursor position
	glfwGetCursorPos(m_glfw_window, &mouse_pos_x_old, &mouse_pos_y_old);

	// Check if registered listener wants the event
	for (auto i = 0; i < m_mousePressFct.size(); i++)
	{
		if (m_mousePressFct[i](button, action, mods))
			return;
	}

	modifier_key = mods;

	if (action == GLFW_PRESS)
		mouse_button = button;
	else
		mouse_button = -1;

	if ((selectionfunc != NULL) && (modifier_key == 0))
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			if (action == GLFW_PRESS)
				m_selectionStart = Vector2i(mouse_pos_x_old, mouse_pos_y_old);
			else
			{
				if (m_selectionStart[0] != -1)
				{
					const Vector2i pos(mouse_pos_x_old, mouse_pos_y_old);
					selectionfunc(m_selectionStart, pos, selectionfuncClientData);
				}
				m_selectionStart = Vector2i(-1, -1);
			}
		}
	}
}

void MiniGL::mouseWheel(GLFWwindow* window, double xoffset, double yoffset)
{
	mouse_wheel_pos += yoffset;

	// Check if registered listener wants the event
	for (auto i = 0; i < m_mouseWheelFct.size(); i++)
	{
		if (m_mouseWheelFct[i](static_cast<int>(mouse_wheel_pos), xoffset, yoffset))
			return;
	}

	if (yoffset > 0)
		movespeed *= 2.0;
	else
		movespeed *= 0.5;
}

void MiniGL::mouseMove(GLFWwindow* window, double x, double y)
{
	// Check if registered listener wants the event
	for (auto i = 0; i < m_mouseMoveFct.size(); i++)
	{
		if (m_mouseMoveFct[i](static_cast<int>(x), static_cast<int>(y)))
			return;
	}

	double d_x = mouse_pos_x_old - x;
	double d_y = y - mouse_pos_y_old;

	if (mouse_button == GLFW_MOUSE_BUTTON_1)
	{
		// translate scene in z direction		
		if (modifier_key == GLFW_MOD_CONTROL)
		{
			move(0, 0, -static_cast<Real>(d_x + d_y) / static_cast<Real>(10.0));
		}
		// translate scene in x/y direction
		else if (modifier_key == GLFW_MOD_SHIFT)
		{
			move(-static_cast<Real>(d_x) / static_cast<Real>(20.0), -static_cast<Real>(d_y) / static_cast<Real>(20.0), 0);
		}
		// rotate scene around x, y axis
		else if (modifier_key == GLFW_MOD_ALT)
		{
			rotateX(static_cast<Real>(d_y) / static_cast<Real>(100.0));
			rotateY(-static_cast<Real>(d_x) / static_cast<Real>(100.0));
		}
	}

	if (mousefunc != NULL)
	{
		if ((mouseFuncButton == -1) || (mouseFuncButton == mouse_button))
			mousefunc(static_cast<int>(x), static_cast<int>(y), selectionfuncClientData);
	}

	mouse_pos_x_old = x;
	mouse_pos_y_old = y;
}

#ifdef PBD_ENABLE_HAPTICS
/*******************************************************************************
 Starts manipulation upon button press.
*******************************************************************************/
void HLCALLBACK MiniGL::hlButtonDownCB(HLenum event, HLuint object, HLenum thread, HLcache* cache, void* userdata)
{
	hlAddEventCallback(HL_EVENT_MOTION, HL_OBJECT_ANY,
		HL_CLIENT_THREAD, &hlMotionCB, NULL);

	std::cout << "hlButtonDown" << std::endl;

	if (event == HL_EVENT_1BUTTONDOWN)
		gButton1DownState = true;
	else if (event == HL_EVENT_2BUTTONDOWN)
		gButton2DownState = true;
	gButtonDownState = DemoHaptics::liveHapticSelectionStateFromButtons(
		gButton1DownState,
		gButton2DownState);
	hlGetDoublev(HL_PROXY_POSITION, gStartDragProxyPos);
	hlGetDoublev(HL_PROXY_ROTATION, gStartDragProxyRot);

	if (hapticselectionfunc != NULL)
	{
		// Get the proxy transform in world coordinates.
		HLdouble proxyxform[16];
		hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);
		DemoHaptics::scaleLiveHapticProxyTransformTranslation(proxyxform, gHapticWorkspaceScale);
		gHapticPos = DemoHaptics::liveHapticProxyTransformTranslation(proxyxform);

		hapticselectionfunc(gHapticPos, selectionfuncClientData);

		//选中周围粒子

	}
	/*HapticManager* pThis = static_cast<HapticManager*>(userdata);

	int nIndex = pThis->getPointIndexFromTouchId(object);

	if (!pThis->isManipulating())
	{
		pThis->startManipulating(nIndex);
	}*/
}


/*******************************************************************************
 Stops manipulation upon button release.
*******************************************************************************/
void HLCALLBACK MiniGL::hlButtonUpCB(HLenum event, HLuint object, HLenum thread, HLcache* cache, void* userdata)
{
	hlRemoveEventCallback(HL_EVENT_MOTION, HL_OBJECT_ANY,
		HL_CLIENT_THREAD, hlMotionCB);

	std::cout << "hlButtonUp" << std::endl;
	
	if (event == HL_EVENT_1BUTTONUP)
		gButton1DownState = false;
	else if (event == HL_EVENT_2BUTTONUP)
		gButton2DownState = false;
	gButtonDownState = DemoHaptics::liveHapticSelectionStateFromButtons(
		gButton1DownState,
		gButton2DownState);
	/*HapticManager* pThis = static_cast<HapticManager*>(userdata);

	if (pThis->isManipulating())
	{
		pThis->stopManipulating();
	}*/
}


/*******************************************************************************
 Called whenever the device position changes.
*******************************************************************************/
void HLCALLBACK MiniGL::hlMotionCB(HLenum event, HLuint object, HLenum thread, HLcache* cache, void* userdata)
{
	hlGetDoublev(HL_PROXY_TRANSFORM, gHapticXform);
	DemoHaptics::scaleLiveHapticProxyTransformTranslation(gHapticXform, gHapticWorkspaceScale);
	gHapticPos = DemoHaptics::liveHapticProxyTransformTranslation(gHapticXform);

	// First update the tranform matrix and then update the start pos and rot
	updateDeltaTransformMat();

	//hlGetDoublev(HL_PROXY_POSITION, gStartDragProxyPos);
	//hlGetDoublev(HL_PROXY_ROTATION, gStartDragProxyRot);

	//std::cout << "hlMotion to pos [" << proxyxform[12] <<", " << proxyxform[13] << ", " << proxyxform[14] << "]" << std::endl;
	/*HapticManager* pThis = static_cast<HapticManager*>(userdata);

	if (pThis->isManipulating())
	{
		// Get the position of the proxy when the motion was detected.
		hduVector3Dd proxyPos;
		hlCacheGetDoublev(cache, HL_PROXY_POSITION, proxyPos);

		pThis->updateManipPoint(proxyPos);
	}*/
}


#endif

void MiniGL::unproject(const int x, const int y, Vector3r& pos)
{
	GLint viewport[4];
	GLdouble mv[16], pm[16];


	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, mv);
	glGetDoublev(GL_PROJECTION_MATRIX, pm);

	GLdouble resx, resy, resz;
	gluUnProject(x, viewport[3] - y, znear, mv, pm, viewport, &resx, &resy, &resz);
	pos[0] = (Real)resx;
	pos[1] = (Real)resy;
	pos[2] = (Real)resz;
}

float MiniGL::getZNear()
{
	return znear;
}

float MiniGL::getZFar()
{
	return zfar;
}

void MiniGL::hsvToRgb(float h, float s, float v, float* rgb)
{
	int i = (int)floor(h * 6);
	float f = h * 6 - i;
	float p = v * (1 - s);
	float q = v * (1 - f * s);
	float t = v * (1 - (1 - f) * s);

	switch (i % 6)
	{
	case 0: rgb[0] = v, rgb[1] = t, rgb[2] = p; break;
	case 1: rgb[0] = q, rgb[1] = v, rgb[2] = p; break;
	case 2: rgb[0] = p, rgb[1] = v, rgb[2] = t; break;
	case 3: rgb[0] = p, rgb[1] = q, rgb[2] = v; break;
	case 4: rgb[0] = t, rgb[1] = p, rgb[2] = v; break;
	case 5: rgb[0] = v, rgb[1] = p, rgb[2] = q; break;
	}
}

bool MiniGL::checkOpenGLVersion(const int major_version, const int minor_version)
{
	if ((m_context_major_version > major_version) ||
		((m_context_major_version == major_version) && (m_context_minor_version >= minor_version)))
		return true;
	return false;
}

Shader* MiniGL::createShader(const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader)
{
	if (checkOpenGLVersion(3, 3))
	{
		Shader* shader = new Shader();

		if (vertexShader != "")
			shader->compileShaderFile(GL_VERTEX_SHADER, vertexShader);
		if (geometryShader != "")
			shader->compileShaderFile(GL_GEOMETRY_SHADER, geometryShader);
		if (fragmentShader != "")
			shader->compileShaderFile(GL_FRAGMENT_SHADER, fragmentShader);
		shader->createAndLinkProgram();
		return shader;
	}
	return NULL;
}

void MiniGL::drawElements()
{
	for (unsigned int i = 0; i < m_drawLines.size(); i++)
	{
		Line& l = m_drawLines[i];
		drawVector(l.a, l.b, l.lineWidth, l.color);
	}
	for (unsigned int i = 0; i < m_drawTriangle.size(); i++)
	{
		Triangle& t = m_drawTriangle[i];
		Vector3r n = ((t.b - t.a).cross(t.c - t.a));
		n.normalize();
		drawTriangle(t.a, t.b, t.c, n, t.color);
	}
	for (unsigned int i = 0; i < m_drawPoints.size(); i++)
	{
		Point& p = m_drawPoints[i];
		drawSphere(p.a, p.pointSize, p.color);
	}

	//drawHapticCursor();
}

void MiniGL::setBreakPointActive(const bool active)
{
	m_breakPointActive = active;
}

void MiniGL::breakPoint()
{
	breakPointMainLoop();
}

void MiniGL::error_callback(int error, const char* description)
{
	LOG_ERR << description;
}

void MiniGL::mainLoop()
{
	auto& mt = MyTimer::GetInstance();
	mt.InitAndStart();

	while (!glfwWindowShouldClose(m_glfw_window))
	{
		char info[MAX_PATH] = { 0 };
		sprintf_s(info, "Virtual eye surgery simulation system--Simulation FPS: %3.2f Hz, Haptic FPS: %3.2f Hz", mt.GetFPS(), 1000.00f);

		glfwSetWindowTitle(m_glfw_window, info);

		if (idlefunc != nullptr)
			idlefunc();

		double currentTime = glfwGetTime();
		if (currentTime - m_lastTime >= 1.0 / 60.0)  // render at maximum at 60 fps
		{
			glfwPollEvents();

			glPolygonMode(GL_FRONT_AND_BACK, drawMode);
			viewport();

			updateHapticMapping();

			drawElements();

			drawHapticObjects();

			if (scenefunc != nullptr)
				scenefunc();

			if (m_vsync)
				glfwSwapBuffers(m_glfw_window);
			else
				glFlush();
			m_lastTime = currentTime;
		}

		mt.UpdatePerFrame();
	}

	if (destroyfunc != nullptr)
		destroyfunc();

	destroy();

	glfwDestroyWindow(m_glfw_window);

	glfwTerminate();
}

void MiniGL::leaveMainLoop()
{
	exitHandler();
	glfwSetWindowShouldClose(m_glfw_window, 1);
}

void MiniGL::swapBuffers()
{
	if (m_vsync)
		glfwSwapBuffers(m_glfw_window);
	else
		glFlush();
}

void MiniGL::getWindowPos(int& x, int& y)
{
	glfwGetWindowPos(m_glfw_window, &x, &y);
}

void MiniGL::getWindowSize(int& w, int& h)
{
	glfwGetWindowSize(m_glfw_window, &w, &h);
}

void MiniGL::setWindowPos(int x, int y)
{
	glfwSetWindowPos(m_glfw_window, x, y);
}

void MiniGL::setWindowSize(int w, int h)
{
	glfwSetWindowSize(m_glfw_window, w, h);
}

bool MiniGL::getWindowMaximized()
{
	return glfwGetWindowAttrib(m_glfw_window, GLFW_MAXIMIZED);
}

void MiniGL::setWindowMaximized(const bool b)
{
	if (b)
		glfwRestoreWindow(m_glfw_window);
	else
		glfwRestoreWindow(m_glfw_window);
}

void MiniGL::breakPointMainLoop()
{
	if (m_breakPointActive)
	{
		m_breakPointLoop = true;
		while (m_breakPointLoop)
		{
			glPolygonMode(GL_FRONT_AND_BACK, drawMode);
			viewport();

			if (scenefunc != nullptr)
				scenefunc();

			if (m_vsync)
				glfwSwapBuffers(m_glfw_window);
			else
				glFlush();
			glfwPollEvents();
		}
	}
}


#ifdef PBD_ENABLE_HAPTICS
/*******************************************************************************
 Sets up and initializes haptic rendering library.
*******************************************************************************/
void MiniGL::initHL()
{
	LOG_INFO << "OpenHaptics support compiled in: PBD_ENABLE_HAPTICS=ON";
	gHapticAvailable = false;
	gButtonDownState = false;
	gButton1DownState = false;
	gButton2DownState = false;
	gHapticPos = Vector3r::Zero();
	gDeltaTMat.makeIdentity();

	HDErrorInfo error;
	ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
		LOG_ERR << "Failed to initialize haptic device: " << error;
		ghHD = HD_INVALID_HANDLE;
		return;
	}

	// Create a haptic context for the device.  The haptic context maintains 
	// the state that persists between frame intervals and is used for
	// haptic rendering.
	ghHLRC = hlCreateContext(ghHD);
	if (ghHLRC == NULL)
	{
		error = hdGetError();
		hduPrintError(stderr, &error, "Failed to create haptic context");
		LOG_ERR << "Failed to create haptic context: " << error;
		if (ghHD != HD_INVALID_HANDLE)
		{
			hdDisableDevice(ghHD);
			ghHD = HD_INVALID_HANDLE;
		}
		return;
	}
	hlMakeCurrent(ghHLRC);
	gHapticAvailable = true;
	LOG_INFO << "OpenHaptics device initialized successfully";

	// Enable optimization of the viewing parameters when rendering
	// geometry for OpenHaptics.
	hlEnable(HL_HAPTIC_CAMERA_VIEW);

	// Generate a shape id to hold the axis snap constraint.
	//gAxisId = hlGenShapes(1);
	//gSphereShapeId = hlGenShapes(1);

	// Add a callback to handle button down in the collision thread.
	//hlAddEventCallback(HL_EVENT_1BUTTONDOWN, HL_OBJECT_ANY, HL_COLLISION_THREAD,
	//	buttonDownCollisionThreadCallback, NULL);
	hlAddEventCallback(HL_EVENT_1BUTTONDOWN, HL_OBJECT_ANY, HL_CLIENT_THREAD, &MiniGL::hlButtonDownCB, NULL);
	hlAddEventCallback(HL_EVENT_1BUTTONUP, HL_OBJECT_ANY, HL_CLIENT_THREAD, &MiniGL::hlButtonUpCB, NULL);
	hlAddEventCallback(HL_EVENT_2BUTTONDOWN, HL_OBJECT_ANY, HL_CLIENT_THREAD, &MiniGL::hlButtonDownCB, NULL);
	hlAddEventCallback(HL_EVENT_2BUTTONUP, HL_OBJECT_ANY, HL_CLIENT_THREAD, &MiniGL::hlButtonUpCB, NULL);
	//hlAddEventCallback(HL_EVENT_1BUTTONDOWN, HL_OBJECT_ANY, HL_CLIENT_THREAD, &button1DownCallback, NULL);
	//hlAddEventCallback(HL_EVENT_1BUTTONUP, HL_OBJECT_ANY, HL_CLIENT_THREAD, &button1UpCallback, NULL);

	// Start an ambient drag friction effect.
	m_effectName = hlGenEffects(1);

	gDeltaTMat.makeIdentity();

	hlBeginFrame();
	hlEffectd(HL_EFFECT_PROPERTY_GAIN, 0.5);
	hlEffectd(HL_EFFECT_PROPERTY_MAGNITUDE, 0.15);
	hlStartEffect(HL_EFFECT_FRICTION, m_effectName);
	hlEndFrame();
}

void MiniGL::setHapticWorkspaceScale(const double scale)
{
	if (scale < 0.2)
		gHapticWorkspaceScale = 0.2;
	else if (scale > 5.0)
		gHapticWorkspaceScale = 5.0;
	else
		gHapticWorkspaceScale = scale;
}

bool MiniGL::refreshHapticButtonState()
{
	if (!gHapticAvailable)
		return false;

	hlCheckEvents();

	HLboolean button1Down = HL_FALSE;
	HLboolean button2Down = HL_FALSE;
	hlGetBooleanv(HL_BUTTON1_STATE, &button1Down);
	const HLerror button1Error = hlGetError();
	hlGetBooleanv(HL_BUTTON2_STATE, &button2Down);
	const HLerror button2Error = hlGetError();
	if (!HL_ERROR(button1Error) && !HL_ERROR(button2Error))
	{
		gButton1DownState = (button1Down == HL_TRUE);
		gButton2DownState = (button2Down == HL_TRUE);
		gButtonDownState = DemoHaptics::liveHapticSelectionStateFromButtons(
			gButton1DownState,
			gButton2DownState);
	}
	return gButtonDownState;
}

/*******************************************************************************
 Cleanup.
*******************************************************************************/
void MiniGL::exitHandler()
{
	if (!gHapticAvailable)
		return;

	hlBeginFrame();
	hlStopEffect(m_effectName);
	hlEndFrame();

	hlDeleteEffects(m_effectName, 1);

	hlRemoveEventCallback(HL_EVENT_1BUTTONDOWN, HL_OBJECT_ANY, HL_CLIENT_THREAD, hlButtonDownCB);
	hlRemoveEventCallback(HL_EVENT_1BUTTONUP, HL_OBJECT_ANY, HL_CLIENT_THREAD, hlButtonUpCB);
	hlRemoveEventCallback(HL_EVENT_2BUTTONDOWN, HL_OBJECT_ANY, HL_CLIENT_THREAD, hlButtonDownCB);
	hlRemoveEventCallback(HL_EVENT_2BUTTONUP, HL_OBJECT_ANY, HL_CLIENT_THREAD, hlButtonUpCB);

	// Free up the haptic rendering context.
	hlMakeCurrent(NULL);
	if (ghHLRC != NULL)
	{
		hlDeleteContext(ghHLRC);
	}

	// Free up the haptic device.
	if (ghHD != HD_INVALID_HANDLE)
	{
		hdDisableDevice(ghHD);
	}
	gHapticAvailable = false;
}

/*******************************************************************************
 Use the current OpenGL viewing transforms to initialize a transform for the
 haptic device workspace so that it's properly mapped to world coordinates.
*******************************************************************************/
void MiniGL::updateHapticMapping(void)
{
	if (!gHapticAvailable)
		return;

	GLdouble modelview[16];
	GLdouble projection[16];
	GLint viewport[4];

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	hlMatrixMode(HL_TOUCHWORKSPACE);
	hlLoadIdentity();

	// Fit haptic workspace to view volume.
	hluFitWorkspace(projection);

	hlGetDoublev(HL_PROXY_TRANSFORM, gHapticXform);
	DemoHaptics::scaleLiveHapticProxyTransformTranslation(gHapticXform, gHapticWorkspaceScale);
	gHapticPos = DemoHaptics::liveHapticProxyTransformTranslation(gHapticXform);
	refreshHapticButtonState();

	// Compute cursor scale.
	gCursorScale = hluScreenToModelScale(modelview, projection, (HLint*)viewport);
	gCursorScale *= CURSOR_SIZE_PIXELS;
}

void MiniGL::drawHapticObjects()
{
	if (!gHapticAvailable)
		return;

	// Start haptic frame.  (Must do this before rendering any haptic shapes.)
	hlBeginFrame();

	// Set material properties for the shapes to be drawn.
	/*hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.7f);
	hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.1f);
	hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.2f);
	hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION, 0.3f);

	// Start a new haptic shape.  Use the feedback buffer to capture OpenGL
	// geometry for haptic rendering.
	hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, gSphereShapeId);

	// Use OpenGL commands to create geometry.
	glutSolidSphere(0.5, 32, 32);

	// End the shape.
	hlEndShape();*/

	// End the haptic frame.
	hlEndFrame();

	hlCheckEvents();
}

/*******************************************************************************
 Draws a 3D cursor for the haptic device using the current local transform,
 the workspace to world transform and the screen coordinate scale.
 *******************************************************************************/
void MiniGL::drawHapticCursor()
{
	if (!gHapticAvailable)
		return;

	static const double kCursorRadius = 0.5;
	static const double kCursorHeight = 1.5;
	static const int kCursorTess = 15;
	HLdouble proxyxform[16];

	GLUquadricObj* qobj = 0;

	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
	glPushMatrix();

	if (!gCursorDisplayList)
	{
		gCursorDisplayList = glGenLists(1);
		glNewList(gCursorDisplayList, GL_COMPILE);
		qobj = gluNewQuadric();

		gluCylinder(qobj, 0.0, kCursorRadius, kCursorHeight,
			kCursorTess, kCursorTess);
		glTranslated(0.0, 0.0, kCursorHeight);
		gluCylinder(qobj, kCursorRadius, 0.0, kCursorHeight / 5.0,
			kCursorTess, kCursorTess);

		gluDeleteQuadric(qobj);
		glEndList();
	}

	// Get the proxy transform in world coordinates.
	hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);
	DemoHaptics::scaleLiveHapticProxyTransformTranslation(proxyxform, gHapticWorkspaceScale);
	glMultMatrixd(proxyxform);

	/*std::cout << std::endl << proxyxform[0] << ", " << proxyxform[1] << ", " << proxyxform[2] << ", " << proxyxform[3] << std::endl
		<< proxyxform[4] << ", " << proxyxform[5] << ", " << proxyxform[6] << ", " << proxyxform[7] << std::endl
		<< proxyxform[8] << ", " << proxyxform[9] << ", " << proxyxform[10] << ", " << proxyxform[11] << std::endl
		<< proxyxform[12] << ", " << proxyxform[13] << ", " << proxyxform[14] << ", " << proxyxform[15] << std::endl;*/

		// Apply the local cursor scale factor.
		//glScaled(gCursorScale, gCursorScale, gCursorScale);

	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(1.0, 1.0, 0.0);
	glCallList(gCursorDisplayList);
	glPopMatrix();

	/*glPushMatrix();
	glMultMatrixd(proxyxform);

	glColor3f(0.0, 1.0, 0.0);
	glPointSize(30);
	glBegin(GL_POINTS);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(1.0, 0.0, 0.0);
	glVertex3f(2.0, 0.0, 0.0);
	glEnd();
	glPopMatrix();*/

	//glPopMatrix();
	glPopAttrib();
}

void MiniGL::drawSurgTool(const std::vector<Vector3r>& vertices, const std::vector<unsigned int>& faces, const std::vector<Vector3r>& vertexNormals, const float* const color)
{
	if (!gHapticAvailable)
	{
		drawMesh(vertices, faces, vertexNormals, color);
		return;
	}

	HLdouble proxyxform[16];

	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
	glPushMatrix();

	// Get the proxy transform in world coordinates.
	hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);
	DemoHaptics::scaleLiveHapticProxyTransformTranslation(proxyxform, gHapticWorkspaceScale);
	glMultMatrixd(proxyxform);

	drawMesh(vertices, faces, vertexNormals, color);

	glPopMatrix();
	glPopAttrib();
}

/******************************************************************************
 Calculates updated object transform for drag object based on changes to
 proxy transform.
******************************************************************************/
void MiniGL::updateDeltaTransformMat()
{
	if (!gHapticAvailable)
	{
		gDeltaTMat.makeIdentity();
		return;
	}

	// Calculated delta between current proxy pos and proxy pos at start of drag.
	hduVector3Dd proxyPos;
	hlGetDoublev(HL_PROXY_POSITION, proxyPos);
	hduVector3Dd dragDeltaTransl = proxyPos - gStartDragProxyPos;

	//std::cout << "Haptic Delta pos : [" << dragDeltaTransl[0] << ", " << dragDeltaTransl[1] << ", " << dragDeltaTransl[2] << "]" << std::endl;

	// Same for rotation.
	hduMatrix deltaRotMat;
	hduQuaternion proxyRotq;
	hlGetDoublev(HL_PROXY_ROTATION, proxyRotq);
	hduQuaternion dragDeltaRot = gStartDragProxyRot.inverse() * proxyRotq;
	dragDeltaRot.normalize();
	dragDeltaRot.toRotationMatrix(deltaRotMat);

	// Want to rotate about the proxy position, not the origin,
	// so need to translate to/from proxy pos.
	hduMatrix toProxy = hduMatrix::createTranslation(-gStartDragProxyPos);
	hduMatrix fromProxy = hduMatrix::createTranslation(gStartDragProxyPos);
	deltaRotMat = toProxy * deltaRotMat * fromProxy;

	// Compose rotation and translation deltas.
	gDeltaTMat = deltaRotMat * hduMatrix::createTranslation(dragDeltaTransl);
}


#else

void MiniGL::initHL()
{
	LOG_INFO << "OpenHaptics support compiled in: PBD_ENABLE_HAPTICS=OFF";
	gHapticAvailable = false;
	gButtonDownState = false;
	gButton1DownState = false;
	gButton2DownState = false;
	gHapticPos = Vector3r::Zero();
	gDeltaTMat.makeIdentity();
}

void MiniGL::setHapticWorkspaceScale(const double scale)
{
	if (scale < 0.2)
		gHapticWorkspaceScale = 0.2;
	else if (scale > 5.0)
		gHapticWorkspaceScale = 5.0;
	else
		gHapticWorkspaceScale = scale;
}

bool MiniGL::refreshHapticButtonState()
{
	gButtonDownState = false;
	gButton1DownState = false;
	gButton2DownState = false;
	return false;
}

void MiniGL::exitHandler()
{
}

void MiniGL::updateHapticMapping(void)
{
}

void MiniGL::drawHapticObjects()
{
}

void MiniGL::drawHapticCursor()
{
}

void MiniGL::drawSurgTool(const std::vector<Vector3r>& vertices, const std::vector<unsigned int>& faces, const std::vector<Vector3r>& vertexNormals, const float* const color)
{
	drawMesh(vertices, faces, vertexNormals, color);
}

void MiniGL::updateDeltaTransformMat()
{
	gDeltaTMat.makeIdentity();
}

#endif
