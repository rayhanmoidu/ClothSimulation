#include "Canvas.hpp"
#include <iostream>
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

Canvas::Canvas() {
}

void Canvas::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    float threshold = 0.5;
    if (key == GLFW_KEY_LEFT)
        camX -= threshold;
    if (key == GLFW_KEY_RIGHT)
        camX += threshold;
    if (key == GLFW_KEY_UP)
        camY -= threshold;
    if (key == GLFW_KEY_DOWN)
        camY += threshold;
    if (key == GLFW_KEY_W)
        camZ -= threshold;
    if (key == GLFW_KEY_S)
        camZ += threshold;
    if (key == GLFW_KEY_P)
        lookAtX -= threshold;
    if (key == GLFW_KEY_L)
        lookAtX += threshold;
    if (key == GLFW_KEY_O)
        lookAtY -= threshold;
    if (key == GLFW_KEY_K)
        lookAtY += threshold;
    if (key == GLFW_KEY_I)
        lookAtZ -= threshold;
    if (key == GLFW_KEY_J)
        lookAtZ += threshold;
}

float Canvas::camX = 0;
float Canvas::camY = 0;
float Canvas::camZ = 0;
float Canvas::lookAtX = 0;
float Canvas::lookAtY = 0;
float Canvas::lookAtZ = 0;

Canvas::Canvas(int screenWidth, int screenHeight, char* windowTitle, RenderingMode mode_) {
    title = windowTitle;
    window = glfwCreateWindow(screenWidth, screenHeight, windowTitle, nullptr, nullptr);
    mode = mode_;
    glfwSetKeyCallback(window, key_callback);
    
        
    glfwGetFramebufferSize(window, &width, &height);
    
    camY = -1;
    camX = -1;
    camZ = -1;
}

void Canvas::initCanvas() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glOrtho(0.0f, width, 0.0f, height, 0.0f, 1.0f);
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity();
    
    glfwPollEvents();
}

Eigen::Matrix4f Canvas::perspectiveMatrix(float fovy, float aspect, float zNear, float zFar) {
    Eigen::Matrix4f m;
    float f = 1.0 / (tan(fovy * (M_PI / 180) / 2.0));
    m << f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (zFar + zNear) / (zNear - zFar), -1,
    0, 0, (zFar*zNear) / (zNear - zFar), 0;
    
    return m;
}

Eigen::Matrix4f Canvas::lookatMatrix(Eigen::Vector3f eye, Eigen::Vector3f center, Eigen::Vector3f up) {
    // transformation to the camera coordinate
    Eigen::Matrix4f m;
    
    Eigen::Vector3f f = (center - eye).normalized();
    Eigen::Vector3f upp = up.normalized();
    Eigen::Vector3f temp = f.cross(upp);
    Eigen::Vector3f s = temp.normalized();
    Eigen::Vector3f u = s.cross(f);

    m << s[0], s[1], s[2], 0,
    u[0], u[1], u[2], 0,
    -f[0], -f[1], -f[2], 0,
    0, 0, 0, 1;
    
    Eigen::Matrix4f t;
    t << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    -eye[0], -eye[1], -eye[2], 1;


    m = m.transpose() * t;
    return m;
}

Eigen::Vector2f Canvas::scaleNDCToViewport(Eigen::Vector2f xy_NDC) {
    float percentX = (xy_NDC[0] + 1) / 2;
    float percentY = (xy_NDC[1] + 1) / 2;

    return Eigen::Vector2f(percentX*width, percentY*height);
}

Eigen::Vector2f Canvas::rasterizePoint(SpringEndpoint* particle) {
    Eigen::Matrix4f pm = perspectiveMatrix(45, width/height, 0.00001, 100);
    Eigen::Matrix4f lm = lookatMatrix(Eigen::Vector3f(camX, camY, camZ), Eigen::Vector3f(lookAtX, lookAtY, lookAtZ), Eigen::Vector3f(0, 1, 0));
    Eigen::Matrix4f plm = lm * pm;
    
    Eigen::Vector4f p = Eigen::Vector4f(particle->getX(), particle->getY(), particle->getZ(), 1);
    Eigen::Vector4f projectedP = plm * p;
    Eigen::Vector4f NDC_V1 = projectedP /projectedP[3];
    Eigen::Vector2f pixel = scaleNDCToViewport(Eigen::Vector2f(NDC_V1[0], NDC_V1[1]));

    return pixel;
}

void Canvas::drawParticles(vector<SpringEndpoint*> particles) {
    glColor3f(0.5f, 0.5f, 0.5f);
    int particleSize = 2;
    for (int i = 0; i < particles.size(); i++) {
        if (mode==THREE_D) {
            Eigen::Vector2f pixel = rasterizePoint(particles[i]);
            glBegin(GL_QUADS);
            glVertex2f(pixel[0]-particleSize, pixel[1]-particleSize);
            glVertex2f(pixel[0]+particleSize, pixel[1]-particleSize);
            glVertex2f(pixel[0]+particleSize, pixel[1]+particleSize);
            glVertex2f(pixel[0]-particleSize, pixel[1]+particleSize);
            glEnd();

            vector<SpringEndpoint*> particleNeighbours = particles[i]->getNeighbourEndpoints();
            for (int j = 0; j < particleNeighbours.size(); j++) {
                Eigen::Vector2f neighbourPixel = rasterizePoint(particleNeighbours[j]);

                glBegin(GL_LINES);
                glVertex2f(pixel[0], pixel[1]);
                glVertex2f(neighbourPixel[0], neighbourPixel[1]);
                glEnd();
            }
        } else if (mode==TWO_D) {
            //ORIGINAL
            glBegin(GL_QUADS);
            glVertex2f(particles[i]->getX()-particleSize, particles[i]->getY()-particleSize);
            glVertex2f(particles[i]->getX()+particleSize, particles[i]->getY()-particleSize);
            glVertex2f(particles[i]->getX()+particleSize, particles[i]->getY()+particleSize);
            glVertex2f(particles[i]->getX()-particleSize, particles[i]->getY()+particleSize);
            glEnd();
            vector<SpringEndpoint*> particleNeighbours = particles[i]->getNeighbourEndpoints();
            for (int j = 0; j < particleNeighbours.size(); j++) {
                glBegin(GL_LINES);
                glVertex2f(particles[i]->getX(), particles[i]->getY());
                glVertex2f(particleNeighbours[j]->getX(), particleNeighbours[j]->getY());
                glEnd();
            }
        }
    }
}

void Canvas::drawSpring(Spring s) {
    //drawParticles(s.getParticles());
}

GLFWwindow* Canvas::getWindow() {
    return window;
}

int Canvas::getHeight() {
    return height;
}

int Canvas::getWidth() {
    return width;
}

//
//- lagrange multipliers
//- h refinement and p refinement
//- hessian construction, do it spring by spring and create a 6x6 hessian and put the corresponding 4 sections of the hessian into the right sections of the global hessian

