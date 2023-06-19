#include "Canvas.hpp"
#include <iostream>
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

Canvas::Canvas() {
}

Canvas::Canvas(int screenWidth, int screenHeight, char* windowTitle) {
    title = windowTitle;
    window = glfwCreateWindow(screenWidth, screenHeight, windowTitle, nullptr, nullptr);
    
        
    glfwGetFramebufferSize(window, &width, &height);
    
    camY = 500;
    camX = 500;
    camZ = -100;
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
    //cout<<"f "<<f<<endl;
    Eigen::Vector3f upp = up.normalized();
    //cout<<"upp "<<upp<<endl;
    Eigen::Vector3f temp = f.cross(upp);
    //cout<<"temp "<<temp<<endl;
    Eigen::Vector3f s = temp.normalized();
    //cout<<"s "<<s<<endl;
    Eigen::Vector3f u = s.cross(f);
    //cout<<"u "<<u<<endl;
    
//    Eigen::Vector3f temp = Eigen::Vector3f(0, 0, 0);
//    Eigen::Vector3f s = temp.normalized();
//    Eigen::Vector3f u = Eigen::Vector3f(0, 0, 0);

    m << s[0], s[1], s[2], 0,
    u[0], u[1], u[2], 0,
    -f[0], -f[1], -f[2], 0,
    0, 0, 0, 1;
    
    //m = m.transpose();

    Eigen::Matrix4f t;
    t << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    -eye[0], -eye[1], -eye[2], 1;


    m = m.transpose() * t;
    //cout << m << endl;
    return m;
}

Eigen::Vector2f Canvas::scaleNDCToViewport(Eigen::Vector2f xy_NDC) {
    float percentX = (xy_NDC[0] + 1) / 2;
    float percentY = (xy_NDC[1] + 1) / 2;

    return Eigen::Vector2f(percentX*width, percentY*height);
}

Eigen::Vector2f Canvas::rasterizePoint(SpringEndpoint* particle) {
    Eigen::Matrix4f pm = perspectiveMatrix(40, width/height, 0.00001, 100);
    Eigen::Matrix4f lm = lookatMatrix(Eigen::Vector3f(camX, camY, camZ), Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 1, 0));
    //pm[0] = 1.81066;
    //cout << pm << endl;
    //cout << "hmm1 "<<pm<<endl;
    //cout << "hmm2 "<<lm<<endl;
    Eigen::Matrix4f plm = pm * lm;
//    plm << 1.81066, 0, 0, 0,
//    0, 2.41421, 0, 0,
//    0, 0, -1, -1,
//    0, 0, 1.4999, 1.5;

    //cout << "hmm "<<plm<<endl;
    
    Eigen::Vector4f p = Eigen::Vector4f(particle->getX(), particle->getY(), particle->getZ(), 1);
    //cout << p << endl;
    Eigen::Vector4f projectedP = plm * p;
    //cout << "0 "<<projectedP[0]<<" "<<projectedP[1]<<" "<<projectedP[2]<<" "<<projectedP[3]<<endl;
    Eigen::Vector4f NDC_V1 = projectedP /projectedP[3];
    //cout << "1 "<<NDC_V1[0]<<" "<<NDC_V1[1]<<endl;
    Eigen::Vector2f pixel = scaleNDCToViewport(Eigen::Vector2f(NDC_V1[0], NDC_V1[1]));
    cout << "2 "<<pixel[0]<<" "<<pixel[1]<<endl;
    //cout << "3 "<<particle->getX()<<" "<<particle->getY()<<" "<<particle->getZ()<<" "<<endl;
    bool isViewable = pixel[0] <= width && pixel[0] >= 0 && pixel[1] <= height && pixel[1] >= 0;

    if (isViewable) {
        return pixel;
    }
    return Eigen::Vector2f(-1, -1);
}

void Canvas::drawParticles(vector<SpringEndpoint*> particles) {
    glColor3f(0.5f, 0.5f, 0.5f);
    int particleSize = 10;
    for (int i = 0; i < particles.size(); i++) {
//        Eigen::Vector2f pixel = rasterizePoint(particles[i]);
//        if (pixel[0]!=-1 && pixel[1]!=-1) {
//            cout <<"hi"<<endl;
//            glBegin(GL_QUADS);
//            glVertex2f(pixel[0]-particleSize, pixel[1]-particleSize);
//            glVertex2f(pixel[0]+particleSize, pixel[1]-particleSize);
//            glVertex2f(pixel[0]+particleSize, pixel[1]+particleSize);
//            glVertex2f(pixel[0]-particleSize, pixel[1]+particleSize);
//            glEnd();
//        }
        
        //***************************************************
        
//        int f = particles[i]->getZ() - camZ;
//        const int newX = std::round(((particles[i]->getX()) * (f / particles[i]->getZ())));
//        const int newY = std::round(((particles[i]->getY()) * (f / particles[i]->getZ())));
//
//        vector<SpringEndpoint*> particleNeighbours = particles[i]->getNeighbourEndpoints();
//
//        for (int j = 0; j < particleNeighbours.size(); j++) {
//            int f = particleNeighbours[j]->getZ() - camZ;
//            const int neighborNewX = std::round(((particleNeighbours[j]->getX()) * (f / particleNeighbours[j]->getZ())));
//            const int neighborNewY = std::round(((particleNeighbours[j]->getY()) * (f / particleNeighbours[j]->getZ())));
//
//            glBegin(GL_LINES);
//            glVertex2f(newX, newY);
//            glVertex2f(neighborNewX, neighborNewY);
//            glEnd();
//        }
//        if (newX >= 0 && newX <= width && newY >=0 && newY <= height) {
//            glBegin(GL_QUADS);
//            glVertex2f(newX-particleSize, newY-particleSize);
//            glVertex2f(newX+particleSize, newY-particleSize);
//            glVertex2f(newX+particleSize, newY+particleSize);
//            glVertex2f(newX-particleSize, newY+particleSize);
//            glEnd();
//        }
        
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
