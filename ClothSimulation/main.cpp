//
//  main.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-22.
//

#include <iostream>
#include "Canvas.hpp"
#include "Simulation.hpp"

const GLint WIDTH = 500, HEIGHT = 500;
const Eigen::Vector3f gravitationalConstant = Eigen::Vector3f(0, -9.81, 0);

// SYSTEM 1

//Eigen::Vector3f system1(Particle p) {
//    // F(x, y, z) = y^2z^3, 2xyz^3, 3xy^2z^2
//    // grad(F) = xy^2z^3 <---- E
//    // f = -grad(E) = -y^2z^3, -2xyz^3, -3xy^2z^2
//
//    float newX = -1 * pow(p.getY(), 2) * pow(p.getZ(), 3);
//    float newY = -2 * pow(p.getX(), 1) * pow(p.getY(), 1) * pow(p.getZ(), 3);
//    float newZ = -3 * pow(p.getX(), 1) * pow(p.getY(), 2) * pow(p.getZ(), 2);
//
//    Eigen::Vector3f lala = Eigen::Vector3f(newX, newY, newZ);
//    cout << lala << endl;
//
//    return Eigen::Vector3f(newX, newY, newZ);
//}

//Eigen::Vector3f gravitationalSystem(Particle p) {
//    // F(x, y, z) = ?????
//    // grad(F) = 10y^2 <---- E
//    // f = -grad(E) = 0, -20y, 0
//
//    float newX = 0
//    float newY = -20 * p.getY();
//    float newZ = 0;
//
//    return Eigen::Vector3f(newX, newY, newZ);
//}
//
//vector<Eigen::Vector3f> forceField(Particle p, float t) {
//    vector<Eigen::Vector3f> resultingForces;
//
//    resultingForces.push_back(system1(p));
//    return resultingForces;
//}

//Eigen::Vector3f addGravitationalForceToParticle(Particle p) {
//    return p.getMass() * gravitationalConstant;
//}

//Eigen::Vector3f getSpiralForce(Particle p, float timestep) {
//    float Fx = (p.getY()-500) - (p.getX()-500);
//    float Fy = (-p.getX()-500) - (p.getY()-500);
//    float Fz = 0;
//    Eigen::Vector3f lala = Eigen::Vector3f(Fx, Fy, Fz);
//    return lala.normalized();
//}

Eigen::Vector3f getSpringForce(Particle p, float timestep) {
    float k = 14000;
    float r = 200;
    float displacement = p.getX();
    cout << displacement << endl;
    Eigen::Vector3f force = Eigen::Vector3f(k * (abs(displacement)/r - 1) * (-displacement/abs(displacement)), 0, 0);
    //Eigen::Vector3f force = Eigen::Vector3f(-1 * k * displacement, 0, 0);
    cout << force << endl;
    return force;
}

int main(int argc, const char * argv[]) {
    glfwInit();
    char* windowTitle = "Isosurface Stuffing";
    Canvas canvas(WIDTH, HEIGHT, windowTitle);
    GLFWwindow *window = canvas.getWindow();
    
    
    if (window == nullptr) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        
        return -1;
    }

    glfwMakeContextCurrent(window);
    glewExperimental = GL_TRUE;
    
    if (GLEW_OK != glewInit()) {
        std::cout << "Failed to create GLEW" << std::endl;
        
        return -1;
    }
    
    // SPRING
    
//    Eigen::Vector3f p0(600, 700, 0);
//    float p0_mass = 50;
//    Eigen::Vector3f p0_v0(0, 0, 0);
//
//    Eigen::Vector3f p1(500, 800, 0);
//    float p1_mass = 5;
//    Eigen::Vector3f p1_v0(0, 0, 0);
//
//    Particle spring_p1(p0, p0_v0, p0_mass);
//    Particle spring_p2(p1, p1_v0, p1_mass);

//    float time = 0;
//    float timeStep = 0.05;

//    Spring s(spring_p1, spring_p2, 14000, 200, 400);
    

//    while (!glfwWindowShouldClose(window)) {
//
//
//        canvas.initCanvas();
//        canvas.drawSpring(s);
//
//        s.applyInternalSpringForces();
//        //s.applyAdditionalForce(&addGravitationalForceToParticle);
//
//        s.stepForward(timeStep);
//        time += timeStep;
//
//        glfwSwapBuffers(window);
//    }
    
    // PARTICLE
    std::vector<Particle> particles1;
    Particle startPos1(Eigen::Vector3f(100, 500, 0), Eigen::Vector3f(99, 500, 0), 5);
    particles1.push_back(startPos1);
    
    std::vector<Particle> particles2;
    Particle startPos2(Eigen::Vector3f(100, 900, 0), Eigen::Vector3f(99, 900, 0), 5);
    particles2.push_back(startPos2);
    
    float timeStep = 0.005;
    
    Simulation simulation1(timeStep, particles1, canvas, OPTIMIZATION_IMPLICIT_EULER);
    Simulation simulation2(timeStep, particles2, canvas, TIMESTEP);
    
    simulation1.addExternalForce(&getSpringForce);
    simulation2.addExternalForce(&getSpringForce);

    while (!glfwWindowShouldClose(window)) {
        canvas.initCanvas();
        simulation1.update();
        //simulation2.update();
        glfwSwapBuffers(window);
    }
    
    glfwTerminate();
    return 0;
}
