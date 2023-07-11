//
//  main.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-22.
//

#include <iostream>
#include "Canvas.hpp"
#include "Simulation.hpp"
#include "Examples.hpp"

const GLint WIDTH = 500, HEIGHT = 500;
const Eigen::Vector3f gravitationalConstant = Eigen::Vector3f(0, -9.81, 0);
const Eigen::Vector3f windConstant = Eigen::Vector3f(0, 0, 0);


Eigen::Vector3f getSpringForce(SpringEndpoint p1, SpringEndpoint p2, float timestep) {
    // before
    float k = 14000;
    float r = 200;
    
    float displacementX = p1.getX() - p2.getX();
    float displacementY = p1.getY() - p2.getY();
    float displacementZ = p1.getZ() - p2.getZ();

    float displacementXY = sqrt(displacementX*displacementX + displacementY*displacementY);
//    float displacement = sqrt(displacementXY*displacementXY + displacementZ*displacementZ);
    
    float displacement = sqrt(displacementX*displacementX + displacementY*displacementY + displacementZ*displacementZ);

    float force = k * (abs(displacement)/r - 1);

    float theta = atan(abs(displacementY) / abs(displacementX));
    float phi = atan(abs(displacementZ) / abs(displacementXY));

    float forceX = force * cos(theta);
    float forceY = force * sin(theta);
    float forceZ = force * sin(phi);

    float dirX = (-displacementX/abs(displacementX));
    float dirY = (-displacementY/abs(displacementY));
    float dirZ = (-displacementZ/abs(displacementZ));

    float fx = displacementX == 0 ? 0 : forceX * dirX;
    float fy = displacementY == 0 ? 0 : forceY * dirY;
    float fz = displacementZ == 0 ? 0 : forceZ * dirZ;

    return Eigen::Vector3f(fx, fy, fz);
}

Eigen::Vector3f getGravitationalForce(SpringEndpoint p1, SpringEndpoint p2, float timestep) {
    return p1.getMass()*gravitationalConstant;
}

Eigen::Vector3f getWindForce(SpringEndpoint p1, SpringEndpoint p2, float timestep) {
    return p1.getMass()*windConstant;
}

int main(int argc, const char * argv[]) {
    glfwInit();
    char* windowTitle = "Cloth Simulation";
    Canvas canvas(WIDTH, HEIGHT, windowTitle, TWO_D);
    GLFWwindow *window = canvas.getWindow();
    Examples exampleFactory;
    
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
 
    // *********************************** SIMULARTION SETUP ******************************************
    
    ClothRepresentation pyramidExample = exampleFactory.getExample(BASIC_CLOTH);
    

    float timeStep = 0.05;
    
    Simulation simulation1(pyramidExample, timeStep, canvas, OPTIMIZATION_IMPLICIT_EULER);
    
    simulation1.addExternalForce(getGravitationalForce, GRAVITY);
    simulation1.addExternalForce(getWindForce, GRAVITY);
    
    while (!glfwWindowShouldClose(window)) {
        canvas.initCanvas();
        simulation1.update();
        glfwSwapBuffers(window);
    }
    
    glfwTerminate();
    return 0;
}
