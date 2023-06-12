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
const Eigen::Vector3f gravitationalConstant = Eigen::Vector3f(0, -9000.81, 0);

Eigen::Vector3f getSpringForce(SpringEndpoint p1, SpringEndpoint p2, float timestep) {
    // before
    float k = 14000;
    float r = 200;
    
    float displacementX = p1.getX() - p2.getX();
    float displacementY = p1.getY() - p2.getY();
    float displacement = sqrt(displacementX*displacementX + displacementY*displacementY);
    
    float force = k * (abs(displacement)/r - 1);
    
    float dirX = (-displacementX/abs(displacementX));
    float dirY = (-displacementY/abs(displacementY));
    
    return Eigen::Vector3f(force * dirX, force * dirY, 0);
    //after
//    float k = 14000;
//    float r = 200;
//
//    float theta = atan(displacementX / displacementY);
//    float displacement = sqrt(displacementX*displacementX + displacementY*displacementY);
//    float s = (abs(displacement) - r);
//    cout << theta << endl;
////    float fx = k * (abs(displacementX)/r - 1) * (-displacementX/abs(displacementX));
////    float fy = k * (abs(displacementY)/r - 1) * (-displacementY/abs(displacementY));
////    cout << fy <<" "<<displacementY <<endl;
////    float fz = k * (abs(displacementZ)/r - 1) * (-displacementZ/abs(displacementZ));
//    return Eigen::Vector3f(k*s*(-displacementX/abs(displacementX)), k*s*(-displacementY/abs(displacementY)), 0);
}

Eigen::Vector3f getGravitationalForce(SpringEndpoint p1, SpringEndpoint p2, float timestep) {
    return p1.getMass()*gravitationalConstant;
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
 
    // SPRING ENDPOINTS
    std::vector<SpringEndpoint*> springEndpoints;
    SpringEndpoint* springEndpoint1 = new SpringEndpoint(Eigen::Vector3f(400, 600, 0), Eigen::Vector3f(400, 600, 0), 5);
    SpringEndpoint* springEndpoint2 = new SpringEndpoint(Eigen::Vector3f(600, 500, 0), Eigen::Vector3f(600.5, 500, 0), 5);
    SpringEndpoint* springEndpoint3 = new SpringEndpoint(Eigen::Vector3f(200, 500, 0), Eigen::Vector3f(199.5, 500, 0), 5);
    SpringEndpoint* springEndpoint4 = new SpringEndpoint(Eigen::Vector3f(500, 1000, 0), Eigen::Vector3f(500, 1000, 0), 5);
    springEndpoint3->addNeighbourEndpoint(springEndpoint1);
    //springEndpoint1->addNeighbourEndpoint(springEndpoint2);
    //springEndpoint1->addNeighbourEndpoint(springEndpoint3);
    //springEndpoint2->addNeighbourEndpoint(springEndpoint1);
    springEndpoint1->addNeighbourEndpoint(springEndpoint4);
    springEndpoint2->addNeighbourEndpoint(springEndpoint4);
    springEndpoint3->addNeighbourEndpoint(springEndpoint4);
    springEndpoints.push_back(springEndpoint1);
    springEndpoints.push_back(springEndpoint2);
    springEndpoints.push_back(springEndpoint3);
//    std::vector<SpringEndpoint> springEndpoints2;
//    SpringEndpoint startPos2(Eigen::Vector3f(100, 900, 0), Eigen::Vector3f(99, 900, 0), 5);
//    springEndpoints2.push_back(startPos2);
    
    float timeStep = 0.005;
    
    Simulation simulation1(timeStep, springEndpoints, canvas, OPTIMIZATION_IMPLICIT_EULER);
    //Simulation simulation2(timeStep, springEndpoints2, canvas, TIMESTEP);
    
    simulation1.addExternalForce(getSpringForce);
    simulation1.addExternalForce(getGravitationalForce);
//    simulation2.addExternalForce(&getSpringForce);
    
    while (!glfwWindowShouldClose(window)) {
        canvas.initCanvas();
        simulation1.update();
        //simulation2.update();
        glfwSwapBuffers(window);
    }
    
    glfwTerminate();
    return 0;
}
