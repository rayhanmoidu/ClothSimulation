//
//  SpringEndpoint.hpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#ifndef SpringEndpoint_hpp
#define SpringEndpoint_hpp

#include <stdio.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <vector>

using namespace std;

enum ForceType {
    SPRING = 0,
    GRAVITY,
};

class SpringEndpoint {
public:
    SpringEndpoint(int, Eigen::Vector3f, float);
    SpringEndpoint();
    
    // adjusting positions
    void assignNewPosition(Eigen::Vector3f);
    void stepForward(float);
    
    // forces
    void resetForces();
    void addExternalForce(std::pair<Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float), ForceType>);
    void computeResultingForce(float);
    
    // evaluation helpers
    Eigen::Vector3f evaluateGradient(Eigen::Vector3f, float, float);
    Eigen::Matrix3f evaluateHessian(Eigen::Vector3f, float, float);
    Eigen::Vector3f evaluateGradient_Test(Eigen::Vector3f, float, float);
    vector<std::pair<int, Eigen::Matrix3f>> evaluateHessian_Test(Eigen::Vector3f, float, float);
    Eigen::Matrix3f evaluateHessian_Spring(Eigen::Vector3f);
    Eigen::Matrix3f evaluateHessian_Gravity(Eigen::Vector3f);
    
    // getters
    float getX();
    float getY();
    float getZ();
    Eigen::Vector3f getPosition();
    Eigen::Vector3f getPreviousPosition();
    Eigen::Matrix3f getMass();
    Eigen::Vector3f getResultingForce();
    vector<SpringEndpoint*> getNeighbourEndpoints();
    int getID();
    
    // helpers
    void addNeighbourEndpoint(SpringEndpoint*);
private:
    Eigen::Vector3f computeNextVelocity(float);
    
    int id;
    
    Eigen::Vector3f position;
    Eigen::Vector3f oldPosition;
    
    Eigen::Matrix3f mass;
    Eigen::Vector3f velocity;
    
    vector<std::pair<Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float), ForceType>> forceAccumulator;
    Eigen::Vector3f resultingForce;
    
    vector<SpringEndpoint*> neighbourEndpoints;
};

#endif /* SpringEndpoint_hpp */
