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

class SpringEndpoint {
public:
    SpringEndpoint(Eigen::Vector3f, Eigen::Vector3f, float);
    SpringEndpoint(Eigen::Vector3f, Eigen::Vector3f, float, vector<Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float)>, float, vector<SpringEndpoint*> neighbours);
    SpringEndpoint();
    
    // adjusting positions
    void assignNewPosition(Eigen::Vector3f);
    void stepForward(float);
    
    // forces
    void resetForces();
    void addExternalForce(Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float));
    void computeResultingForce(float);
    
    // evaluation helpers
    Eigen::Vector3f evaluateGradient(Eigen::Vector3f, float, float);
    Eigen::Matrix3f evaluateHessian(Eigen::Vector3f, float, float);
    
    // getters
    float getX();
    float getY();
    float getZ();
    Eigen::Vector3f getPosition();
    Eigen::Matrix3f getMass();
    Eigen::Vector3f getResultingForce();
    vector<SpringEndpoint*> getNeighbourEndpoints();
    
    // helpers
    void addNeighbourEndpoint(SpringEndpoint*);
private:
    Eigen::Vector3f computeNextVelocity(float);
    
    Eigen::Vector3f position;
    Eigen::Vector3f oldPosition;
    
    Eigen::Matrix3f mass;
    Eigen::Vector3f velocity;
    
    vector<Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float)> forceAccumulator;
    Eigen::Vector3f resultingForce;
    
    vector<SpringEndpoint*> neighbourEndpoints;
};

#endif /* SpringEndpoint_hpp */
