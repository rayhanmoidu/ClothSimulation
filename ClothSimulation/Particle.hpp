//
//  Particle.hpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#ifndef Particle_hpp
#define Particle_hpp

#include <stdio.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <vector>

using namespace std;

class Particle {
public:
    Particle(Eigen::Vector3f, Eigen::Vector3f, float);
    Particle();
    
    void stepForward(float);
    void resetForces(vector<Eigen::Vector3f>);
    void applyExternalForce(Eigen::Vector3f);
    
    void optimizationImplicitEuler(float);
    bool evaluateGradient(float);
    
    float getX();
    float getY();
    float getZ();
    Eigen::Vector3f getPosition();
    Eigen::Matrix3f getMass();
    Eigen::Vector3f getVelocity();
private:
//    void computeNextVelocity(float);
    Eigen::Vector3f computeResultingForce();
    
    Eigen::Vector3f position;
    Eigen::Vector3f oldPosition;
    
    Eigen::Matrix3f mass;
    Eigen::Vector3f velocity;
    vector<Eigen::Vector3f> forceAccumulator;
    Eigen::Vector3f resultingForce;
};

#endif /* Particle_hpp */
