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
#include <vector>

using namespace std;

class Particle {
public:
    Particle(Eigen::Array3f, Eigen::Array3f, float);
    Particle();
    
    void stepForward(float);
    void resetForces(vector<Eigen::Array3f>);
    void addForce(Eigen::Array3f);
    
    float getX();
    float getY();
    float getZ();
    float getMass();
    Eigen::Array3f getVelocity();
private:
    void computeNextVelocity(float);
    Eigen::Array3f computeResultingForce();
    Eigen::Array3f position;
    float mass;
    Eigen::Array3f velocity;
    vector<Eigen::Array3f> forceAccumulator;
    Eigen::Array3f resultingForce;
};

#endif /* Particle_hpp */
