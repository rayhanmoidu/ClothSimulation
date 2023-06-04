//
//  Spring.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#include "Spring.hpp"
#include <cmath>
#include <iostream>

Spring::Spring(Particle pp1, Particle pp2, float ks, float kd, float r) {
    restLength = r;
    springConstant = ks;
    dampingConstant = kd;
    p1 = pp1;
    p2 = pp2;
}

void Spring::stepForward(float timeStep) {
    p1.stepForward(timeStep);
    p2.stepForward(timeStep);
}

vector<Particle> Spring::getParticles() {
    vector<Particle> particles;
    particles.push_back(p1);
    particles.push_back(p2);
    return particles;
}

void Spring::applyInternalSpringForces() {
    float curLengthX = p2.getX() - p1.getX();
    float curLengthY = p2.getY() - p1.getY();
    float curLengthZ = p2.getZ() - p1.getZ();
    float term1X = (abs(curLengthX) / restLength) - 1;
    float term1Y = (abs(curLengthY) / restLength) - 1;
    float term1Z = (abs(curLengthZ) / restLength) - 1;
    float term2X = ((p2.getVelocity()[0] - p1.getVelocity()[0]) * (curLengthX)) / (restLength*abs(curLengthX));
    float term2Y = ((p2.getVelocity()[1] - p1.getVelocity()[1]) * (curLengthY)) / (restLength*abs(curLengthY));
    float term2Z = ((p2.getVelocity()[2] - p1.getVelocity()[2]) * (curLengthZ)) / (restLength*abs(curLengthZ));
    float resultX = (springConstant*term1X + dampingConstant*term2X) * (curLengthX/abs(curLengthX));
    float resultY = (springConstant*term1Y + dampingConstant*term2Y) * (curLengthY/abs(curLengthY));
    float resultZ = (springConstant*term1Z + dampingConstant*term2Z) * (curLengthZ/abs(curLengthZ));
    
    if (curLengthX==0) resultX = 0;
    if (curLengthY==0) resultY = 0;
    if (curLengthZ==0) resultZ = 0;
    Eigen::Array3f result(resultX, resultY, resultZ);

    vector<Eigen::Array3f> p1Forces;
    vector<Eigen::Array3f> p2Forces;
    p1Forces.push_back(result);
    //p2Forces.push_back(-result);
    p1.resetForces(p1Forces);
    p2.resetForces(p2Forces);
}

void Spring::applyAdditionalForce(Eigen::Array3f (*getAdditionalForce)(Particle)) {
    p1.addForce(getAdditionalForce(p1));
    //p2.addForce(getAdditionalForce(p2));
}
