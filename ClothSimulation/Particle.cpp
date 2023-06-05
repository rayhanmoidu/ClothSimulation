//
//  Particle.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#include "Particle.hpp"
#include <iostream>

Particle::Particle() {
    position = Eigen::Vector3f(0, 0, 0);
    oldPosition = Eigen::Vector3f(0, 0, 0);
    velocity = Eigen::Vector3f(0, 0, 0);
    mass = Eigen::Matrix3f::Identity();
}

Particle::Particle(Eigen::Vector3f curPos, Eigen::Vector3f v0, float m) {
    position = curPos;
    oldPosition = curPos;
    mass = Eigen::Matrix3f::Identity() * m;
    velocity = v0;
}

Eigen::Vector3f Particle::getVelocity() {
    return velocity;
}

float Particle::getX() {
    return position[0];
}

float Particle::getY() {
    return position[1];
}

float Particle::getZ() {
    return position[2];
}

Eigen::Vector3f Particle::getPosition() {
    return position;
}

Eigen::Vector3f Particle::computeResultingForce() {
    Eigen::Vector3f result(0, 0, 0);
    for (int i = 0; i < forceAccumulator.size(); i++) {
        result += forceAccumulator[i];
    }
    return result;
}

void Particle::resetForces(vector<Eigen::Vector3f> newForces) {
    forceAccumulator = newForces;
    resultingForce = computeResultingForce();
}

void Particle::applyExternalForce(Eigen::Vector3f newForce) {
    forceAccumulator.push_back(newForce);
    resultingForce = computeResultingForce();
}

//void Particle::computeNextVelocity(float timeStep) {
//    cout << this->forceAccumulator.size()<<endl;
//    for (int i = 0; i < this->forceAccumulator.size(); i++) {
//        cout << this->forceAccumulator[i]<<endl;
//    }
    //cout<<resultingForce<<endl;
    //velocity = velocity + timeStep * (resultingForce * mass.inverse());
//}

Eigen::Matrix3f Particle::getMass() {
    return mass;
}

void Particle::stepForward(float timeStep) {
//    Eigen::Vector3f nextForce = resultingForce + (resultingForce.)
    //Eigen::Vector3f newpos = 2*position - oldPosition + (timeStep*timeStep)*(mass.inverse())*(resultingForce);
    Eigen::Vector3f y = 2*position - oldPosition;
    Eigen::Vector3f rhs = mass * y;
    float xCoefficient = mass.diagonal()[0] + timeStep*timeStep*14000/200;
    float yCoefficient = mass.diagonal()[0];
    float zCoefficient = mass.diagonal()[0];
    Eigen::Vector3f newpos = Eigen::Vector3f((rhs[0] + timeStep*timeStep*14000)/xCoefficient, rhs[1]/yCoefficient, rhs[2]/zCoefficient);
    cout << newpos << endl;
    oldPosition = position;
    position = newpos;
}

//void Particle::optimizationImplicitEuler() {
//    bool converged = false;
//    Eigen::Vector3f curGuessPoint = Eigen::Vector3f(0, 0, 0);
//    while (!converged) {
//        curGuessPoint =
//    }
//}
//
//void Particle::evaluateGradient(float timeStep, Eigen::Vector3f point) {
//    Eigen::Vector3f nextForces = computeForceOnParticle(
//}
