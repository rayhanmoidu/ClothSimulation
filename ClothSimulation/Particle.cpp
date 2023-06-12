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

Particle::Particle(Eigen::Vector3f curPos, Eigen::Vector3f oldPos, float m) {
    position = curPos;
    oldPosition = oldPos;
    mass = Eigen::Matrix3f::Identity() * m;
    velocity = Eigen::Vector3f(0, 0, 0);
}

Particle::Particle(Eigen::Vector3f curPos, Eigen::Vector3f oldPos, float m, vector<Eigen::Vector3f (*)(Particle, float)> forces, float time) {
    position = curPos;
    oldPosition = oldPos;
    mass = Eigen::Matrix3f::Identity() * m;
    forceAccumulator = forces;
    computeResultingForce(time);
}

//Eigen::Vector3f Particle::getVelocity() {
//    return velocity;
//}

float Particle::getX() {
    return position[0];
}

float Particle::getY() {
    return position[1];
}

float Particle::getZ() {
    return position[2];
}

void Particle::assignNewPosition(Eigen::Vector3f newPosition) {
    oldPosition = position;
    position = newPosition;
}

Eigen::Vector3f Particle::getPosition() {
    return position;
}

void Particle::computeResultingForce(float time) {
    Eigen::Vector3f result(0, 0, 0);
    for (int i = 0; i < forceAccumulator.size(); i++) {
        result += forceAccumulator[i](*this, time);
    }
    resultingForce = result;
}

void Particle::resetForces() {
    forceAccumulator.clear();
}

void Particle::addExternalForce(Eigen::Vector3f (*newForce)(Particle, float)) {
    forceAccumulator.push_back(newForce);
}

Eigen::Vector3f Particle::computeNextVelocity(float timeStep) {
    return velocity + timeStep * (mass.inverse() * resultingForce);
}

Eigen::Matrix3f Particle::getMass() {
    return mass;
}

Eigen::Vector3f Particle::getResultingForce() {
    return resultingForce;
}

void Particle::stepForward(float timeStep) {
    velocity = computeNextVelocity(timeStep);
    Eigen::Vector3f newpos = position + timeStep * velocity;
    oldPosition = position;
    position = newpos;
}

Eigen::Vector3f Particle::evaluateGradient(Eigen::Vector3f newPosition, float timeStep, float time) {
    Particle nextPos = Particle(newPosition, position, mass.diagonal()[0], forceAccumulator, time);
    Eigen::Vector3f y = 2*position - oldPosition;
    Eigen::Vector3f clause2 = mass * (newPosition - y);
    Eigen::Vector3f clause1 = (timeStep * timeStep) * nextPos.getResultingForce();
    return clause2 - clause1;
}

Eigen::Matrix3f Particle::evaluateHessian(Eigen::Vector3f newPosition, float timeStep, float time) {
    Eigen::Matrix3f m;
    float x = newPosition[0];
    float y = newPosition[1];
    float koverr = -14000/200;
    m << koverr, 0, 0,
         0, 0, 0,
         0, 0, 0;
    
    Eigen::Matrix3f clause2 = timeStep*timeStep*m;
    return mass - clause2;
}
