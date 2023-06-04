//
//  Particle.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#include "Particle.hpp"
#include <iostream>

Particle::Particle() {
    position = Eigen::Array3f(0, 0, 0);
    velocity = Eigen::Array3f(0, 0, 0);
    mass = 0;
}

Particle::Particle(Eigen::Array3f curPos, Eigen::Array3f v0, float m) {
    position = curPos;
    mass = m;
    velocity = v0;
}

Eigen::Array3f Particle::getVelocity() {
    return velocity;
}
//
//void Particle::changePosition(Eigen::Array3f newPos) {
//    position = newPos;
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

Eigen::Array3f Particle::computeResultingForce() {
    Eigen::Array3f result(0, 0, 0);
    for (int i = 0; i < forceAccumulator.size(); i++) {
        result[0] += forceAccumulator[i][0];
        result[1] += forceAccumulator[i][1];
        result[2] += forceAccumulator[i][2];
    }
    cout<<"boi"<<endl;
    cout << result[0]<<endl;
    cout<<"lala"<<endl;
    cout << result[1]<<endl;
    return result;
}

void Particle::resetForces(vector<Eigen::Array3f> newForces) {
    forceAccumulator = newForces;
    resultingForce = computeResultingForce();
}

void Particle::addForce(Eigen::Array3f newForce) {
    forceAccumulator.push_back(newForce);
    resultingForce = computeResultingForce();
}

void Particle::computeNextVelocity(float timeStep) {
    velocity = velocity + timeStep * (resultingForce / mass);
}

float Particle::getMass() {
    return mass;
}

void Particle::stepForward(float timeStep) {
    computeNextVelocity(timeStep);
    Eigen::Array3f newpos = position + timeStep * velocity;
    cout <<"new y pos"<<newpos[1]<<endl;
    position = newpos;
}
