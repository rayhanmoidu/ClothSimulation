//
//  SpringEndpoint.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#include "SpringEndpoint.hpp"
#include <iostream>

SpringEndpoint::SpringEndpoint() {
    position = Eigen::Vector3f(0, 0, 0);
    oldPosition = Eigen::Vector3f(0, 0, 0);
    velocity = Eigen::Vector3f(0, 0, 0);
    mass = Eigen::Matrix3f::Identity();
}

SpringEndpoint::SpringEndpoint(Eigen::Vector3f curPos, Eigen::Vector3f oldPos, float m) {
    position = curPos;
    oldPosition = oldPos;
    mass = Eigen::Matrix3f::Identity() * m;
    velocity = Eigen::Vector3f(0, 0, 0);
}

vector<SpringEndpoint*> SpringEndpoint::getNeighbourEndpoints() {
    return neighbourEndpoints;
}

SpringEndpoint::SpringEndpoint(Eigen::Vector3f curPos, Eigen::Vector3f oldPos, float m, vector<Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float)> forces, float time, vector<SpringEndpoint*> neighbours) {
    position = curPos;
    oldPosition = oldPos;
    mass = Eigen::Matrix3f::Identity() * m;
    forceAccumulator = forces;
    neighbourEndpoints = neighbours;

    computeResultingForce(time);
}

float SpringEndpoint::getX() {
    return position[0];
}

float SpringEndpoint::getY() {
    return position[1];
}

float SpringEndpoint::getZ() {
    return position[2];
}

void SpringEndpoint::assignNewPosition(Eigen::Vector3f newPosition) {
    oldPosition = position;
    position = newPosition;
}

Eigen::Vector3f SpringEndpoint::getPosition() {
    return position;
}

void SpringEndpoint::computeResultingForce(float time) {
    Eigen::Vector3f result(0, 0, 0);
//    for (int j = 0; j < neighbourEndpoints.size(); j++) {
        for (int i = 0; i < forceAccumulator.size(); i++) {
            for (int j = 0; j < neighbourEndpoints.size(); j++) {
                result += forceAccumulator[i](*this, *neighbourEndpoints[j], time);
            }
        }
//    }
    //cout << result << endl;
    //cout <<"result"<<endl;
    //cout << result<<endl;
    resultingForce = result;
}

void SpringEndpoint::resetForces() {
    forceAccumulator.clear();
}

void SpringEndpoint::addExternalForce(Eigen::Vector3f (*newForce)(SpringEndpoint, SpringEndpoint, float)) {
    forceAccumulator.push_back(newForce);
}

Eigen::Vector3f SpringEndpoint::computeNextVelocity(float timeStep) {
    return velocity + timeStep * (mass.inverse() * resultingForce);
}

Eigen::Matrix3f SpringEndpoint::getMass() {
    return mass;
}

Eigen::Vector3f SpringEndpoint::getResultingForce() {
    return resultingForce;
}

void SpringEndpoint::addNeighbourEndpoint(SpringEndpoint* neighbour) {
    neighbourEndpoints.push_back(neighbour);
}

void SpringEndpoint::stepForward(float timeStep) {
    velocity = computeNextVelocity(timeStep);
    Eigen::Vector3f newpos = position + timeStep * velocity;
    oldPosition = position;
    position = newpos;
}

Eigen::Vector3f SpringEndpoint::evaluateGradient(Eigen::Vector3f newPosition, float timeStep, float time) {
    SpringEndpoint nextPos = *this;
    nextPos.assignNewPosition(newPosition);
    Eigen::Vector3f y = 2*position - oldPosition;
    Eigen::Vector3f clause2 = mass * (newPosition - y);
    Eigen::Vector3f clause1 = (timeStep * timeStep) * nextPos.getResultingForce();
    return clause2 - clause1;
}

Eigen::Matrix3f SpringEndpoint::evaluateHessian(Eigen::Vector3f newPosition, float timeStep, float time) {
    Eigen::Matrix3f m;
    float x = newPosition[0];
    float y = newPosition[1];
    float koverr = -14000/200;
    m << koverr, koverr, koverr,
    koverr, koverr, koverr,
    koverr, koverr, koverr;
    
    Eigen::Matrix3f clause2 = timeStep*timeStep*m;
    return mass - clause2;
}
