//
//  SpringEndpoint.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#include "SpringEndpoint.hpp"
#include <iostream>

// ******************************** CONSTRUCTORS ****************************************

SpringEndpoint::SpringEndpoint() {
    position = Eigen::Vector3f(0, 0, 0);
    oldPosition = Eigen::Vector3f(0, 0, 0);
    velocity = Eigen::Vector3f(0, 0, 0);
    mass = Eigen::Matrix3f::Identity();
    id = -1;
    resultingForce = Eigen::Vector3f(0, 0, 0);
}

SpringEndpoint::SpringEndpoint(int id_, Eigen::Vector3f curPos, float m) {
    position = curPos;
    oldPosition = curPos;
    mass = Eigen::Matrix3f::Identity() * m;
    velocity = Eigen::Vector3f(0, 0, 0);
    id = id_;
    resultingForce = Eigen::Vector3f(0, 0, 0);
}

// ******************************** GETTERS ****************************************

int SpringEndpoint::getID() {
    return id;
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

Eigen::Vector3f SpringEndpoint::getPosition() {
    return position;
}

Eigen::Vector3f SpringEndpoint::getPreviousPosition() {
    return oldPosition;
}

Eigen::Matrix3f SpringEndpoint::getMass() {
    return mass;
}

Eigen::Vector3f SpringEndpoint::getResultingForce() {
    return resultingForce;
}

vector<SpringEndpoint*> SpringEndpoint::getNeighbourEndpoints() {
    return neighbourEndpoints;
}

// ******************************** HELPERS ****************************************

void SpringEndpoint::addNeighbourEndpoint(SpringEndpoint* neighbour) {
    neighbourEndpoints.push_back(neighbour);
}

void SpringEndpoint::assignNewPosition(Eigen::Vector3f newPosition) {
    if (id!=3) {
        oldPosition = position;
        position = newPosition;
    }
}

void SpringEndpoint::resetForces() {
    resultingForce = Eigen::Vector3f(0, 0, 0);
}

void SpringEndpoint::addForce(Eigen::Vector3f newForce){
    resultingForce += newForce;
}

Eigen::Vector3f SpringEndpoint::computeNextVelocity(float timeStep) {
    return velocity + timeStep * (mass.inverse() * resultingForce);
}

void SpringEndpoint::stepForward(float timeStep) {
    velocity = computeNextVelocity(timeStep);
    Eigen::Vector3f newpos = position + timeStep * velocity;
    oldPosition = position;
    position = newpos;
}
//
//Eigen::Vector3f SpringEndpoint::evaluateGradient(Eigen::Vector3f newPosition, float timeStep, float time) {
//    SpringEndpoint nextPos = *this;
//    nextPos.assignNewPosition(newPosition);
//    nextPos.computeResultingForce(time);
//    Eigen::Vector3f y = 2*position - oldPosition;
//    Eigen::Vector3f clause2 = mass * (newPosition - y);
//    Eigen::Vector3f clause1 = (timeStep * timeStep) * nextPos.getResultingForce();
//    return clause2 - clause1;
//}
//
//Eigen::Vector3f SpringEndpoint::evaluateGradient_Test(Eigen::Vector3f newPosition, float timeStep, float time) {
//    SpringEndpoint nextPos = *this;
//    nextPos.assignNewPosition(newPosition);
//    nextPos.computeResultingForce(time);
//    return nextPos.getResultingForce();
//}
//
//vector<std::pair<int, Eigen::Matrix3f>> SpringEndpoint::evaluateHessian_Test(Eigen::Vector3f newPosition, float timeStep, float time) {
//    vector<std::pair<int, Eigen::Matrix3f>> output;
//
//    for (int i = 0; i < neighbourEndpoints.size(); i++) {
//        int neighbourID = neighbourEndpoints[i]->getID();
//        Eigen::Matrix3f curHessianPortion;
//        curHessianPortion << 0, 0, 0,
//        0, 0, 0,
//        0, 0, 0;
//
//        for (int i = 0; i < forceAccumulator.size(); i++) {
//            switch(forceAccumulator[i].second) {
//                case SPRING:
//                    curHessianPortion += evaluateHessian_Spring(newPosition);
//                    break;
//                case GRAVITY:
//                    curHessianPortion += evaluateHessian_Gravity(newPosition);
//                    break;
//                default:
//                    break;
//            }
//        }
//
//        std::pair<int, Eigen::Matrix3f> newPair = std::pair<int, Eigen::Matrix3f>(neighbourID, curHessianPortion);
//        output.push_back(newPair);
//    }
//    return output;
//}
//
//Eigen::Matrix3f SpringEndpoint::evaluateHessian(Eigen::Vector3f newPosition, float timeStep, float time) {
//    Eigen::Matrix3f finalMatrix;
//    finalMatrix << 0, 0, 0,
//    0, 0, 0,
//    0, 0, 0;
//
//    for (int i = 0; i < forceAccumulator.size(); i++) {
//        switch(forceAccumulator[i].second) {
//            case SPRING:
//                finalMatrix += evaluateHessian_Spring(newPosition);
//                break;
//            case GRAVITY:
//                finalMatrix += evaluateHessian_Gravity(newPosition);
//                break;
//            default:
//                break;
//        }
//    }
//
//    Eigen::Matrix3f clause2 = timeStep*timeStep*finalMatrix;
//    return mass - clause2;
//}
//
//Eigen::Matrix3f SpringEndpoint::evaluateHessian_Spring(Eigen::Vector3f newPosition) {
//    Eigen::Matrix3f m;
//    float koverr = -14000/200;
//    m << koverr, 0, 0,
//    0, koverr, 0,
//    0, 0, koverr;
//    return m;
//}
//
//Eigen::Matrix3f SpringEndpoint::evaluateHessian_Gravity(Eigen::Vector3f newPosition) {
//    Eigen::Matrix3f m;
//    m << 0, 0, 0,
//    0, 0, 0,
//    0, 0, 0;
//    return m;
//}
