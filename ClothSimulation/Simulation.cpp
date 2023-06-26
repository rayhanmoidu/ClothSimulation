//
//  Simulation.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-06-04.
//

#include "Simulation.hpp"
#include <unordered_set>

Simulation::Simulation(float newTimeStep, vector<Spring> newSprings, vector<SpringEndpoint*> endpoints, Canvas newCanvas, StateComputationMode curMode) {
    timeStep = newTimeStep;
    time = 0;
    springs = newSprings;
    canvas = newCanvas;
    mode = curMode;
    
    n = endpoints.size();
    particles = endpoints;

//    getParticlesFromSprings();
    
    hessian = Eigen::MatrixXf(3*n, 3*n);
    gradient = Eigen::VectorXf(3*n);
    
    evaluateMassMatrix();
}

//void Simulation::getParticlesFromSprings() {
//
//    for (int i = 0; i < springs.size(); i++) {
//        vector<SpringEndpoint*> curEndpoints = springs[i].getEndpoints();
//        curEndpoints[0]->addNeighbourEndpoint(curEndpoints[1]);
//        curEndpoints[1]->addNeighbourEndpoint(curEndpoints[2]);
//    }
//    
//    unordered_set<int> particleIds;
//
//    for (int i = 0; i < springs.size(); i++) {
//        vector<SpringEndpoint*> curEndpoints = springs[i].getEndpoints();
//        if (particleIds.find(curEndpoints[0]->getID()) == particleIds.end()) {
//            particles.push_back(curEndpoints[0]);
//            particleIds.insert(curEndpoints[0]->getID());
//        }
//        if (particleIds.find(curEndpoints[1]->getID()) == particleIds.end()) {
//            particles.push_back(curEndpoints[1]);
//            particleIds.insert(curEndpoints[1]->getID());
//        }
////        vector<SpringEndpoint*> curEndpoints = springs[i].getEndpoints();
////        particles[curEndpoints[0]->getID()] = curEndpoints[0];
////        particles[curEndpoints[1]->getID()] = curEndpoints[1];
//    }
//    cout<<particles.size()<<endl;
//}

void Simulation::update() {
    canvas.drawParticles(particles);
    
//    applyForces();
    
    computeNewParticleStates(mode);
    
    time += timeStep;
}

void Simulation::evaluateMassMatrix() {
    Eigen::MatrixXf newMassMatrix(3*n, 3*n);
    for (int i = 0; i < 3*n; i++) {
        for (int j = 0; j < 3*n; j++) {
            newMassMatrix(i, j) = 0;
        }
    }
    for (int i = 0; i < particles.size(); i++) {
        int id = particles[i]->getID();
        Eigen::Matrix3f massPortion = particles[i]->getMass();
        
        newMassMatrix(id*3, id*3) = massPortion(0, 0);
        newMassMatrix(id*3, id*3+1) = massPortion(0, 1);
        newMassMatrix(id*3, id*3+2) = massPortion(0, 2);
        newMassMatrix(id*3+1, id*3) = massPortion(1, 0);
        newMassMatrix(id*3+1, id*3+1) = massPortion(1, 1);
        newMassMatrix(id*3+1, id*3+2) = massPortion(1, 2);
        newMassMatrix(id*3+2, id*3) = massPortion(2, 0);
        newMassMatrix(id*3+2, id*3+1) = massPortion(2, 1);
        newMassMatrix(id*3+2, id*3+2) = massPortion(2, 2);
    }
    massMatrix = newMassMatrix;
}

Eigen::VectorXf Simulation::applyNewtonsMethod_Test() {
    Eigen::VectorXf curGuessPosition(3*n);
    for (int i = 0; i < 3*n; i++) {
        curGuessPosition[i] = 0;
    }
    evaluateGradient(curGuessPosition);
    while (gradient.squaredNorm() > __FLT_EPSILON__) {
        //cout <<"hello..."<<endl;
        evaluateHessian(curGuessPosition);
        Eigen::VectorXf nextGuessPosition = curGuessPosition - hessian.inverse()*gradient;
        curGuessPosition = nextGuessPosition;
        evaluateGradient(curGuessPosition);
        cout <<"finding"<<endl;
    }
    cout <<"found!"<<endl;
    //cout << curGuessPosition << endl;
    //cout << numIts << endl;
    return curGuessPosition;
}

//Eigen::Vector3f Simulation::applyNewtonsMethod(SpringEndpoint p) {
//    Eigen::Vector3f curGuessPosition = Eigen::Vector3f(1, 1, 1);
//    Eigen::Vector3f gradient = p.evaluateGradient(curGuessPosition, timeStep, time);
//    int numIts = 1;
//    while (gradient.squaredNorm() > __FLT_EPSILON__) {
//        //cout <<"hello..."<<endl;
//        Eigen::Matrix3f hessian = p.evaluateHessian(curGuessPosition, timeStep, time);
//        Eigen::Vector3f nextGuessPosition = curGuessPosition - hessian.inverse()*gradient;
//        curGuessPosition = nextGuessPosition;
//        gradient = p.evaluateGradient(curGuessPosition, timeStep, time);
//        numIts++;
//        //cout <<"finding"<<endl;
//    }
//    //cout <<"found!"<<endl;
//    //cout << curGuessPosition << endl;
//    //cout << numIts << endl;
//    return curGuessPosition;
//}

Eigen::VectorXf Simulation::getCurPosition() {
    Eigen::VectorXf curPosition(3*n);
    for (int i = 0; i < 3*n; i++) {
        curPosition[i] = 0;
    }
    for (int i = 0; i < particles.size(); i++) {
        int rowId = particles[i]->getID();
        Eigen::Vector3f curPosPortion = particles[i]->getPosition();
        curPosition[rowId*3] = curPosPortion[0];
        curPosition[rowId*3 + 1] = curPosPortion[1];
        curPosition[rowId*3 + 2] = curPosPortion[2];
    }
    return curPosition;
}

Eigen::VectorXf Simulation::getPrevPosition() {
    Eigen::VectorXf prevPosition(3*n);
    for (int i = 0; i < 3*n; i++) {
        prevPosition[i] = 0;
    }
    for (int i = 0; i < particles.size(); i++) {
        int rowId = particles[i]->getID();
        Eigen::Vector3f prevPosPortion = particles[i]->getPreviousPosition();
        prevPosition[rowId*3] = prevPosPortion[0];
        prevPosition[rowId*3 + 1] = prevPosPortion[1];
        prevPosition[rowId*3 + 2] = prevPosPortion[2];
    }
    return prevPosition;
}

void Simulation::evaluateGradient(Eigen::VectorXf curGuessPosition) {
    Eigen::VectorXf newGradient(3*n);
    for (int i = 0; i < 3*n; i++) {
        newGradient[i] = 0;
    }
    for (int i = 0; i < particles.size(); i++) {
        
        
        
        int rowId = particles[i]->getID();
        Eigen::Vector3f curGuessPositionPortion = Eigen::Vector3f(0, 0, 0);
        curGuessPositionPortion[0] = curGuessPosition[rowId*3];
        curGuessPositionPortion[1] = curGuessPosition[rowId*3 + 1];
        curGuessPositionPortion[2] = curGuessPosition[rowId*3 + 2];
        
        vector<SpringEndpoint*> neighbours = particles[i]->getNeighbourEndpoints();
        Eigen::Vector3f accumForce(0, 0, 0);
        for(int j = 0; j < neighbours.size(); j++) {
            accumForce += calculateSpringForce(curGuessPositionPortion, neighbours[j]->getPosition(), 200, 14000);
        }
        accumForce += particles[i]->getMass()*Eigen::Vector3f(0, -900.81, 0);
        
        
        Eigen::Vector3f gradientPortion = accumForce;
        newGradient[rowId*3] = gradientPortion[0];
        newGradient[rowId*3 + 1] = gradientPortion[1];
        newGradient[rowId*3 + 2] = gradientPortion[2];
    }
    
    Eigen::VectorXf curPos = getCurPosition();
    Eigen::VectorXf prevPos = getPrevPosition();
    
    Eigen::VectorXf y = 2*curPos - prevPos;
    Eigen::VectorXf clause2 = massMatrix * (curGuessPosition - y);
    Eigen::VectorXf clause1 = (timeStep * timeStep) * newGradient;
    gradient = clause2 - clause1;
}


//void Simulation::evaluateGradient(Eigen::VectorXf curGuessPosition) {
//    Eigen::VectorXf newGradient(3*n);
//    for (int i = 0; i < 3*n; i++) {
//        newGradient[i] = 0;
//    }
//
//    // Spring Forces
//    for (int i = 0; i < springs.size(); i++) {
//        vector<SpringEndpoint*> endpoints = springs[i].getEndpoints();
//
//        int p1_id = endpoints[0]->getID();
//        int p2_id = endpoints[1]->getID();
//
//        Eigen::Vector3f p1_guessPosPortion = Eigen::Vector3f(curGuessPosition[p1_id*3], curGuessPosition[p1_id*3 + 1], curGuessPosition[p1_id*3 + 2]);
//        Eigen::Vector3f p2_guessPosPortion = Eigen::Vector3f(curGuessPosition[p2_id*3], curGuessPosition[p2_id*3 + 1], curGuessPosition[p2_id*3 + 2]);
//
//        Eigen::Vector3f forceOnP1 = calculateSpringForce(p1_guessPosPortion, p2_guessPosPortion, springs[i].getRestLength(), springs[i].getSpringConstant());
//        Eigen::Vector3f forceOnP2 = calculateSpringForce(p2_guessPosPortion, p1_guessPosPortion, springs[i].getRestLength(), springs[i].getSpringConstant());
//
//        for (int c = 0; c < 3; c++) {
//            newGradient[p1_id*3 + c] += forceOnP1[c];
//            newGradient[p2_id*3 + c] += forceOnP2[c];
//        }
//    }
//
//    // External Forces
//    for (int i = 0; i < particles.size(); i++) {
//        int p_id = particles[i]->getID();
//        Eigen::Vector3f externalForce(0, 0, 0);
//
//        for (int j = 0; j < externalForces.size(); j++) {
//            externalForce += externalForces[j].first(*particles[i]);
//        }
//
//        for (int c = 0; c < 3; c++) {
//            newGradient[p_id*3 + c] += externalForce[c];
//        }
//    }
//
//    Eigen::VectorXf curPos = getCurPosition();
//    Eigen::VectorXf prevPos = getPrevPosition();
//
//    Eigen::VectorXf y = 2*curPos - prevPos;
//    Eigen::VectorXf clause2 = massMatrix * (curGuessPosition - y);
//    Eigen::VectorXf clause1 = (timeStep * timeStep) * newGradient;
//    gradient = clause2 - clause1;
//}

void Simulation::evaluateHessian(Eigen::VectorXf curGuessPosition) {
    // hmm... might want to just do the diagnoals
    Eigen::MatrixXf newHessian(3*n, 3*n);
    for (int i = 0; i < 3*n; i++) {
        for (int j = 0; j < 3*n; j++) {
            newHessian(i, j) = 0;
        }
    }
    for (int i = 0; i < particles.size(); i++) {
        int rowId = particles[i]->getID();
        Eigen::Vector3f curGuessPositionPortion = Eigen::Vector3f(0, 0, 0);
        curGuessPositionPortion[0] = curGuessPosition[rowId*3];
        curGuessPositionPortion[1] = curGuessPosition[rowId*3 + 1];
        curGuessPositionPortion[2] = curGuessPosition[rowId*3 + 2];
        
        // working
        Eigen::Matrix3f curHessianPortion;
        float koverr = -14000/200;
        curHessianPortion << koverr, 0, 0,
        0, koverr, 0,
        0, 0, koverr;
        
        newHessian(rowId*3, rowId*3) = curHessianPortion(0, 0);
        newHessian(rowId*3, rowId*3+1) = curHessianPortion(0, 1);
        newHessian(rowId*3, rowId*3+2) = curHessianPortion(0, 2);
        newHessian(rowId*3+1, rowId*3) = curHessianPortion(1, 0);
        newHessian(rowId*3+1, rowId*3+1) = curHessianPortion(1, 1);
        newHessian(rowId*3+1, rowId*3+2) = curHessianPortion(1, 2);
        newHessian(rowId*3+2, rowId*3) = curHessianPortion(2, 0);
        newHessian(rowId*3+2, rowId*3+1) = curHessianPortion(2, 1);
        newHessian(rowId*3+2, rowId*3+2) = curHessianPortion(2, 2);
        
    }
    
    Eigen::MatrixXf clause2 = timeStep*timeStep*newHessian;
    hessian = massMatrix - clause2;
}


Eigen::MatrixXf Simulation::evaluateHessian_Portion(Eigen::Vector3f p1, Eigen::Vector3f p2) {
    Eigen::MatrixXf hessian(6, 6);
    float koverr = -14000/200;
//    hessian << koverr, 0, 0, 0, 0, 0,
//    0, koverr, 0, 0, 0, 0,
//    0, 0, koverr, 0, 0, 0,
//    0, 0, 0, koverr, 0, 0,
//    0, 0, 0, 0, koverr, 0,
//    0, 0, 0, 0, 0, koverr;
//    hessian << 0, 0, 0, koverr, 0, 0,
//    0, 0, 0, 0, koverr, 0,
//    0, 0, 0, 0, 0, koverr,
//    koverr, 0, 0, 0, 0, 0,
//    0, koverr, 0, 0, 0, 0,
//    0, 0, koverr, 0, 0, 0;
    return hessian;
}

//void Simulation::evaluateHessian(Eigen::VectorXf curGuessPosition) {
//    // hmm... might want to just do the diagnoals
//    Eigen::MatrixXf newHessian(3*n, 3*n);
//    for (int i = 0; i < 3*n; i++) {
//        for (int j = 0; j < 3*n; j++) {
//            newHessian(i, j) = 0;
//        }
//    }
//
//    for (int i = 0; i < springs.size(); i++) {
//        vector<SpringEndpoint*> endpoints = springs[i].getEndpoints();
//
//        int p1_id = endpoints[0]->getID();
//        int p2_id = endpoints[1]->getID();
//
//        Eigen::Vector3f p1_guessPosPortion = Eigen::Vector3f(curGuessPosition[p1_id*3], curGuessPosition[p1_id*3 + 1], curGuessPosition[p1_id*3 + 2]);
//        Eigen::Vector3f p2_guessPosPortion = Eigen::Vector3f(curGuessPosition[p2_id*3], curGuessPosition[p2_id*3 + 1], curGuessPosition[p2_id*3 + 2]);
//
//        Eigen::MatrixXf hessianPortion = evaluateHessian_Portion(p1_guessPosPortion, p2_guessPosPortion);
//
//        newHessian(p1_id*3, p2_id*3) = hessianPortion(0, 3);
//        newHessian(p1_id*3, p2_id*3+1) = hessianPortion(0, 4);
//        newHessian(p1_id*3, p2_id*3+2) = hessianPortion(0, 5);
//        newHessian(p1_id*3+1, p2_id*3) = hessianPortion(1, 3);
//        newHessian(p1_id*3+1, p2_id*3+1) = hessianPortion(1, 4);
//        newHessian(p1_id*3+1, p2_id*3+2) = hessianPortion(1, 5);
//        newHessian(p1_id*3+2, p2_id*3) = hessianPortion(2, 3);
//        newHessian(p1_id*3+2, p2_id*3+1) = hessianPortion(2, 4);
//        newHessian(p1_id*3+2, p2_id*3+2) = hessianPortion(2, 5);
//
//        newHessian(p2_id*3, p1_id*3) = hessianPortion(3, 0);
//        newHessian(p2_id*3, p1_id*3+1) = hessianPortion(3, 1);
//        newHessian(p2_id*3, p1_id*3+2) = hessianPortion(3, 2);
//        newHessian(p2_id*3+1, p1_id*3) = hessianPortion(4, 0);
//        newHessian(p2_id*3+1, p1_id*3+1) = hessianPortion(4, 1);
//        newHessian(p2_id*3+1, p1_id*3+2) = hessianPortion(4, 2);
//        newHessian(p2_id*3+2, p1_id*3) = hessianPortion(5, 0);
//        newHessian(p2_id*3+2, p1_id*3+1) = hessianPortion(5, 1);
//        newHessian(p2_id*3+2, p1_id*3+2) = hessianPortion(5, 2);
//
//    }
//
//    // if any external forces impart hessians, need to have those here as well!
//
//    Eigen::MatrixXf clause2 = timeStep*timeStep*newHessian;
//    hessian = massMatrix - clause2;
//}

void Simulation::optimizationImplicitEuler() {
    Eigen::VectorXf newParticleState = applyNewtonsMethod_Test();
    for (int i = 0; i < particles.size(); i++) {
        Eigen::Vector3f newParticlePosition;
        int particleID = particles[i]->getID();
        newParticlePosition[0] = newParticleState[particleID*3];
        newParticlePosition[1] = newParticleState[particleID*3+1];
        newParticlePosition[2] = newParticleState[particleID*3+2];
        particles[i]->assignNewPosition(newParticlePosition);
    }
}

//void Simulation::optimizationImplicitEuler_ByParticle() {
//    for (int i = 0; i < particles.size(); i++) {
//        Eigen::Vector3f newParticlePosition = applyNewtonsMethod(*particles[i]);
//        particles[i]->assignNewPosition(newParticlePosition);
//    }
//}

void Simulation::timeStepping() {
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->stepForward(timeStep);
    }
}

void Simulation::computeNewParticleStates(StateComputationMode mode) {
    switch (mode) {
        case TIMESTEP:
            timeStepping();
            break;
        case OPTIMIZATION_IMPLICIT_EULER:
            optimizationImplicitEuler();
            break;
//        case OPTIMIZATION_IMPLICIT_EULER_BY_PARTICLE:
//            optimizationImplicitEuler_ByParticle();
//            break;
    }
}

void Simulation::addExternalForce(Eigen::Vector3f (*newForce)(SpringEndpoint), ForceType forceType) {
    externalForces.push_back(std::pair<Eigen::Vector3f (*)(SpringEndpoint), ForceType>(newForce, forceType));
}

Eigen::Vector3f Simulation::calculateSpringForce(Eigen::Vector3f p1, Eigen::Vector3f p2, float r, float k) {
    
    float displacementX = p1[0] - p2[0];
    float displacementY = p1[1] - p2[1];
    float displacementZ = p1[2] - p2[2];

    float displacementXY = sqrt(displacementX*displacementX + displacementY*displacementY);
    float displacement = sqrt(displacementXY*displacementXY + displacementZ*displacementZ);

    float force = k * (abs(displacement)/r - 1);

    float theta = atan(abs(displacementY) / abs(displacementX));
    float phi = atan(abs(displacementZ) / abs(displacementXY));

    float forceX = force * cos(theta);
    float forceY = force * sin(theta);
    float forceZ = force * sin(phi);

    float dirX = (-displacementX/abs(displacementX));
    float dirY = (-displacementY/abs(displacementY));
    float dirZ = (-displacementZ/abs(displacementZ));

    float fx = displacementX == 0 ? 0 : forceX * dirX;
    float fy = displacementY == 0 ? 0 : forceY * dirY;
    float fz = displacementZ == 0 ? 0 : forceZ * dirZ;

    return Eigen::Vector3f(fx, fy, fz);
}

//void Simulation::applyForces() {
//    for (int i = 0; i < particles.size(); i++) {
//        particles[i]->resetForces();
//
//        for (int j = 0; j < externalForces.size(); j++) {
//            particles[i]->addForce(externalForces[j](particles[i]));
//        }
//    }
//}
