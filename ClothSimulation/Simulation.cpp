//
//  Simulation.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-06-04.
//

#include "Simulation.hpp"
#include <eigen3/Eigen/Sparse>

Simulation::Simulation(ClothRepresentation cloth, float newTimeStep, Canvas newCanvas, StateComputationMode curMode) {
    timeStep = newTimeStep;
    time = 0;
    particles = cloth.endpoints;
    canvas = newCanvas;
    mode = curMode;
    
    springs = cloth.springs;
    
    computeNeighbourRelationships();
    
    n = particles.size();
    hessian = Eigen::MatrixXf(3*n, 3*n);
    gradient = Eigen::VectorXf(3*n);
    
    fixedIds = cloth.fixedIds;
    
    evaluateMassMatrix();
}

void Simulation::computeNeighbourRelationships() {
    for (int i = 0; i < particles.size(); i++) {
        for (int j = 0; j < springs.size(); j++) {
            vector<SpringEndpoint*> endpoints = springs[j].getEndpoints();
            if (endpoints[0]==particles[i]) {
                particles[i]->addNeighbourEndpoint(endpoints[1]);
            } else if (endpoints[1]==particles[i]) {
                particles[i]->addNeighbourEndpoint(endpoints[0]);
            }
        }
    }
}

bool Simulation::isParticleFixed(SpringEndpoint* p) {
    for (int i = 0; i < fixedIds.size(); i++) {
        if (fixedIds[i] == p->getID()) return true;
    }
    return false;
}

void Simulation::update() {
    canvas.drawParticles(particles);
    
    if (mode!=OPTIMIZATION_IMPLICIT_EULER) {
    applyExternalForces();
    }
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

bool isGradientSatisfied(Eigen::VectorXf grad) {
    for (int i = 0; i < grad.size(); i++) {
        if (grad[i]*grad[i] > __FLT_EPSILON__ *pow(grad.size(), 2)) return false;
    }
    return true;
}

Eigen::VectorXf Simulation::applyNewtonsMethod() {
    Eigen::VectorXf curGuessPosition(3*n);
    for (int i = 0; i < 3*n; i++) {
        curGuessPosition[i] = 0;
    }
    
    curGuessPosition = getPrevPosition();
    evaluateGradient(curGuessPosition);
//    while (!isGradientSatisfied(gradient)) {
    while (gradient.squaredNorm() > (__FLT_EPSILON__ * pow(gradient.size(), 2))) {
//    while (gradient.squaredNorm() > 0.5) {
//        cout << gradient.squaredNorm()<<endl;
        //cout <<"hello..."<<endl;
        evaluateHessian(curGuessPosition);
        Eigen::VectorXf nextGuessPosition = curGuessPosition - hessian.inverse()*gradient;
        curGuessPosition = nextGuessPosition;
        evaluateGradient(curGuessPosition);
//        cout <<"finding"<<endl;
//        cout << gradient << endl;
//        cout << gradient.squaredNorm() << endl;
//        cout << __FLT_EPSILON__ << endl;
    }
    //cout <<"found!"<<endl;
    //cout << curGuessPosition << endl;
    //cout << numIts << endl;
    return curGuessPosition;
}

Eigen::Vector3f Simulation::applyNewtonsMethod_ByParticle(SpringEndpoint p) {
    Eigen::Vector3f curGuessPosition = Eigen::Vector3f(1, 1, 1);
    Eigen::Vector3f gradient = p.evaluateGradient(curGuessPosition, timeStep, time);
    int numIts = 1;
    while (gradient.squaredNorm() > __FLT_EPSILON__) {
        //cout <<"hello..."<<endl;
        Eigen::Matrix3f hessian = p.evaluateHessian(curGuessPosition, timeStep, time);
        Eigen::Vector3f nextGuessPosition = curGuessPosition - hessian.inverse()*gradient;
        curGuessPosition = nextGuessPosition;
        gradient = p.evaluateGradient(curGuessPosition, timeStep, time);
        numIts++;
        //cout <<"finding"<<endl;
    }
    //cout <<"found!"<<endl;
    //cout << curGuessPosition << endl;
    //cout << numIts << endl;
    return curGuessPosition;
}

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
    
    for (int i = 0; i < springs.size(); i++) {
        vector<SpringEndpoint*> endpoints = springs[i].getEndpoints();

        int p1_id = endpoints[0]->getID();
        int p2_id = endpoints[1]->getID();

        Eigen::Vector3f p1_guessPosPortion = Eigen::Vector3f(curGuessPosition[p1_id*3], curGuessPosition[p1_id*3 + 1], curGuessPosition[p1_id*3 + 2]);
        Eigen::Vector3f p2_guessPosPortion = Eigen::Vector3f(curGuessPosition[p2_id*3], curGuessPosition[p2_id*3 + 1], curGuessPosition[p2_id*3 + 2]);

        Eigen::Vector3f forceOnP1 = calculateSpringForce(p1_guessPosPortion, p2_guessPosPortion, springs[i].getRestLength(), springs[i].getSpringConstant());
        Eigen::Vector3f forceOnP2 = calculateSpringForce(p2_guessPosPortion, p1_guessPosPortion, springs[i].getRestLength(), springs[i].getSpringConstant());

        for (int c = 0; c < 3; c++) {
            if (!isParticleFixed(endpoints[0])) {
                newGradient[p1_id*3 + c] += forceOnP1[c];
            }
            if (!isParticleFixed(endpoints[1])) {
                newGradient[p2_id*3 + c] += forceOnP2[c];
            }
        }
    }
    
    for (int i = 0; i < particles.size(); i++) {
        int p_id = particles[i]->getID();
        Eigen::Vector3f externalForce(0, 0, 0);

        for (int j = 0; j < externalForces.size(); j++) {
            // todo why do I need 2 here?
            externalForce += externalForces[j].first(*particles[i], *particles[i], 0);
//            externalForce += particles[i]->getMass()*Eigen::Vector3f(0, -90.81, 0);
        }

        for (int c = 0; c < 3; c++) {
            if (!isParticleFixed(particles[i])) {
                newGradient[p_id*3 + c] += externalForce[c];
            }
        }
    }
    
    Eigen::VectorXf curPos = getCurPosition();
    Eigen::VectorXf prevPos = getPrevPosition();
    
    Eigen::VectorXf y = 2*curPos - prevPos;
    Eigen::VectorXf clause2 = massMatrix * (curGuessPosition - y);
    Eigen::VectorXf clause1 = (timeStep * timeStep) * newGradient;
    gradient = clause2 - clause1;
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
    hessian << koverr, 0, 0, koverr, 0, 0,
    0, koverr, 0, 0, koverr, 0,
    0, 0, koverr, 0, 0, koverr,
    koverr, 0, 0, koverr, 0, 0,
    0, koverr, 0, 0, koverr, 0,
    0, 0, koverr, 0, 0, koverr;
//        hessian << 0, 0, 0, 0, 0, 0,
//        0, 0, 0, 0, 0, 0,
//        0, 0, 0, 0, 0, 0,
//        0, 0, 0, 0, 0, 0,
//        0, 0, 0, 0, 0, 0,
//        0, 0, 0, 0, 0, 0;
    return hessian;
}

vector<Eigen::Triplet<float>> getHessianTriplets(Eigen::Vector3f p1, Eigen::Vector3f p2) {
    float koverr = -14000/200;
    vector<Eigen::Triplet<float>> retVal;
    retVal.push_back(Eigen::Triplet<float>(p1_id*3, p1_id*3, koverr));
    retVal.push_back(Eigen::Triplet<float>(p1_id*3+1, p1_id*3+1, koverr));
    retVal.push_back(Eigen::Triplet<float>(p1_id*3+2, p1_id*3+2, koverr));
}

void Simulation::evaluateHessian(Eigen::VectorXf curGuessPosition) {
    // hmm... might want to just do the diagnoals
    Eigen::MatrixXf newHessian(3*n, 3*n);
    for (int i = 0; i < 3*n; i++) {
        for (int j = 0; j < 3*n; j++) {
            newHessian(i, j) = 0;
        }
    }
//    Eigen::SparseMatrix<float> newHessian(3*n, 3*n);
    for (int i = 0; i < springs.size(); i++) {
        vector<SpringEndpoint*> endpoints = springs[i].getEndpoints();

        int p1_id = endpoints[0]->getID();
        int p2_id = endpoints[1]->getID();

        Eigen::Vector3f p1_guessPosPortion = Eigen::Vector3f(curGuessPosition[p1_id*3], curGuessPosition[p1_id*3 + 1], curGuessPosition[p1_id*3 + 2]);
        Eigen::Vector3f p2_guessPosPortion = Eigen::Vector3f(curGuessPosition[p2_id*3], curGuessPosition[p2_id*3 + 1], curGuessPosition[p2_id*3 + 2]);

        Eigen::MatrixXf hessianPortion = evaluateHessian_Portion(p1_guessPosPortion, p2_guessPosPortion);

        newHessian(p1_id*3, p2_id*3) += hessianPortion(0, 3);
        newHessian(p1_id*3, p2_id*3+1) += hessianPortion(0, 4);
        newHessian(p1_id*3, p2_id*3+2) += hessianPortion(0, 5);
        newHessian(p1_id*3+1, p2_id*3) += hessianPortion(1, 3);
        newHessian(p1_id*3+1, p2_id*3+1) += hessianPortion(1, 4);
        newHessian(p1_id*3+1, p2_id*3+2) += hessianPortion(1, 5);
        newHessian(p1_id*3+2, p2_id*3) += hessianPortion(2, 3);
        newHessian(p1_id*3+2, p2_id*3+1) += hessianPortion(2, 4);
        newHessian(p1_id*3+2, p2_id*3+2) += hessianPortion(2, 5);
        
        newHessian(p1_id*3, p1_id*3) += hessianPortion(0, 0);
        newHessian(p1_id*3, p1_id*3+1) += hessianPortion(0, 1);
        newHessian(p1_id*3, p1_id*3+2) += hessianPortion(0, 2);
        newHessian(p1_id*3+1, p1_id*3) += hessianPortion(1, 0);
        newHessian(p1_id*3+1, p1_id*3+1) += hessianPortion(1, 1);
        newHessian(p1_id*3+1, p1_id*3+2) += hessianPortion(1, 2);
        newHessian(p1_id*3+2, p1_id*3) += hessianPortion(2, 0);
        newHessian(p1_id*3+2, p1_id*3+1) += hessianPortion(2, 1);
        newHessian(p1_id*3+2, p1_id*3+2) += hessianPortion(2, 2);

        newHessian(p2_id*3, p1_id*3) += hessianPortion(3, 0);
        newHessian(p2_id*3, p1_id*3+1) += hessianPortion(3, 1);
        newHessian(p2_id*3, p1_id*3+2) += hessianPortion(3, 2);
        newHessian(p2_id*3+1, p1_id*3) += hessianPortion(4, 0);
        newHessian(p2_id*3+1, p1_id*3+1) += hessianPortion(4, 1);
        newHessian(p2_id*3+1, p1_id*3+2) += hessianPortion(4, 2);
        newHessian(p2_id*3+2, p1_id*3) += hessianPortion(5, 0);
        newHessian(p2_id*3+2, p1_id*3+1) += hessianPortion(5, 1);
        newHessian(p2_id*3+2, p1_id*3+2) += hessianPortion(5, 2);
        
        newHessian(p2_id*3, p2_id*3) += hessianPortion(3, 3);
        newHessian(p2_id*3, p2_id*3+1) += hessianPortion(3, 4);
        newHessian(p2_id*3, p2_id*3+2) += hessianPortion(3, 5);
        newHessian(p2_id*3+1, p2_id*3) += hessianPortion(4, 3);
        newHessian(p2_id*3+1, p2_id*3+1) += hessianPortion(4, 4);
        newHessian(p2_id*3+1, p2_id*3+2) += hessianPortion(4, 5);
        newHessian(p2_id*3+2, p2_id*3) += hessianPortion(5, 3);
        newHessian(p2_id*3+2, p2_id*3+1) += hessianPortion(5, 4);
        newHessian(p2_id*3+2, p2_id*3+2) += hessianPortion(5, 5);
    }
    
    // if any external forces are imparting hessian values, add them here
    
    Eigen::MatrixXf clause2 = timeStep*timeStep*newHessian;
    hessian = massMatrix - clause2;
}

void Simulation::optimizationImplicitEuler() {
    //cout<<"applying newton"<<endl;
    Eigen::VectorXf newParticleState = applyNewtonsMethod();
    //cout<<"done!"<<endl;
    for (int i = 0; i < particles.size(); i++) {
        Eigen::Vector3f newParticlePosition;
        int particleID = particles[i]->getID();
        newParticlePosition[0] = newParticleState[particleID*3];
        newParticlePosition[1] = newParticleState[particleID*3+1];
        newParticlePosition[2] = newParticleState[particleID*3+2];
        particles[i]->assignNewPosition(newParticlePosition);
    }
}

void Simulation::optimizationImplicitEuler_ByParticle() {
    for (int i = 0; i < particles.size(); i++) {
        Eigen::Vector3f newParticlePosition = applyNewtonsMethod_ByParticle(*particles[i]);
        if (!isParticleFixed(particles[i])) {
            particles[i]->assignNewPosition(newParticlePosition);
        }
    }
}

void Simulation::timeStepping() {
    for (int i = 0; i < particles.size(); i++) {
        if (!isParticleFixed(particles[i])) {
            particles[i]->stepForward(timeStep);
        }
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
        case OPTIMIZATION_IMPLICIT_EULER_BY_PARTICLE:
            optimizationImplicitEuler_ByParticle();
            break;
    }
}

void Simulation::addExternalForce(Eigen::Vector3f (*newForce)(SpringEndpoint, SpringEndpoint, float), ForceType forceType) {
    externalForces.push_back(std::pair<Eigen::Vector3f (*)(SpringEndpoint, SpringEndpoint, float), ForceType>(newForce, forceType));
}

void Simulation::applyExternalForces() {
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->resetForces();
        for (int j = 0; j < externalForces.size(); j++) {
            particles[i]->addExternalForce(externalForces[j]);
        }
        particles[i]->computeResultingForce(time);
    }
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
