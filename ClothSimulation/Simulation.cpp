//
//  Simulation.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-06-04.
//

#include "Simulation.hpp"

Simulation::Simulation(float newTimeStep, vector<Particle> newParticles, Canvas newCanvas, StateComputationMode curMode) {
    timeStep = newTimeStep;
    time = 0;
    particles = newParticles;
    canvas = newCanvas;
    mode = curMode;
}

void Simulation::update() {
    canvas.drawParticles(particles);
    
    applyExternalForces();
    
    computeNewParticleStates(mode);
    
    time += timeStep;
}

Eigen::Vector3f Simulation::applyNewtonsMethod(Particle p) {
    Eigen::Vector3f curGuessPosition = Eigen::Vector3f(5, 6, 7);
    Eigen::Vector3f gradient = p.evaluateGradient(curGuessPosition, timeStep, time);
    int numIts = 1;
    while (gradient.squaredNorm() > __FLT_EPSILON__) {
        cout <<"hello..."<<endl;
        Eigen::Matrix3f hessian = p.evaluateHessian(curGuessPosition, timeStep, time);
        Eigen::Vector3f nextGuessPosition = curGuessPosition - hessian.inverse()*gradient;
        curGuessPosition = nextGuessPosition;
        gradient = p.evaluateGradient(curGuessPosition, timeStep, time);
        numIts++;
    }
    cout << numIts << endl;
    return curGuessPosition;
}

void Simulation::optimizationImplicitEuler() {
    for (int i = 0; i < particles.size(); i++) {
        Eigen::Vector3f newParticlePosition = applyNewtonsMethod(particles[i]);
        cout << newParticlePosition << endl;
        particles[i].assignNewPosition(newParticlePosition);
    }
}

void Simulation::timeStepping() {
    for (int i = 0; i < particles.size(); i++) {
        particles[i].stepForward(timeStep);
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
    }
}

void Simulation::addExternalForce(Eigen::Vector3f (*newForce)(Particle, float)) {
    externalForces.push_back(newForce);
}

void Simulation::applyExternalForces() {
    for (int i = 0; i < particles.size(); i++) {
        particles[i].resetForces();
        for (int j = 0; j < externalForces.size(); j++) {
            particles[i].addExternalForce(externalForces[i]);
        }
        particles[i].computeResultingForce(time);
    }
}
