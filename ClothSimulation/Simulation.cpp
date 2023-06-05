//
//  Simulation.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-06-04.
//

#include "Simulation.hpp"

Simulation::Simulation(float newTimeStep, vector<Particle> newParticles, Canvas newCanvas) {
    timeStep = newTimeStep;
    time = 0;
    particles = newParticles;
    canvas = newCanvas;
}

void Simulation::update() {
    canvas.drawParticles(particles);
    
    applyExternalForces();
    
    computeNewParticleStates();
    
    time += timeStep;
}

void Simulation::computeNewParticleStates() {
    for (int i = 0; i < particles.size(); i++) {
        particles[i].stepForward(timeStep);
    }
}

void Simulation::addExternalForce(Eigen::Vector3f (*newForce)(Particle, float)) {
    externalForces.push_back(newForce);
}

void Simulation::applyExternalForces() {
    for (int i = 0; i < particles.size(); i++) {
        for (int j = 0; j < externalForces.size(); j++) {
            particles[i].applyExternalForce(externalForces[i](particles[i], time));
        }
    }
}
