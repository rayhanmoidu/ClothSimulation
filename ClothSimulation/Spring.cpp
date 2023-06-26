//
//  Spring.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-23.
//

#include "Spring.hpp"
#include <cmath>
#include <iostream>

Spring::Spring(SpringEndpoint* pp1, SpringEndpoint* pp2, float r, float ks, float kd) {
    restLength = r;
    springConstant = ks;
    dampingConstant = kd;
    p1 = pp1;
    p2 = pp2;
}

float Spring::getRestLength() {
    return restLength;
}
float Spring::getSpringConstant() {
    return springConstant;
}

vector<SpringEndpoint*> Spring::getEndpoints() {
    vector<SpringEndpoint*> ret;
    ret.push_back(p1);
    ret.push_back(p2);
    return ret;
}
