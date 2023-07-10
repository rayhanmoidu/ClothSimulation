//
//  Examples.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-06-26.
//

#include "Examples.hpp"

Examples::Examples() {
    
}

ClothRepresentation Examples::getExample(ExampleType type) {
    switch (type) {
        case BASIC_PYRAMID:
            return getExample_BasicPyramid();
            break;
        case BASIC_CLOTH:
            return getExample_BasicCloth();
            break;
        default:
            return getExample_BasicPyramid();
            break;
    }
}

ClothRepresentation Examples::getExample_BasicCloth() {
    int adder = 150;
    int springKVal = 14000;
    
    std::vector<std::vector<SpringEndpoint*>> springEndpoints;
    int id = 0;
    for (int i = 50; i <= 950; i+=adder) {
        std::vector<SpringEndpoint*> newRow;
        for (int j = 50; j <= 950; j+=adder) {
            SpringEndpoint* springEndpoint = new SpringEndpoint(id++, Eigen::Vector3f(i, j, 0), 5);
            newRow.push_back(springEndpoint);
        }
        springEndpoints.push_back(newRow);
    }

    //
    
    int dim = 900 / adder;
    std::vector<int> fixedIds;

    for (int i = dim; i < (dim+1)*(dim+1); i+= dim+1) {
        fixedIds.push_back(i);
    }
    
    //
    
    std::vector<Spring> springs;
    
    //horizontal directions
    for (int i = 0; i < springEndpoints.size(); i++) {
        vector<SpringEndpoint*> curRow = springEndpoints[i];
        for (int j = 0; j < curRow.size() -1; j++) {
            Spring s1(curRow[j], curRow[j+1], 100, springKVal, 0);
            springs.push_back(s1);
        }
    }
    
    //vertical directions
    for (int i = 0; i < springEndpoints.size() - 1; i++) {
        for (int j = 0; j < springEndpoints[i].size(); j++) {
            Spring s1(springEndpoints[i][j], springEndpoints[i+1][j], 100, springKVal, 0);
            springs.push_back(s1);

        }
    }
    
    // forward diagnoals
    for (int i = 0; i < springEndpoints.size() - 1; i++) {
        for (int j = 0; j < springEndpoints[i].size() - 1; j++) {
            Spring s1(springEndpoints[i][j], springEndpoints[i+1][j+1], 100, springKVal, 0);
            springs.push_back(s1);

        }
    }
    
    // backward diagnoals
    for (int i = 0; i < springEndpoints.size() - 1; i++) {
        for (int j = 1; j < springEndpoints[i].size(); j++) {
            Spring s1(springEndpoints[i][j], springEndpoints[i+1][j-1], 100, springKVal, 0);
            springs.push_back(s1);
        }
    }
    
    vector<SpringEndpoint*> finalEndpoints;
    for (int i = 0; i < springEndpoints.size() ; i++) {
        for (int j = 0; j < springEndpoints[i].size(); j++) {
            finalEndpoints.push_back(springEndpoints[i][j]);
        }
    }
    
    return ClothRepresentation(springs, finalEndpoints, fixedIds);
}


ClothRepresentation Examples::getExample_BasicPyramid() {
    std::vector<SpringEndpoint*> springEndpoints;
    SpringEndpoint* springEndpoint1 = new SpringEndpoint(0, Eigen::Vector3f(400, 500, 0), 5);
    SpringEndpoint* springEndpoint2 = new SpringEndpoint(1, Eigen::Vector3f(600, 500, 0), 5);
    SpringEndpoint* springEndpoint3 = new SpringEndpoint(2, Eigen::Vector3f(200, 500, 0), 5);
    SpringEndpoint* springEndpoint4 = new SpringEndpoint(3, Eigen::Vector3f(500, 1000, 13), 5);
    
    springEndpoints.push_back(springEndpoint1);
    springEndpoints.push_back(springEndpoint4);
    springEndpoints.push_back(springEndpoint2);
    springEndpoints.push_back(springEndpoint3);
    
    //
    
    std::vector<int> fixedIds;
    fixedIds.push_back(3);
    
    //
    
    std::vector<Spring> springs;
    
    Spring s1(springEndpoint1, springEndpoint2, 200, 14000, 0);
    Spring s2(springEndpoint1, springEndpoint3, 200, 14000, 0);
    Spring s3(springEndpoint2, springEndpoint3, 200, 14000, 0);
    
    Spring s4(springEndpoint1, springEndpoint4, 200, 14000, 0);
    Spring s5(springEndpoint2, springEndpoint4, 200, 14000, 0);
    Spring s6(springEndpoint3, springEndpoint4, 200, 14000, 0);
    
    springs.push_back(s1);
    springs.push_back(s2);
    springs.push_back(s3);
    springs.push_back(s4);
    springs.push_back(s5);
    springs.push_back(s6);
    
    //
    
    return ClothRepresentation(springs, springEndpoints, fixedIds);
}
