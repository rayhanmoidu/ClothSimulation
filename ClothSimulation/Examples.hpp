//
//  Examples.hpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-06-26.
//

#ifndef Examples_hpp
#define Examples_hpp

#include <stdio.h>
#include "Spring.hpp"
#include "SpringEndpoint.hpp"

struct ClothRepresentation {
    ClothRepresentation(std::vector<Spring> s, std::vector<SpringEndpoint*> e, std::vector<int> i) {
        springs = s;
        endpoints = e;
        fixedIds = i;
    }
    std::vector<Spring> springs;
    std::vector<SpringEndpoint*> endpoints;
    std::vector<int> fixedIds;
};

enum ExampleType {
    BASIC_PYRAMID,
    BASIC_CLOTH,
};

class Examples {
public:
    Examples();
    ClothRepresentation getExample(ExampleType);
    
private:
    ClothRepresentation getExample_BasicPyramid();
    ClothRepresentation getExample_BasicCloth();
};

#endif /* Examples_hpp */
