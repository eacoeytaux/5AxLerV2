//
//  BuildMapToMATLAB.hpp
//  5AxLerV2
//
//  Created by Ethan Coeytaux on 3/26/17.
//  Copyright © 2017 mapmqp. All rights reserved.
//

#ifndef BuildMapToMATLAB_hpp
#define BuildMapToMATLAB_hpp

#include <string>

#include "BuildMap.hpp"

namespace cura {
    class BuildMapToMATLAB {
    public:
        enum OutputType {
            PLANE,
            SPHERE,
            SPHERE_SMOOTH
        };
        
        static bool parse(std::string filePath, const BuildMap & buildMap, OutputType type, int precision = 1);
    };
}

#endif /* BuildMapToMATLAB_hpp */
