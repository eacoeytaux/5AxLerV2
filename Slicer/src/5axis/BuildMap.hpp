//
//  BuildMap.hpp
//  generator
//
//  Created by Ethan Coeytaux on 10/6/16.
//  Copyright © 2016 MAP MQP. All rights reserved.
//

#ifndef BuildMap_hpp
#define BuildMap_hpp

#include <vector>

#include <clipper/clipper.hpp>

#include "../mesh.h"
#include "../utils/intpoint.h"

namespace cura {
    class BuildMap {
    public:
        BuildMap(const Mesh & mesh);
        
        double area() const;
        bool checkVector(const FPoint3 & v, bool includeEdges = true) const;
        std::vector<FPoint3> findValidVectors() const;
        FPoint3 findBestVector() const;
        double averageCuspHeight(const FPoint3 & v) const;
        
        static FPoint3 mapToFPoint3(int x, int y);
        static std::pair<int, int> FPoint3ToMap(const FPoint3 & v);
        static int thetaToBAxisRange(double theta);
        static int phiToAAxisRange(double phi);
        static double bAxisValToTheta(double bAxisVal);
        static double aAxisValToPhi(double aAxisVal);
        
    private:
        const Mesh & m_mesh;
        ClipperLib::Paths m_buildMap2D; //x->theta, y->phi
        bool m_phiZeroAvailable = true; //whether or not the point at phi = 0 is true
        
        std::pair<std::pair<int, int>, double> hillClimb(int x, int y) const;
        
        //        std::vector<FPoint3> findValidVectorsUtil(int xStart, int yStart, int width, int height) const;
        //        std::pair<FPoint3, double> findBestVectorUtil(int x, int y, double prevHeuristic, int depth = 0) const;
    };
}

#endif /* BuildMap_hpp */
