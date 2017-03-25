//
//  BuildMap.cpp
//  generator
//
//  Created by Ethan Coeytaux on 10/6/16.
//  Copyright © 2016 MAP MQP. All rights reserved.
//

#include "BuildMap.hpp"

#include <cmath>
#include <vector>

#include "Utility.hpp"

#define ELLIPSE_PRECISION 100

using namespace cura;
using namespace std;
using namespace ClipperLib;

BuildMap::BuildMap(const Mesh & mesh) :
m_mesh(mesh) {
    //all statics - should only be executed once
    
    static vector<pair<int, int>> ellipseCoors; //ellipse to remove from build map with center of v.theta() and v.phi()
    static bool ellipseBuilt = false; //whether or not ellipse has been calculated
    
    if (!ellipseBuilt) {
        //use ceil() to over-estimate area
        double deltaTheta = thetaToBAxisRange(THETA_MAX);
        double deltaPhi = phiToAAxisRange(THETA_MAX);
        
        //to overapproximate ellipse, we extend the radius by this constant
        double radiusExtension = 1.0 / cos(M_PI / ELLIPSE_PRECISION);
        
        log("[INFO] BUILD MAP - delta-theta: %f", deltaTheta);
        log("[INFO] BUILD MAP - delta-phi: %f", deltaPhi);
        log("[INFO] BUILD MAP - ellipse area: %f", M_PI * deltaTheta * deltaPhi);
        log("[INFO] BUILD MAP - radius extension: %f", radiusExtension);
        
        //polygon goes in clockwise form
        for (unsigned int i = 0; i < ELLIPSE_PRECISION; i++) {
            double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(ELLIPSE_PRECISION);
            
            double xDouble = deltaTheta * radiusExtension * cos(angle);
            double yDouble = deltaPhi * radiusExtension * sin(angle);
            
            int xInt = (xDouble > 0) ? ceil(xDouble) : floor(xDouble);
            int yInt = (yDouble > 0) ? ceil(yDouble) : floor(yDouble);
            
            ellipseCoors.push_back(pair<int, int>(xInt, yInt));
        }
        
        ellipseBuilt = true;
    }
    
    //end static section
    
    //remove all contraints from face normals
    Paths holes;
    for (vector<MeshFace>::const_iterator it = m_mesh.faces.begin(); it != m_mesh.faces.end(); it++) {
        FPoint3 v = faceNormal(m_mesh, *it);
        v = v * -1;
        
        if (phiFromCartesian(v) == 0) {
            m_phiZeroAvailable = false;
            
            Path hole;
            hole << ClipperLib::IntPoint(0, 0) << ClipperLib::IntPoint(0, phiToAAxisRange(THETA_MAX)) << ClipperLib::IntPoint(B_AXIS_DISCRETE_POINTS, phiToAAxisRange(THETA_MAX)) << ClipperLib::IntPoint(B_AXIS_DISCRETE_POINTS, 0);
            
            //union all holes into one polygon
            Clipper holeClipper;
            holeClipper.AddPaths(holes, ptSubject, true);
            holeClipper.AddPath(hole, ptClip, true);
            if (!holeClipper.Execute(ctUnion, holes, pftNonZero, pftNonZero)) {
                log("[ERROR] BUILD MAP - error taking union of holes");
                //TODO return false;
            }
        } else {
            //if top point of build map is covered, set phiZeroAvailable to false
            m_phiZeroAvailable &= (fabs(phiFromCartesian(v)) > THETA_MAX);
            
            int xCenter = thetaToBAxisRange(thetaFromCartesian(v));
            int yCenter = phiToAAxisRange(phiFromCartesian(v));
            
            double sinPhi = sin(phiFromCartesian(v));
            
            Path hole,
            holeWrapThetaPos,
            holeWrapThetaNeg;
            
            bool wrapAroundThetaPos = false,
            wrapAroundThetaNeg = false;
            
            for (vector<pair<int, int>>::iterator it = ellipseCoors.begin(); it < ellipseCoors.end(); it++) {
                int x = (it->first / sinPhi) + xCenter;
                int y = it->second + yCenter;
                hole << ClipperLib::IntPoint(x, y);
                
                if (!wrapAroundThetaPos) {
                    if (x > B_AXIS_DISCRETE_POINTS) {
                        wrapAroundThetaPos = true;
                    }
                }
                holeWrapThetaPos << ClipperLib::IntPoint(x - B_AXIS_DISCRETE_POINTS, y);
                
                if (!wrapAroundThetaNeg) {
                    if (x < 0) {
                        wrapAroundThetaNeg = true;
                    }
                }
                holeWrapThetaNeg << ClipperLib::IntPoint(x + B_AXIS_DISCRETE_POINTS, y);
            }
            
            //union all holes into one polygon
            Clipper holeClipper;
            holeClipper.AddPaths(holes, ptSubject, true);
            
            holeClipper.AddPath(hole, ptClip, true);
            if (wrapAroundThetaPos) {
                holeClipper.AddPath(holeWrapThetaPos, ptClip, true);
            }
            if (wrapAroundThetaNeg) {
                holeClipper.AddPath(holeWrapThetaNeg, ptClip, true);
            }
            
            if (!holeClipper.Execute(ctUnion, holes, pftNonZero, pftNonZero)) {
                log("[ERROR] BUILD MAP - error taking union of holes");
                //TODO return false;
            }
        }
        
        //TODO will this be more efficient?
        //TODO check area of holes and if it's >= total build map area then we know the build map is empty and we can stop here
        
        //SimplifyPolygons(holes_); //I don't think this is necessary and takes extra time but not sure
    }
    
    Clipper buildMapClipper;
    //set up subject (only look in this box)
    Path subject;
    subject << ClipperLib::IntPoint(0, 0) << ClipperLib::IntPoint(0, A_AXIS_DISCRETE_POINTS) << ClipperLib::IntPoint(B_AXIS_DISCRETE_POINTS, A_AXIS_DISCRETE_POINTS) << ClipperLib::IntPoint(B_AXIS_DISCRETE_POINTS, 0);
    buildMapClipper.AddPath(subject, ptSubject, true);
    buildMapClipper.AddPaths(holes, ptClip, true);
    if (!buildMapClipper.Execute(ctDifference, m_buildMap2D, pftNonZero, pftNonZero)) {
        log("[ERROR] BUILD MAP - error taking difference of map and holes");
        //TODO return false;
    }
}

double BuildMap::area() const {
    if (m_buildMap2D.size() == 0) { //check to make sure paths are not empty (meaning empty area)
        return 0;
    }
    
    double area = 0;
    for (unsigned int i = 0; i < m_buildMap2D.size(); i++) {
        area += Area(m_buildMap2D[i]);
    }
    return area;
}

bool BuildMap::checkVector(const FPoint3 & v, bool includeEdges) const {
    if (area() == 0) { //build map is empty
        return false;
    } else if (phiFromCartesian(v) == 0) {
        return m_phiZeroAvailable;
    }
    
    //will return 0 if false, -1 if on edge, 1 otherwise
    int pointIn = PointInPolygon(ClipperLib::IntPoint(thetaToBAxisRange(thetaFromCartesian(v)), phiToAAxisRange(phiFromCartesian(v))), m_buildMap2D[0]);
    return (includeEdges ? (pointIn != 0) : (pointIn == 1));
}

FPoint3 BuildMap::findValidVector() const {
    if (area() == 0) {
        return FPoint3(0, 0, 0);
    }
    
    //TODO can this just be done by grabbing a point from the outline of m_buildMap2D?
    
    FPoint3 v = findValidVectorUtil(0, 0, B_AXIS_DISCRETE_POINTS, A_AXIS_DISCRETE_POINTS);
    if (!checkVector(v)) {
        log("[ERROR] BUILD MAP - arbitrary vector(%f, %f, %f) not in buildmap", v.x, v.y, v.z);
    }
    return v;
}

FPoint3 BuildMap::findValidVectorUtil(int xStart, int yStart, int width, int height) const {
#ifdef DEBUG_MODE
    //writeLog(INFO, "BUILD MAP - checking build map theta(%d-%d) phi(%d-%d)", xStart, xStart + width, yStart, yStart + height);
#endif
    if ((width == 1) && (height == 1)) {
        return FPoint3FromSpherical(BuildMap::bAxisValToTheta(xStart), BuildMap::aAxisValToPhi(yStart));
    }
    
    Clipper searchClipper;
    searchClipper.AddPaths(m_buildMap2D, ptSubject, true);
    
    bool cutHorizontally = width < height;
    int dx = (cutHorizontally ? width : ceil(static_cast<double>(width) / 2.0));
    int dy = (cutHorizontally ? ceil(static_cast<double>(height) / 2.0) : height);
    
    Path search;
    search << ClipperLib::IntPoint(xStart, yStart) << ClipperLib::IntPoint(xStart, yStart + dy) << ClipperLib::IntPoint(xStart + dx, yStart + dy) << ClipperLib::IntPoint(xStart + dx, yStart);
    searchClipper.AddPath(search, ptClip, true);
    
    Paths solution;
    searchClipper.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
    
    bool searchSuccess;
    if (solution.size() != 0) {
        double area = 0;
        for (unsigned int i = 0; i < solution.size(); i++) {
            area += Area(solution[i]);
        }
#ifdef DEBUG_MODE
        //writeLog(INFO, "BUILD MAP - area of region: %f", area);
#endif
        searchSuccess = (area > 0);
    } else {
        searchSuccess = false;
    }
    
    return findValidVectorUtil(xStart + width - dx, yStart + height - dy, dx, dy);
}

FPoint3 BuildMap::findBestVector() const {
    //TODO it may be possible build map is disjoint, in which find best vector on both disjoint areas
    
    FPoint3 v = findValidVector();
    double heuristic = averageCuspHeight(v);
    
    return findBestVectorUtil(thetaToBAxisRange(thetaFromCartesian(v)), phiToAAxisRange(phiFromCartesian(v)), B_AXIS_DISCRETE_POINTS / 4, A_AXIS_DISCRETE_POINTS / 4, heuristic).first;
}

pair<FPoint3, double> BuildMap::findBestVectorUtil(int x, int y, int dx, int dy, double prevHeuristic) const {
    vector<pair<FPoint3, double>> options; //FPoint3 with lowest heuristic in this vector is the best vector
    
    double north = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi((y + dy) % A_AXIS_DISCRETE_POINTS)));
    double south = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi((y - dy) % A_AXIS_DISCRETE_POINTS)));
    double east = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta((x + dx) % B_AXIS_DISCRETE_POINTS), aAxisValToPhi(y)));
    double west = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta((x - dx) % B_AXIS_DISCRETE_POINTS), aAxisValToPhi(y)));
    
    int newDx = ceil(static_cast<double>(dx) / 2.0);
    int newDy = ceil(static_cast<double>(dy) / 2.0);
    
    if (prevHeuristic < fmin(north, fmin(south, fmin(east, west)))) { //there is no better option so continue narrowing down search
        if ((dx == 1) && (dy == 1)) {
            options.push_back(pair<FPoint3, double>(FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi(y)), prevHeuristic));
        } else {
            options.push_back(findBestVectorUtil(x, y, newDx, newDy, prevHeuristic));
        }
    } else {
        if (north < prevHeuristic) {
            options.push_back(findBestVectorUtil((x + newDx) % B_AXIS_DISCRETE_POINTS, y, newDx, newDy, north));
        }
        if (south < prevHeuristic) {
            options.push_back(findBestVectorUtil((x - newDx) % B_AXIS_DISCRETE_POINTS, y, newDx, newDy, south));
        }
        if (east < prevHeuristic) {
            options.push_back(findBestVectorUtil(x, (y + newDy) % A_AXIS_DISCRETE_POINTS, newDx, newDy, east));
        }
        if (west < prevHeuristic) {
            options.push_back(findBestVectorUtil(x, (y + newDy) % A_AXIS_DISCRETE_POINTS, newDx, newDy, west));
        }
    }
    
    pair<FPoint3, double> bestOption(FPoint3(0, 0, 0), INFINITY);
    for (vector<pair<FPoint3, double>>::iterator it = options.begin(); it != options.end(); it++) {
        if (it->second < bestOption.second) {
            bestOption = *it;
        }
    }
    
    if ((bestOption.first.x == 0) && (bestOption.first.y == 0) && (bestOption.first.z == 0)) {
        log("[ERROR] BUILD MAP - finding best vector returned no valid options");
    }
    
    return bestOption;
}

double BuildMap::averageCuspHeight(const FPoint3 & v) const {
    if (!checkVector(v)) {
        return INFINITY;
    }
    
    double weight = 0, totalFaceArea = 0;
    for (vector<MeshFace>::const_iterator it = m_mesh.faces.begin(); it != m_mesh.faces.end(); it++) {
        double weightToAdd = v * faceNormal(m_mesh, *it);
        weightToAdd /= v.vSize();
        
        //find area of face
        //take cross product of (v1 - v0) and (v2 - v0)
        FPoint3 normalUnnormalized = (m_mesh.vertices[it->vertex_index[1]].p - m_mesh.vertices[it->vertex_index[0]].p).cross(m_mesh.vertices[it->vertex_index[2]].p - m_mesh.vertices[it->vertex_index[0]].p);
        //area is equal to half the magnitude of a cross product
        double faceArea = normalUnnormalized.vSize() / 2;
        
        weight += fabs(weightToAdd) * faceArea;
        
        totalFaceArea += faceArea;
    }
    return weight /= totalFaceArea;
}

FPoint3 BuildMap::mapToVector(int x, int y) {
    return FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi(y));
}

std::pair<int, int> BuildMap::FPoint3ToMap(const FPoint3 & v) {
    return pair<int, int>(thetaToBAxisRange(thetaFromCartesian(v)), phiToAAxisRange(phiFromCartesian(v)));
}

int BuildMap::thetaToBAxisRange(double theta) {
    return radiansToDegrees(theta) / B_AXIS_PRECISION_DEGREES;
}

int BuildMap::phiToAAxisRange(double phi) {
    return radiansToDegrees(phi) / A_AXIS_PRECISION_DEGREES;
}

double BuildMap::bAxisValToTheta(double bAxisVal) {
    return degreesToRadians(bAxisVal * B_AXIS_PRECISION_DEGREES);
}

double BuildMap::aAxisValToPhi(double aAxisVal) {
    return degreesToRadians(aAxisVal * A_AXIS_PRECISION_DEGREES);
}
