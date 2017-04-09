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
        
        log("[INFO] BUILD MAP - delta-theta: %f\n", deltaTheta);
        log("[INFO] BUILD MAP - delta-phi: %f\n", deltaPhi);
        log("[INFO] BUILD MAP - ellipse area: %f\n", M_PI * deltaTheta * deltaPhi);
        log("[INFO] BUILD MAP - radius extension: %f\n", radiusExtension);
        
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
                log("[ERROR] BUILD MAP - error taking union of holes\n");
                //TODO return false;
            }
        } else {
            //if top point of build map is covered, set phiZeroAvailable to false
            m_phiZeroAvailable &= (fabs(phiFromCartesian(v)) > THETA_MAX);
            
            int xCenter = thetaToBAxisRange(thetaFromCartesian(v));
            int yCenter = phiToAAxisRange(phiFromCartesian(v));
            
            //            log("[INFO] BUILD MAP - removing hole centered at theta(%d) phi(%d)\n", xCenter, yCenter);
            
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
                
                wrapAroundThetaPos |= (x > B_AXIS_DISCRETE_POINTS);
                holeWrapThetaPos << ClipperLib::IntPoint(x - B_AXIS_DISCRETE_POINTS, y);
                
                wrapAroundThetaNeg |= (x < 0);
                holeWrapThetaNeg << ClipperLib::IntPoint(x + B_AXIS_DISCRETE_POINTS, y);
            }
            
            //            break;
            
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
                log("[ERROR] BUILD MAP - error taking union of holes\n");
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
        log("[ERROR] BUILD MAP - error taking difference of map and holes\n");
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
    for (int i = 0; i < m_buildMap2D.size(); i++) {
        int pointIn = PointInPolygon(ClipperLib::IntPoint(thetaToBAxisRange(thetaFromCartesian(v)), phiToAAxisRange(phiFromCartesian(v))), m_buildMap2D[i]);
        if (includeEdges ? (pointIn != 0) : (pointIn == 1)) {
            return true;
        }
    }
    return false;
}

vector<FPoint3> BuildMap::findValidVectors() const {
    if (area() == 0) {
        return vector<FPoint3>();
    }
    
    vector<FPoint3> valids;
    
    for (int i = 0; i < m_buildMap2D.size(); i++) {
        //        FPoint3 v = findValidVectorsUtil(0, 0, B_AXIS_DISCRETE_POINTS, A_AXIS_DISCRETE_POINTS);
        FPoint3 v = mapToFPoint3(m_buildMap2D[i][0].X, m_buildMap2D[i][0].Y);
        if (!checkVector(v)) {
            log("[ERROR] BUILD MAP - arbitrary vector(%f, %f, %f) not in buildmap\n", v.x, v.y, v.z);
        }
        valids.push_back(v);
    }
    return valids;
}

//std::vector<FPoint3> BuildMap::findValidVectorUtil(int xStart, int yStart, int width, int height) const {
//    //    log("[INFO] BUILD MAP - checking build map theta(%d-%d) phi(%d-%d)\n", xStart, xStart + width, yStart, yStart + height);
//
//    if ((width == 1) && (height == 1)) {
//        if (checkVector(FPoint3FromSpherical(BuildMap::bAxisValToTheta(xStart), BuildMap::aAxisValToPhi(yStart)))) {
//            return FPoint3FromSpherical(BuildMap::bAxisValToTheta(xStart), BuildMap::aAxisValToPhi(yStart));
//        } else if (checkVector(FPoint3FromSpherical(BuildMap::bAxisValToTheta(xStart + 1), BuildMap::aAxisValToPhi(yStart)))) {
//            return FPoint3FromSpherical(BuildMap::bAxisValToTheta(xStart + 1), BuildMap::aAxisValToPhi(yStart));
//        } else if (checkVector(FPoint3FromSpherical(BuildMap::bAxisValToTheta(xStart), BuildMap::aAxisValToPhi(yStart + 1)))) {
//            return FPoint3FromSpherical(BuildMap::bAxisValToTheta(xStart), BuildMap::aAxisValToPhi(yStart + 1));
//        } else if (checkVector(FPoint3FromSpherical(BuildMap::bAxisValToTheta(xStart + 1), BuildMap::aAxisValToPhi(yStart + 1)))) {
//            return FPoint3FromSpherical(BuildMap::bAxisValToTheta(xStart + 1), BuildMap::aAxisValToPhi(yStart + 1));
//        }
//
//        log("[ERROR] BUILD MAP - valid vector found is not in build map\n");
//        return FPoint3(0, 0, 0);
//    }
//
//    Clipper searchClipper;
//    searchClipper.AddPaths(m_buildMap2D, ptSubject, true);
//
//    bool cutHorizontally = width < height;
//    int dx = (cutHorizontally ? width : ceil(static_cast<double>(width) / 2.0));
//    int dy = (cutHorizontally ? ceil(static_cast<double>(height) / 2.0) : height);
//
//    Path search;
//    search << ClipperLib::IntPoint(xStart, yStart) << ClipperLib::IntPoint(xStart, yStart + dy) << ClipperLib::IntPoint(xStart + dx, yStart + dy) << ClipperLib::IntPoint(xStart + dx, yStart);
//    searchClipper.AddPath(search, ptClip, true);
//
//    Paths solution;
//    searchClipper.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
//
//    bool searchSuccess;
//    if (solution.size() != 0) {
//        double area = 0;
//        for (unsigned int i = 0; i < solution.size(); i++) {
//            area += Area(solution[i]);
//        }
//        //        log("[INFO] BUILD MAP - area of region theta(%d-%d) phi(%d-%d): %f\n", xStart, xStart + dx, yStart, yStart + dy, area);
//
//        searchSuccess = (area > 0);
//    } else {
//        //        log("[INFO] BUILD MAP - area of region theta(%d-%d) phi(%d-%d): %f\n", xStart, xStart + dx, yStart, yStart + dy, 0);
//
//        searchSuccess = false;
//    }
//
//    //    log("[INFO] search success: %s\n", searchSuccess ? "true" : "false");
//
//    if (searchSuccess) {
//        return findValidVectorUtil(xStart, yStart, dx, dy);
//    } else {
//        return findValidVectorUtil(xStart + width - dx, yStart + height - dy, dx, dy);
//    }
//}

FPoint3 BuildMap::findBestVector() const {
    //TODO it may be possible build map is disjoint, in which find best vector on both disjoint areas
    
    //    log("[INFO] number of disjoint areas in buildmap: %d\n", m_buildMap2D.size());
    
    //    vector<FPoint3> vs = findValidVectors();
    
    //    pair<FPoint3, double> heuristic = pair<FPoint3, double>(FPoint3(0, 0, 0), INFINITY);
    //    for (int i = 0; i < m_buildMap2D.size(); i++) {
    //        for (int j = 0; j < m_buildMap2D[i].size(); j++) {
    ////            log("[INFO] checking vector [%f, %f, %f]\n", vs[i].x, vs[i].y, vs[i].z);
    ////            FPoint3 v = mapToFPoint3(m_buildMap2D[i][j]);
    //
    //            pair<FPoint3, double> tempHeuristic = findBestVectorUtil(m_buildMap2D[i][j].X, m_buildMap2D[i][j].Y, averageCuspHeight(mapToFPoint3(m_buildMap2D[i][j].X, m_buildMap2D[i][j].Y)));
    //            if (tempHeuristic.second < heuristic.second) {
    //                heuristic = tempHeuristic;
    //            }
    //        }
    //    }
    //
    //    return heuristic.first;
    
    //    int precision = 1;
    //
    //    pair<FPoint3, double> heuristic = pair<FPoint3, double>(FPoint3(0, 0, 0), INFINITY);
    //    for (int y = 0; y <= A_AXIS_DISCRETE_POINTS; y += precision) {
    //        for (int x = 0; x <= B_AXIS_DISCRETE_POINTS; x += precision) {
    //            FPoint3 v = BuildMap::mapToFPoint3(x, y);
    //            double weight = averageCuspHeight(v);
    //
    //            if (weight < heuristic.second) {
    //                heuristic.first = v;
    //                heuristic.second = weight;
    //            }
    //        }
    //    }
    //
    //    return heuristic.first;
    
    //see http://katrinaeg.com/simulated-annealing.html
    pair<pair<int, int>, double> solution(pair<int, int>(m_buildMap2D[0][0].X, m_buildMap2D[0][0].Y), averageCuspHeight(mapToFPoint3(m_buildMap2D[0][0].X, m_buildMap2D[0][0].Y)));
    
    double temperature = 1.0;
    double minTemperature = 0.00001;
    double alpha = 0.99;
    unsigned int interations = 200;
    
    while (temperature > minTemperature) {
        for (int i = 0; i <= interations; i++) {
            int x = rand() % B_AXIS_DISCRETE_POINTS;
            int y = rand() % A_AXIS_DISCRETE_POINTS;
            
            pair<pair<int, int>, double> tempSolution(pair<int, int>(x, y), averageCuspHeight(mapToFPoint3(x, y)));
            
//            double tempHeuristic = averageCuspHeight(mapToFPoint3(tempSolution.first, tempSolution.second));
            
            double acceptanceProbability = exp((solution.second - tempSolution.second) / temperature);
            
            if (acceptanceProbability > (static_cast<double>(rand()) / static_cast<double>(RAND_MAX))) {
                solution = tempSolution;
            }
        }
        temperature *= alpha;
    }
    solution = hillClimb(solution.first.first, solution.first.second);
    return mapToFPoint3(solution.first.first, solution.first.second);
}

pair<pair<int, int>, double> BuildMap::hillClimb(int x, int y) const {
    double heuristic = averageCuspHeight(mapToFPoint3(x, y));
    
    while (true) {
//                printf("checking [%d, %d] %f\n", x, y, heuristic);
        
        if (heuristic == INFINITY) {
            return pair<pair<int, int>, double>(pair<int, int>(x, y), heuristic);
        }
        
        pair<int, int> n(x, (y + 1) % (A_AXIS_DISCRETE_POINTS + 1));
        pair<int, int> s(x, (y - 1) % (A_AXIS_DISCRETE_POINTS + 1));
        pair<int, int> e((x + 1) % (B_AXIS_DISCRETE_POINTS + 1), y);
        pair<int, int> w((x - 1) % (B_AXIS_DISCRETE_POINTS + 1), y);
        
        
        if (s.second < 0) {
            s.second = A_AXIS_DISCRETE_POINTS;
        }
        if (w.first < 0) {
            w.first = B_AXIS_DISCRETE_POINTS;
        }
        
        double nHeuristic = averageCuspHeight(mapToFPoint3(n.first, n.second));
        double sHeuristic = averageCuspHeight(mapToFPoint3(s.first, s.second));
        double eHeuristic = averageCuspHeight(mapToFPoint3(e.first, e.second));
        double wHeuristic = averageCuspHeight(mapToFPoint3(w.first, w.second));
        
//        printf(" checking n [%d, %d] %f\n", n.first, n.second, nHeuristic);
//        printf(" checking s [%d, %d] %f\n", s.first, s.second, sHeuristic);
//        printf(" checking e [%d, %d] %f\n", e.first, e.second, eHeuristic);
//        printf(" checking w [%d, %d] %f\n", w.first, w.second, wHeuristic);
        
        if (heuristic < fmin(nHeuristic, fmin(sHeuristic, fmin(eHeuristic, wHeuristic)))) {
            return pair<pair<int, int>, double>(pair<int, int>(x, y), heuristic);
        } else if (nHeuristic < fmin(sHeuristic, fmin(eHeuristic, wHeuristic))) {
            x = n.first;
            y = n.second;
            heuristic = nHeuristic;
        } else if (sHeuristic < fmin(eHeuristic, wHeuristic)) {
            x = s.first;
            y = s.second;
            heuristic = sHeuristic;
        } else if (eHeuristic < wHeuristic) {
            x = e.first;
            y = e.second;
            heuristic = eHeuristic;
        } else {
            x = w.first;
            y = w.second;
            heuristic = wHeuristic;
        }
    };
}

//pair<FPoint3, double> BuildMap::findBestVectorUtil(int x, int y, double prevHeuristic, int depth) const {
//    vector<pair<FPoint3, double>> options; //FPoint3 with lowest heuristic in this vector is the best vector
//    
//    std::string spaces = "";
//    for (int i = 0; i < depth % 100; i++) {
//        spaces += " ";
//    }
//    
//    log("depth: %d\n", depth);
//    
//    log("[INFO] %schecking position %d, %d with heuristic %f...\n", spaces.c_str(), x, y, prevHeuristic);
//    
//    double north = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi(y + 1)));
//    double south = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi(y - 1)));
//    double east = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta(x + 1), aAxisValToPhi(y)));
//    double west = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta(x - 1), aAxisValToPhi(y)));
//    
//    //    if (north < prevHeuristic) {
//    log("[INFO] %snorth: %d, %d with heuristic %f\n", spaces.c_str(), x, y + 1, north);
//    //    }
//    //    if (south < prevHeuristic) {
//    log("[INFO] %ssouth: %d, %d with heuristic %f\n", spaces.c_str(), x, y - 1, south);
//    //    }
//    //    if (east < prevHeuristic) {
//    log("[INFO] %seast: %d, %d with heuristic %f\n", spaces.c_str(), x + 1, y, east);
//    //    }
//    //    if (west < prevHeuristic) {
//    log("[INFO] %swest: %d, %d with heuristic %f\n", spaces.c_str(), x - 1, y, west);
//    //    }
//    
//    //    log("[INFO] %snorth: %d, %d with heurisitc %f (%s)\n", spaces.c_str(), x, y + 1, north, (north > prevHeuristic) ? "greater" : "less than");
//    //    log("[INFO] %ssouth: %d, %d with heurisitc %f (%s)\n", spaces.c_str(), x, y - 1, south, (south > prevHeuristic) ? "greater" : "less than");
//    //    log("[INFO] %seast: %d, %d with heurisitc %f (%s)\n", spaces.c_str(), x + 1, y, east, (east > prevHeuristic) ? "greater" : "less than");
//    //    log("[INFO] %swest: %d, %d with heurisitc %f (%s)\n", spaces.c_str(), x - 1, y, west, (west > prevHeuristic) ? "greater" : "less than");
//    
//    if (prevHeuristic < fmin(north, fmin(south, fmin(east, west)))) { //there is no better option so continue narrowing down search
//        return pair<FPoint3, double>(FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi(y)), prevHeuristic);
//    } else {
//        if (north < fmin(south, fmin(east, west))) {
//            options.push_back(findBestVectorUtil(x, y + 1, north, depth + 1));
//        } else if (south < fmin(east, west)) {
//            options.push_back(findBestVectorUtil(x, y - 1, south, depth + 1));
//        } else if (east < west) {
//            options.push_back(findBestVectorUtil(x + 1, y, east, depth + 1));
//        } else {
//            options.push_back(findBestVectorUtil(x - 1, y, west, depth + 1));
//        }
//        
//    }
//    
//    
//    //        double north = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi((y + dy) % A_AXIS_DISCRETE_POINTS)));
//    //        double south = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi((y - dy) % A_AXIS_DISCRETE_POINTS)));
//    //        double east = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta((x + dx) % B_AXIS_DISCRETE_POINTS), aAxisValToPhi(y)));
//    //        double west = averageCuspHeight(FPoint3FromSpherical(bAxisValToTheta((x - dx) % B_AXIS_DISCRETE_POINTS), aAxisValToPhi(y)));
//    //
//    //    log("[INFO] prev: [%d, %d] %f, n: [%d, %d] %f, s: [%d, %d] %f, e: [%d, %d] %f, w: [%d, %d] %f\n", x, y, prevHeuristic, x, (y + dy) % A_AXIS_DISCRETE_POINTS, north, x, (y - dy) % A_AXIS_DISCRETE_POINTS, south, (x + dx) % B_AXIS_DISCRETE_POINTS, y, east, (x - dx) % B_AXIS_DISCRETE_POINTS, y, west);
//    //
//    //    int newDx = ceil(static_cast<double>(dx) / 2.0);
//    //    int newDy = ceil(static_cast<double>(dy) / 2.0);
//    //
//    //    if (prevHeuristic > fmax(north, fmax(south, fmax(east, west)))) { //there is no better option so continue narrowing down search
//    //        if ((dx == 1) && (dy == 1)) {
//    //            options.push_back(pair<FPoint3, double>(FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi(y)), prevHeuristic));
//    //        } else {
//    //            options.push_back(findBestVectorUtil(x, y, newDx, newDy, prevHeuristic));
//    //        }
//    //    } else {
//    //        if (north > prevHeuristic) {
//    //            options.push_back(findBestVectorUtil((x + newDx) % B_AXIS_DISCRETE_POINTS, y, newDx, newDy, north));
//    //        }
//    //        if (south > prevHeuristic) {
//    //            options.push_back(findBestVectorUtil((x - newDx) % B_AXIS_DISCRETE_POINTS, y, newDx, newDy, south));
//    //        }
//    //        if (east > prevHeuristic) {
//    //            options.push_back(findBestVectorUtil(x, (y + newDy) % A_AXIS_DISCRETE_POINTS, newDx, newDy, east));
//    //        }
//    //        if (west > prevHeuristic) {
//    //            options.push_back(findBestVectorUtil(x, (y + newDy) % A_AXIS_DISCRETE_POINTS, newDx, newDy, west));
//    //        }
//    //    }
//    
//    pair<FPoint3, double> bestOption(FPoint3(0, 0, 0), INFINITY);
//    for (vector<pair<FPoint3, double>>::iterator it = options.begin(); it != options.end(); it++) {
//        //        log("[INFO] [%f, %f] heuristic: %f\n", 180/M_PI * thetaFromCartesian(it->first), 180/M_PI * phiFromCartesian(it->first), it->second);
//        if (it->second < bestOption.second) {
//            bestOption = *it;
//        }
//    }
//    
//    if ((bestOption.first.x == 0) && (bestOption.first.y == 0) && (bestOption.first.z == 0)) {
//        log("[ERROR] BUILD MAP - finding best vector returned no valid options\n");
//    }
//    
//    return bestOption;
//}

double BuildMap::averageCuspHeight(const FPoint3 & v) const {
    if (!checkVector(v)) {
        return INFINITY;
    }
    
    //    printf("checking vector[%f, %f, %f]\n", v.x, v.y, v.z);
    
    double weight = 0, totalFaceArea = 0;
    for (vector<MeshFace>::const_iterator it = m_mesh.faces.begin(); it != m_mesh.faces.end(); it++) {
        
        FPoint3 normal = faceNormal(m_mesh, *it);
        
        double cosTheta = (v * faceNormal(m_mesh, *it)) / (v.vSize() * normal.vSize());
        
        //        printf("cos(theta): %f\n", cosTheta);
        
        //        printf("checking face with normal[%f, %f, %f]\n", normal.x, normal.y, normal.z);
        
        double weightToAdd = v * normal;
        weightToAdd /= v.vSize() * normal.vSize();
        
        if ((cosTheta == -1) || (cosTheta == 1)) {
            weightToAdd = 0;
        }
        
        //find area of face
        //take cross product of (v1 - v0) and (v2 - v0)
        FPoint3 normalUnnormalized = FPoint3::cross((m_mesh.vertices[it->vertex_index[1]].p - m_mesh.vertices[it->vertex_index[0]].p), (m_mesh.vertices[it->vertex_index[2]].p - m_mesh.vertices[it->vertex_index[0]].p));
        //area is equal to half the magnitude of a cross product
        double faceArea = normalUnnormalized.vSize() / 2;
        
        weight += fabs(weightToAdd) * faceArea;
        
        //        printf("weight: %f, area: %f\n", weightToAdd, faceArea);
        
        totalFaceArea += faceArea;
        
        //        printf("total weight: %f, total area: %f\n", weight, totalFaceArea);
    }
    return weight / totalFaceArea;
}

FPoint3 BuildMap::mapToFPoint3(int x, int y) {
    return FPoint3FromSpherical(bAxisValToTheta(x), aAxisValToPhi(y));
}

std::pair<int, int> BuildMap::FPoint3ToMap(const FPoint3 & v) {
    return pair<int, int>(thetaToBAxisRange(thetaFromCartesian(v)), phiToAAxisRange(phiFromCartesian(v)));
}

int BuildMap::thetaToBAxisRange(double theta) {
    double ret = radiansToDegrees(theta) / B_AXIS_PRECISION_DEGREES;
    while (ret < 0) {
        ret += B_AXIS_DISCRETE_POINTS;
    }
    return ret;
}

int BuildMap::phiToAAxisRange(double phi) {
    double ret = radiansToDegrees(phi) / A_AXIS_PRECISION_DEGREES;
    while (ret < 0) {
        ret += A_AXIS_DISCRETE_POINTS;
    }
    return ret;
}

double BuildMap::bAxisValToTheta(double bAxisVal) {
    return degreesToRadians(bAxisVal * B_AXIS_PRECISION_DEGREES);
}

double BuildMap::aAxisValToPhi(double aAxisVal) {
    return degreesToRadians(aAxisVal * A_AXIS_PRECISION_DEGREES);
}
