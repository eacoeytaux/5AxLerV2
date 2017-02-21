#include "VolumeDecomposer.hpp"
#include "../utils/polygon.h"
#include "../utils/intpoint.h"

#include "Utility.hpp"

#include <algorithm>

using namespace cura;
using namespace std;

VolumeDecomposer::VolumeDecomposer(Mesh& mesh, Slicer* slicer) {
    vector<SlicerLayer> & layers = slicer->layers;
    
    // Initialize all of our comparison vars
    SlicerLayer & comparisonSlice = layers[0];
    Polygons & comparisonPolys = comparisonSlice.polygons;
    
    for (unsigned int layer_idx = 1; layer_idx < layers.size(); ++layer_idx) {
        SlicerLayer & slice = layers[layer_idx];
        Polygons & polys = slice.polygons;
        Polygons & openPolys = slice.openPolylines;
        vector<vector<int>> polyFaces = slice.polyFaces;
        
        log("[CUSTOM] %d) polygons size: %d, open polys size: %d, poly faces size: %d, height: %d\n", layer_idx, polys.size(), openPolys.size(), polyFaces.size(), slice.z);
        
        // For printing the polygons
        // for (unsigned int poly_idx = 0; poly_idx < polys.size(); ++poly_idx) {
        //     const PolygonRef polyRef = polys[poly_idx];
        
        //     log("[CUSTOM] Polygon ID: %d\n", poly_idx);
        //     for (unsigned int point_idx = 0; point_idx < polyRef.size(); ++point_idx) {
        //         Point p = polyRef[point_idx];
        
        //         log("\t(%d) %d, %d\n", point_idx, p.X, p.Y);
        //     }
        // }
        
        // For printing the faces
        for (unsigned int polyfaces_idx = 0; polyfaces_idx < polyFaces.size(); ++polyfaces_idx) {
            vector<int> faces = polyFaces[polyfaces_idx];
            log("[CUSTOM] Polygon ID: %d\n", polyfaces_idx);
            for (unsigned int face_idx = 0; face_idx < faces.size(); ++face_idx) {
                int faceID = faces[face_idx];
                string faceString = VolumeDecomposer::faceToString(mesh, faceID);
                pair<bool, int> intersectOverhangResult = faceIsOverhangIntersect(mesh, faceID, comparisonPolys);
                bool intersectingOverhang = intersectOverhangResult.first;
                int polygonID = intersectOverhangResult.second;
                
                log("\t%s (intersects: %s)\n", faceString.c_str(), intersectingOverhang ? "true" : "false");
                
                if (intersectingOverhang) {
                    // FPoint3 intersection = findPolyFaceIntersection(mesh, faceID, comparisonPolys[polygonID], sliceHeight);
                    pair<Point3, Point3> splitPoints;
                    int numSplitPoints = findSplitPoints(mesh, faceID, comparisonPolys[polygonID], splitPoints);
                    splitFaces(mesh, comparisonPolys[polygonID], faceID, numSplitPoints, splitPoints);
                }
            }
        }
        
        comparisonPolys = polys;
    }
}

void VolumeDecomposer::splitFaces(Mesh& mesh, PolygonRef intersectingPolyRef, int faceID, int numSplitPoints, pair<Point3, Point3>& splitPoints) {
    
    Point3 prevSplitPoint;
    int prevSplitPointIndex;
    int prevSplitPointPrimeIndex;
    bool pointOnPrevFirstEdge = true;
    tuple<int, int, int> prevFaceIDs;
    int prevFaceCase = 0;
    
    tuple<int, int, int> originalFaceIDs;
    
    bool firstCycle = true;
    int firstFaceCase = 0;
    
    /*
     case I: neither split point is on a vertex
     case II: exactly one split points is on a vertex
     case III: both split point are on the same vertex
     case IV: each split point is on a different vertex
     */
    
    while (true) {
        if (numSplitPoints == 0) {
            log("[ERROR] adjacent face of split face has no split points");
            return;
        }
        
        // Grab the actual MeshFace
        MeshFace& face = mesh.faces[faceID];
        
        //save original vertex indices for future reference
        int originalVertexIndices[3] = {-1};
        originalVertexIndices[0] = face.vertex_index[0];
        originalVertexIndices[1] = face.vertex_index[1];
        originalVertexIndices[2] = face.vertex_index[2];
        
        //determine if the edge intersections any vertices
        pair<int, int> splitPointVertexIntersectionIndices(-1, -1);
        
        if (splitPoints.first == mesh.vertices[face.vertex_index[0]].p) {
            splitPointVertexIntersectionIndices.first = 0;
        } else if (splitPoints.first == mesh.vertices[face.vertex_index[1]].p) {
            splitPointVertexIntersectionIndices.first = 1;
        } else if (splitPoints.first == mesh.vertices[face.vertex_index[2]].p) {
            splitPointVertexIntersectionIndices.first = 2;
        }
        
        if (splitPoints.second == mesh.vertices[face.vertex_index[0]].p) {
            splitPointVertexIntersectionIndices.second = 0;
        } else if (splitPoints.second == mesh.vertices[face.vertex_index[1]].p) {
            splitPointVertexIntersectionIndices.second = 1;
        } else if (splitPoints.second == mesh.vertices[face.vertex_index[2]].p) {
            splitPointVertexIntersectionIndices.second = 2;
        }
        
        if ((splitPointVertexIntersectionIndices.first >= 0) && (splitPointVertexIntersectionIndices.second >= 0)) { //case III or case case IV
            
            if (splitPointVertexIntersectionIndices.first == splitPointVertexIntersectionIndices.second) { //case III
                
                int x, y, z;
                if (splitPointVertexIntersectionIndices.first == 0) {
                    x = 0;
                    y = 1;
                    z = 2;
                } else if (splitPointVertexIntersectionIndices.first == 1) {
                    x = 1;
                    y = 2;
                    z = 0;
                } else if (splitPointVertexIntersectionIndices.first == 2) {
                    x = 2;
                    y = 0;
                    z = 1;
                } else {
                    log("[ERROR] splitFaceRecursiveB() has matching split points not on a vertex");
                    return;
                }
                
                vector<uint32_t>::iterator it; //index of faceID in vector
                
                //remove original face from xth vertex
                it = find(mesh.vertices[originalVertexIndices[x]].connected_faces.begin(), mesh.vertices[originalVertexIndices[x]].connected_faces.end(), faceID);
                if (it != mesh.vertices[originalVertexIndices[x]].connected_faces.end()) {
                    mesh.vertices[originalVertexIndices[x]].connected_faces.erase(it);
                }
                
                if (firstCycle) {
                    firstCycle = false;
                    firstFaceCase = 3;
                    
                    prevSplitPointIndex = face.vertex_index[x];
                    
                    mesh.vertices.push_back(MeshVertex(mesh.vertices[face.vertex_index[x]].p));
                    prevSplitPointPrimeIndex = mesh.vertices.size() - 1;
                    
                    originalFaceIDs = tuple<int, int, int>(faceID, -1, -1);
                }
                
                //add face to overhang vertex
                mesh.vertices[prevSplitPointPrimeIndex].connected_faces.push_back(faceID);
                
                //reset variables
                prevFaceCase = 3;
                prevFaceIDs = tuple<int, int, int>(faceID, -1, -1);
                if ((face.connected_face_index[x] == get<0>(prevFaceIDs)) || (face.connected_face_index[x] == get<1>(prevFaceIDs))) {
                    faceID = face.connected_face_index[z];
                } else if ((face.connected_face_index[z] == get<0>(prevFaceIDs)) || (face.connected_face_index[z] == get<0>(prevFaceIDs))) {
                    faceID = face.connected_face_index[x];
                }
                pointOnPrevFirstEdge = true;
                
                if (faceID == get<0>(originalFaceIDs)) {
                    if ((firstFaceCase == 3) || (firstFaceCase == 4)) {
                        return; //nothing more needs to be done here
                    } else {
                        log("[ERROR] splitFaces() ended on face that is not a case III/IV from a case III face");
                        return;
                    }
                } else if (faceID == get<1>(originalFaceIDs)) {
                    if (firstFaceCase == 2) {
                        return; //nothing more needs to be done here
                    } else {
                        log("[ERROR] splitFaces() ended on face that is not a case II from a case III face");
                        return;
                    }
                }
                
                numSplitPoints = findSplitPoints(mesh, faceID, intersectingPolyRef, splitPoints);
                
            } else { //case IV
                
                bool switched = false;
                if ((splitPointVertexIntersectionIndices.first == 1) && (splitPointVertexIntersectionIndices.second == 0)) {
                    splitPointVertexIntersectionIndices = pair<int, int>(0, 1);
                    switched = true;
                } else if ((splitPointVertexIntersectionIndices.first == 2) && (splitPointVertexIntersectionIndices.first == 1)) {
                    splitPointVertexIntersectionIndices = pair<int, int>(1, 2);
                    switched = true;
                } else if ((splitPointVertexIntersectionIndices.second == 0) && (splitPointVertexIntersectionIndices.first == 2)) {
                    splitPointVertexIntersectionIndices = pair<int, int>(2, 0);
                    switched = true;
                }
                
                int x, y, z;
                if ((splitPointVertexIntersectionIndices.first == 0) && (splitPointVertexIntersectionIndices.second == 1)) {
                    x = 0;
                    y = 1;
                    z = 2;
                } else if ((splitPointVertexIntersectionIndices.first == 1) && (splitPointVertexIntersectionIndices.second == 2)) {
                    x = 1;
                    y = 2;
                    z = 0;
                } else if ((splitPointVertexIntersectionIndices.first == 2) && (splitPointVertexIntersectionIndices.second == 0)) {
                    x = 2;
                    y = 0;
                    z = 1;
                } else {
                    log("[ERROR] split points are on impossible order of edges");
                    return;
                }
                
                //whether or not the first split point is the split point from the previous face
                bool firstPointMatch = true;
                
                pair<int, int> newVertexIndices, newVertexPrimeIndices; //indices of vertices in mesh (prime are the vertices on the overhang)
                
                if (firstCycle) {
                    firstCycle = true;
                    firstFaceCase = 4;
                    
                    firstPointMatch = true;
                    
                    mesh.vertices.push_back(MeshVertex(mesh.vertices[face.vertex_index[x]].p));
                    mesh.vertices.push_back(MeshVertex(mesh.vertices[face.vertex_index[y]].p));
                    newVertexPrimeIndices = pair<int, int>(mesh.vertices.size() - 2, mesh.vertices.size() - 1);
                    
                    prevSplitPointIndex = face.vertex_index[x];
                    prevSplitPointPrimeIndex = mesh.vertices.size() - 1;
                    
                    originalFaceIDs = tuple<int, int, int>(faceID, -1, -1);
                } else {
                    if (splitPoints.first == prevSplitPoint) {
                        firstPointMatch = true;
                        
                        mesh.vertices.push_back(MeshVertex(splitPoints.second));
                        newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.vertices.size() - 1); //indexes of new vertices in mesh
                    } else if (splitPoints.second == prevSplitPoint) {
                        firstPointMatch = false;
                        
                        mesh.vertices.push_back(MeshVertex(splitPoints.first));
                        newVertexPrimeIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointPrimeIndex); //indexes of new vertices in mesh
                    } else {
                        log("[ERROR] adjacent face to split face does not have a matching split point");
                        return;
                    }
                }
                
                face.vertex_index[x] = newVertexPrimeIndices.first;
                face.vertex_index[y] = newVertexPrimeIndices.second;
                
                vector<uint32_t>::iterator it; //index of faceID in vector
                
                //remove original face from xth vertex
                it = find(mesh.vertices[originalVertexIndices[x]].connected_faces.begin(), mesh.vertices[originalVertexIndices[x]].connected_faces.end(), faceID);
                if (it != mesh.vertices[originalVertexIndices[x]].connected_faces.end()) {
                    mesh.vertices[originalVertexIndices[x]].connected_faces.erase(it);
                }
                //add face to new xth vertex
                mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(faceID);
                
                //remove original face from yth vertex
                it = find(mesh.vertices[originalVertexIndices[y]].connected_faces.begin(), mesh.vertices[originalVertexIndices[y]].connected_faces.end(), faceID);
                if (it != mesh.vertices[originalVertexIndices[y]].connected_faces.end()) {
                    mesh.vertices[originalVertexIndices[y]].connected_faces.erase(it);
                }
                //add face to new yth vertex
                mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(faceID);
                
                MeshFace& adjacentFace = mesh.faces[face.connected_face_index[x]]; //adjacent face on other side of split
                //remove face from adjacent face list of adjacent face on other side of split
                if (adjacentFace.connected_face_index[0] == faceID) {
                    adjacentFace.connected_face_index[0] = -1;
                } else if (adjacentFace.connected_face_index[1] == faceID) {
                    adjacentFace.connected_face_index[1] = -1;
                } else if (adjacentFace.connected_face_index[2] == faceID) {
                    adjacentFace.connected_face_index[2] = -1;
                } else {
                    log("[ERROR] adjacent face does not list current face as an adjacent face");
                    return;
                }
                
                //remove adjacent face on other side of split from face
                face.connected_face_index[x] = -1;
                
                //reset variables
                prevFaceCase = 4;
                prevFaceIDs = tuple<int, int, int>(faceID, -1, -1);
                if (firstPointMatch) {
                    faceID = face.connected_face_index[y];
                    prevSplitPointIndex = originalVertexIndices[y];
                    prevSplitPointPrimeIndex = newVertexPrimeIndices.second;
                } else {
                    faceID = face.connected_face_index[z];
                    prevSplitPointIndex = originalVertexIndices[x];
                    prevSplitPointPrimeIndex = newVertexPrimeIndices.first;
                }
                pointOnPrevFirstEdge = true;
                
                if (faceID == get<0>(originalFaceIDs)) {
                    if ((firstFaceCase == 3) || (firstFaceCase == 4)) {
                        return; //nothing more needs to be done here
                    } else {
                        log("[ERROR] splitFaces() ended on face that is not a case III/IV from a case IV face");
                        return;
                    }
                } else if (faceID == get<1>(originalFaceIDs)) {
                    if (firstFaceCase == 2) {
                        return; //nothing more needs to be done here
                    } else {
                        log("[ERROR] splitFaces() ended on face that is not a case II from a case III face");
                        return;
                    }
                }
                
                numSplitPoints = findSplitPoints(mesh, faceID, intersectingPolyRef, splitPoints);
                
            }
            
        } else if ((splitPointVertexIntersectionIndices.first >= 0) || (splitPointVertexIntersectionIndices.second >= 0)) { //case II
            
            //the xth vertex is always the vertex being intersected by the split point
            //the first face ID is whichever face is NOT on the overhang
            //the second face ID is whichever face is on the overhang
            
            bool switched = false;
            if (splitPointVertexIntersectionIndices.second >= 0) {
                switched = true;
                
                int tempInt = splitPointVertexIntersectionIndices.first;
                splitPointVertexIntersectionIndices.first = splitPointVertexIntersectionIndices.second;
                splitPointVertexIntersectionIndices.second = tempInt;
                
                Point3 tempPoint = splitPoints.first;
                splitPoints.first = splitPoints.second;
                splitPoints.second = tempPoint;
            }
            
            int x, y, z;
            if (splitPointVertexIntersectionIndices.first == 0) {
                x = 0;
                y = 1;
                z = 2;
            } else if (splitPointVertexIntersectionIndices.first == 1) {
                x = 1;
                y = 2;
                z = 0;
            } else if (splitPointVertexIntersectionIndices.first == 2) {
                x = 2;
                y = 0;
                z = 1;
            } else {
                log("[ERROR] split points vertex intersection index is not 0, 1, or 2");
                return;
            }
            
            int newVertexIndex = -1;
            pair<int, int> newVertexPrimeIndices(-1, -1);
            
            if (firstCycle) {
                
            } else {
                newVertexIndex = prevSplitPointIndex;
                
                mesh.vertices.push_back(MeshVertex(splitPoints.second));
                newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.vertices.size() - 1);
            }
            
            mesh.faces.push_back(MeshFace());
            int newFaceID = mesh.faces.size() - 1;
            MeshFace& newFace = mesh.faces[newFaceID];
            
            if (intersectingPolyRef.inside(Point(mesh.vertices[face.vertex_index[y]].p.x, mesh.vertices[face.vertex_index[y]].p.y))) {
                newFace.vertex_index[0] = newVertexPrimeIndices.first;
                newFace.vertex_index[1] = newVertexPrimeIndices.second;
                newFace.vertex_index[2] = face.vertex_index[z];
                newFace.connected_face_index[2] = face.connected_face_index[2];
                if (mesh.faces[newFace.connected_face_index[2]].connected_face_index[0] == faceID) {
                    mesh.faces[newFace.connected_face_index[2]].connected_face_index[0] = newFaceID;
                } else if (mesh.faces[newFace.connected_face_index[2]].connected_face_index[1] == faceID) {
                    mesh.faces[newFace.connected_face_index[2]].connected_face_index[1] = newFaceID;
                } else if (mesh.faces[newFace.connected_face_index[2]].connected_face_index[2] == faceID) {
                    mesh.faces[newFace.connected_face_index[2]].connected_face_index[2] = newFaceID;
                }
                
                face.vertex_index[z] = newVertexIndex;
                face.connected_face_index[z] = -1;
                
                if (prevFaceCase == 1) {
                    if (pointOnPrevFirstEdge) {
                        //connect A and prevA
                        face.connected_face_index[y] = get<0>(prevFaceIDs);
                        
                        //conect B and prevB
                        int prevFaceIDsSecond = get<1>(prevFaceIDs);
                        newFace.connected_face_index[1] = prevFaceIDsSecond;
                        if (mesh.faces[prevFaceIDsSecond].connected_face_index[0] == faceID) {
                            mesh.faces[prevFaceIDsSecond].connected_face_index[0] = newFaceID;
                        } else if (mesh.faces[prevFaceIDsSecond].connected_face_index[1] == faceID) {
                            mesh.faces[prevFaceIDsSecond].connected_face_index[1] = newFaceID;
                        } else if (mesh.faces[prevFaceIDsSecond].connected_face_index[2] == faceID) {
                            mesh.faces[prevFaceIDsSecond].connected_face_index[2] = newFaceID;
                        }
                        
                        //call recursively on zth edge of newFace
                    } else {
                        //connect A and prevC
                    }
                }
            } else {
                newFace.vertex_index[0] = newVertexPrimeIndices.first;
                newFace.vertex_index[1] = face.vertex_index[y];
                newFace.vertex_index[2] = newVertexPrimeIndices.second;
                newFace.connected_face_index[0] = face.connected_face_index[0];
                if (mesh.faces[newFace.connected_face_index[0]].connected_face_index[0] == faceID) {
                    mesh.faces[newFace.connected_face_index[0]].connected_face_index[0] = newFaceID;
                } else if (mesh.faces[newFace.connected_face_index[0]].connected_face_index[1] == faceID) {
                    mesh.faces[newFace.connected_face_index[0]].connected_face_index[1] = newFaceID;
                } else if (mesh.faces[newFace.connected_face_index[0]].connected_face_index[2] == faceID) {
                    mesh.faces[newFace.connected_face_index[0]].connected_face_index[2] = newFaceID;
                }
                
                face.vertex_index[y] = newVertexIndex;
                face.connected_face_index[x] = -1;
                
                if (pointOnPrevFirstEdge) {
                    //conect A and prevB
                    face.connected_face_index[y] = get<1>(prevFaceIDs);
                    
                    //connect B and prevA
                    int prevFaceIDsFirst = get<0>(prevFaceIDs);
                    newFace.connected_face_index[1] = prevFaceIDsFirst;
                    if (mesh.faces[prevFaceIDsFirst].connected_face_index[0] == faceID) {
                        mesh.faces[prevFaceIDsFirst].connected_face_index[0] = newFaceID;
                    } else if (mesh.faces[prevFaceIDsFirst].connected_face_index[1] == faceID) {
                        mesh.faces[prevFaceIDsFirst].connected_face_index[1] = newFaceID;
                    } else if (mesh.faces[prevFaceIDsFirst].connected_face_index[2] == faceID) {
                        mesh.faces[prevFaceIDsFirst].connected_face_index[2] = newFaceID;
                    }
                    
                    //call recursively on xth edge of newFace
                    prevFaceIDs = tuple<int, int, int>(faceID, newFaceID, -1);
                }
                
            }
            
        } else { //case I
            
            //the face is always split so the side that is a triangle is left as the original face in the face array
            //the trapezoid side is always split (from the perspective of the larger base being on the bottom) from top-left to bottom-right
            //triangle side is always first ID, top triangle of trapezoid is second, and bottom triangle of trapezoid is third (regardless of which side is the overhang)
            
            
            //determine which edges split points are on
            pair<int, int> splitPointEdgeIndices(-1, -1);
            
            FPoint3 edges[3];
            edges[0] = mesh.vertices[face.vertex_index[1]].p - mesh.vertices[face.vertex_index[0]].p;
            edges[1] = mesh.vertices[face.vertex_index[2]].p - mesh.vertices[face.vertex_index[1]].p;
            edges[2] = mesh.vertices[face.vertex_index[0]].p - mesh.vertices[face.vertex_index[2]].p;
            
            if (edges[0].cross(splitPoints.first - mesh.vertices[face.vertex_index[0]].p).vSize2() == 0) {
                splitPointEdgeIndices.first = 0;
            } else if (edges[1].cross(splitPoints.first - mesh.vertices[face.vertex_index[1]].p).vSize2() == 0) {
                splitPointEdgeIndices.first = 1;
            } else if (edges[2].cross(splitPoints.first - mesh.vertices[face.vertex_index[2]].p).vSize2() == 0) {
                splitPointEdgeIndices.first = 2;
            } else {
                log("[ERROR] split point was not found to be on any edge of face");
                return;
            }
            
            if (splitPointEdgeIndices.first == splitPointEdgeIndices.second) {
                log("[ERROR] split points are on same edge");
                return;
            }
            
            //if order is either 1-0, 2-0, or 2-1 switch it to 0-1, 0-2, or 1-2 repsectively so there's only 3 scenarios
            bool switched = false;
            if ((splitPointEdgeIndices.first == 1) && (splitPointEdgeIndices.second == 0)) {
                splitPointEdgeIndices = pair<int, int>(0, 1);
                switched = true;
            } else if ((splitPointEdgeIndices.first == 2) && (splitPointEdgeIndices.first == 1)) {
                splitPointEdgeIndices = pair<int, int>(1, 2);
                switched = true;
            } else if ((splitPointEdgeIndices.second == 0) && (splitPointEdgeIndices.first == 2)) {
                splitPointEdgeIndices = pair<int, int>(2, 0);
                switched = true;
            }
            
            if (switched) {
                Point3 temp = splitPoints.first;
                splitPoints.first = splitPoints.second;
                splitPoints.second = temp;
            }
            
            //at this point, the edges that the split points are on are determined
            
            int x, y, z;
            if ((splitPointEdgeIndices.first == 0) && (splitPointEdgeIndices.second == 1)) {
                x = 0;
                y = 1;
                z = 2;
            } else if ((splitPointEdgeIndices.first == 1) && (splitPointEdgeIndices.second == 2)) {
                x = 1;
                y = 2;
                z = 0;
            } else if ((splitPointEdgeIndices.first == 2) && (splitPointEdgeIndices.second == 0)) {
                x = 2;
                y = 0;
                z = 1;
            } else {
                log("[ERROR] split points are on impossible order of edges");
                return;
            }
            
            //if this is the last triangle before return to original face where cut was started
            bool isFinalFaceInRecurse = (((face.connected_face_index[x] == get<0>(originalFaceIDs)) || (face.connected_face_index[y] == get<0>(originalFaceIDs))) && (get<0>(prevFaceIDs) != get<0>(originalFaceIDs))); //oh shit it's OG!!
            
            //whether or not the trapezoid created by the cut is in the overhang
            bool trapezoidInOverhang = intersectingPolyRef.inside(face.vertex_index[y]); //y vertex is connected to base volume
            
            //whether or not the first split point is the split point from the previous face
            bool firstPointMatch = true;
            
            pair<int, int> newVertexIndices, newVertexPrimeIndices; //indices of vertices in mesh (prime are the vertices on the overhang)
            
            if (firstCycle) {
                mesh.vertices.push_back(MeshVertex(splitPoints.first));
                mesh.vertices.push_back(MeshVertex(splitPoints.second));
                pair<int, int> newVertexIndices(mesh.vertices.size() - 2, mesh.vertices.size() - 1); //indexes of new vertices in mesh
                
                mesh.vertices.push_back(MeshVertex(splitPoints.first));
                mesh.vertices.push_back(MeshVertex(splitPoints.second));
                pair<int, int> newVertexPrimeIndices(mesh.vertices.size() - 2, mesh.vertices.size() - 1); //indexes of new vertices in mesh
            } else if (isFinalFaceInRecurse) {
                if (splitPoints.first == prevSplitPoint) {
                    firstPointMatch = true;
                    
                    if (trapezoidInOverhang) {
                        newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.faces[get<0>(originalFaceIDs)].vertex_index[2] - 2); //indexes of new vertices in mesh
                        newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.faces[get<0>(originalFaceIDs)].vertex_index[2]); //indexes of new vertices in mesh
                    } else {
                        newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.faces[get<0>(originalFaceIDs)].vertex_index[2]); //indexes of new vertices in mesh
                        newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.faces[get<0>(originalFaceIDs)].vertex_index[2] + 2); //indexes of new vertices in mesh
                    }
                } else if (splitPoints.second == prevSplitPoint) {
                    firstPointMatch = false;
                    
                    if (trapezoidInOverhang) {
                        newVertexIndices = pair<int, int>(mesh.faces[get<0>(originalFaceIDs)].vertex_index[2], prevSplitPointIndex); //indexes of new vertices in mesh
                        newVertexPrimeIndices = pair<int, int>(mesh.faces[get<0>(originalFaceIDs)].vertex_index[2] + 2, prevSplitPointPrimeIndex); //indexes of new vertices in mesh
                    } else {
                        newVertexIndices = pair<int, int>(mesh.faces[get<0>(originalFaceIDs)].vertex_index[2] - 2, prevSplitPointIndex); //indexes of new vertices in mesh
                        newVertexPrimeIndices = pair<int, int>(mesh.faces[get<0>(originalFaceIDs)].vertex_index[2], prevSplitPointPrimeIndex); //indexes of new vertices in mesh
                    }
                } else {
                    log("[ERROR] adjacent face to split face does not have a matching split point");
                    return;
                }
            } else {
                if (splitPoints.first == prevSplitPoint) {
                    firstPointMatch = true;
                    
                    //add split points to vertex list
                    mesh.vertices.push_back(MeshVertex(splitPoints.second));
                    newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.vertices.size() - 1); //indexes of new vertices in mesh
                    
                    mesh.vertices.push_back(MeshVertex(splitPoints.second));
                    newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.vertices.size() - 1); //indexes of new vertices in mesh
                } else if (splitPoints.second == prevSplitPoint) {
                    firstPointMatch = false;
                    
                    mesh.vertices.push_back(MeshVertex(splitPoints.first));
                    newVertexIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointIndex); //indexes of new vertices in mesh
                    
                    mesh.vertices.push_back(MeshVertex(splitPoints.first));
                    newVertexPrimeIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointPrimeIndex); //indexes of new vertices in mesh
                } else {
                    log("[ERROR] adjacent face to split face does not have a matching split point");
                    return;
                }
            }
            
            //add new faces
            mesh.faces.push_back(MeshFace());
            mesh.faces.push_back(MeshFace());
            pair<int, int> newFaceIDs(mesh.faces.size() - 2, mesh.faces.size() - 1);
            pair<MeshFace&, MeshFace&> newFaces(mesh.faces[newFaceIDs.first], mesh.faces[newFaceIDs.second]);
            
            if (trapezoidInOverhang) {
                //alter original face
                face.vertex_index[x] = newVertexIndices.first;
                face.vertex_index[z] = newVertexIndices.second;
                
                //add first face
                newFaces.first.vertex_index[0] = originalVertexIndices[x];
                newFaces.first.vertex_index[1] = newVertexPrimeIndices.first;
                newFaces.first.vertex_index[2] = newVertexPrimeIndices.second;
                
                //add second face
                newFaces.second.vertex_index[0] = originalVertexIndices[z];
                newFaces.second.vertex_index[1] = originalVertexIndices[x];
                newFaces.second.vertex_index[2] = newVertexPrimeIndices.second;
            } else {
                //alter original face
                face.vertex_index[x] = newVertexPrimeIndices.first;
                face.vertex_index[z] = newVertexPrimeIndices.second;
                
                //add first face
                newFaces.first.vertex_index[0] = originalVertexIndices[x];
                newFaces.first.vertex_index[1] = newVertexIndices.first;
                newFaces.first.vertex_index[2] = newVertexIndices.second;
                
                //add second face
                newFaces.second.vertex_index[0] = originalVertexIndices[z];
                newFaces.second.vertex_index[1] = originalVertexIndices[x];
                newFaces.second.vertex_index[2] = newVertexIndices.second;
            }
            
            //add connected faces to new vertices
            if (trapezoidInOverhang) {
                mesh.vertices[newVertexIndices.first].connected_faces.push_back(faceID);
                mesh.vertices[newVertexIndices.second].connected_faces.push_back(faceID);
                mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(newFaceIDs.first);
                mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(newFaceIDs.first);
                mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(newFaceIDs.second);
            } else {
                mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(faceID);
                mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(faceID);
                mesh.vertices[newVertexIndices.first].connected_faces.push_back(newFaceIDs.first);
                mesh.vertices[newVertexIndices.second].connected_faces.push_back(newFaceIDs.first);
                mesh.vertices[newVertexIndices.second].connected_faces.push_back(newFaceIDs.second);
            }
            
            //--fix adjacent faces--
            
            //remove connected faces from existing vertices
            
            vector<uint32_t>::iterator it; //index of faceID in vector
            
            //remove original face from xth vertex
            it = find(mesh.vertices[originalVertexIndices[x]].connected_faces.begin(), mesh.vertices[originalVertexIndices[x]].connected_faces.end(), faceID);
            if (it != mesh.vertices[originalVertexIndices[x]].connected_faces.end()) {
                mesh.vertices[originalVertexIndices[x]].connected_faces.erase(it);
            }
            //add first and second added face to xth vertex
            mesh.vertices[originalVertexIndices[x]].connected_faces.push_back(newFaceIDs.first);
            mesh.vertices[originalVertexIndices[x]].connected_faces.push_back(newFaceIDs.second);
            
            //remove original face from zth vertex
            it = find(mesh.vertices[originalVertexIndices[z]].connected_faces.begin(), mesh.vertices[originalVertexIndices[z]].connected_faces.end(), faceID);
            if (it != mesh.vertices[originalVertexIndices[z]].connected_faces.end()) {
                mesh.vertices[originalVertexIndices[z]].connected_faces.erase(it);
            }
            //add just second added face to xth vertex
            mesh.vertices[originalVertexIndices[z]].connected_faces.push_back(newFaceIDs.second);
            
            //updated index of adjacent face
            if (mesh.faces[face.connected_face_index[z]].connected_face_index[0] == faceID) {
                mesh.faces[face.connected_face_index[z]].connected_face_index[0] = newFaceIDs.second;
            } else if (mesh.faces[face.connected_face_index[z]].connected_face_index[1] == faceID) {
                mesh.faces[face.connected_face_index[z]].connected_face_index[1] = newFaceIDs.second;
            } else if (mesh.faces[face.connected_face_index[z]].connected_face_index[2] == faceID) {
                mesh.faces[face.connected_face_index[z]].connected_face_index[2] = newFaceIDs.second;
            } else {
                log("[ERROR] original face is not listed as an adjacent face of adjacent face");
                return;
            }
            newFaces.first.connected_face_index[2] = newFaceIDs.second;
            newFaces.second.connected_face_index[1] = newFaceIDs.first;
            newFaces.second.connected_face_index[0] = face.connected_face_index[z];
            face.connected_face_index[z] = -1;
            
            if (firstCycle) {
                //reset variables
                firstCycle = false;
                firstFaceCase = 1;
                
                prevFaceCase = 1;
                originalFaceIDs = prevFaceIDs = tuple<int, int, int>(faceID, newFaceIDs.first, newFaceIDs.second);
                prevSplitPointIndex = newVertexIndices.first;
                prevSplitPointPrimeIndex = newVertexPrimeIndices.first;
                
                pointOnPrevFirstEdge = true;
                faceID = face.connected_face_index[x];
                numSplitPoints = findSplitPoints(mesh, faceID, intersectingPolyRef, splitPoints);
                
            } else {
                
                if (firstPointMatch) {
                    if (pointOnPrevFirstEdge) {
                        //connect A and prevB
                        face.connected_face_index[0] = get<1>(prevFaceIDs);
                        mesh.faces[get<1>(prevFaceIDs)].connected_face_index[0] = faceID;
                        
                        //connect B and prevA
                        newFaces.first.connected_face_index[0] = get<0>(prevFaceIDs);
                        mesh.faces[get<0>(prevFaceIDs)].connected_face_index[0] = newFaceIDs.first;
                        
                        if (trapezoidInOverhang) {
                            mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<1>(prevFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<0>(prevFaceIDs));
                        } else {
                            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<1>(prevFaceIDs));
                            mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<0>(prevFaceIDs));
                        }
                        
                    } else {
                        //connect A and prevA
                        face.connected_face_index[0] = get<0>(prevFaceIDs);
                        mesh.faces[get<0>(prevFaceIDs)].connected_face_index[1] = faceID;
                        
                        //connect B and prevC
                        newFaces.first.connected_face_index[0] = get<2>(prevFaceIDs);
                        mesh.faces[get<2>(prevFaceIDs)].connected_face_index[2] = newFaceIDs.first;
                        
                        if (trapezoidInOverhang) {
                            mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<0>(prevFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<1>(prevFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<2>(prevFaceIDs));
                        } else {
                            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<0>(prevFaceIDs));
                            mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<1>(prevFaceIDs));
                            mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<2>(prevFaceIDs));
                        }
                    }
                } else {
                    if (pointOnPrevFirstEdge) {
                        //connect A and prevA
                        face.connected_face_index[1] = get<0>(prevFaceIDs);
                        mesh.faces[get<0>(prevFaceIDs)].connected_face_index[0] = faceID;
                        
                        //connect C and prevB
                        newFaces.second.connected_face_index[2] = get<1>(prevFaceIDs);
                        mesh.faces[get<1>(prevFaceIDs)].connected_face_index[0] = newFaceIDs.second;
                        
                        if (trapezoidInOverhang) {
                            mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<0>(prevFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<1>(prevFaceIDs));
                        } else {
                            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<1>(prevFaceIDs));
                            mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<0>(prevFaceIDs));
                        }
                        
                    } else {
                        //connect A and prevC
                        face.connected_face_index[1] = get<2>(prevFaceIDs);
                        mesh.faces[get<2>(prevFaceIDs)].connected_face_index[2] = faceID;
                        
                        //connect C and prevA
                        newFaces.second.connected_face_index[2] = get<0>(prevFaceIDs);
                        mesh.faces[get<0>(prevFaceIDs)].connected_face_index[1] = newFaceIDs.second;
                        
                        if (trapezoidInOverhang) {
                            mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<1>(prevFaceIDs));
                            mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<2>(prevFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<0>(prevFaceIDs));
                        } else {
                            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<1>(prevFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<2>(prevFaceIDs));
                            mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<0>(prevFaceIDs));
                        }
                        
                    }
                }
                
                if (isFinalFaceInRecurse) {
                    if (firstPointMatch) {
                        if (trapezoidInOverhang) {
                            mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<1>(originalFaceIDs));
                            mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<2>(originalFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<0>(originalFaceIDs));
                        } else {
                            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<1>(originalFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<2>(originalFaceIDs));
                            mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<0>(originalFaceIDs));
                        }
                        
                        //connect A to originalC
                        face.connected_face_index[1] = get<2>(originalFaceIDs);
                        mesh.faces[get<2>(originalFaceIDs)].connected_face_index[2] = faceID;
                        
                        //connect C to originalA
                        newFaces.second.connected_face_index[2] = get<0>(originalFaceIDs);
                        mesh.faces[get<0>(originalFaceIDs)].connected_face_index[1] = newFaceIDs.second;
                        
                    } else {
                        if (trapezoidInOverhang) {
                            mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<0>(originalFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<1>(originalFaceIDs));
                            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<2>(originalFaceIDs));
                        } else {
                            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<0>(originalFaceIDs));
                            mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<1>(originalFaceIDs));
                            mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<2>(originalFaceIDs));
                        }
                        
                        //connect A to connectA
                        face.connected_face_index[0] = get<0>(originalFaceIDs);
                        mesh.faces[get<0>(originalFaceIDs)].connected_face_index[0] = faceID;
                        
                        //conect B to connectC
                        newFaces.first.connected_face_index[0] = get<2>(originalFaceIDs);
                        mesh.faces[get<2>(originalFaceIDs)].connected_face_index[0] = newFaceIDs.first;
                    }
                    
                    return;
                } else {
                    //reset variables
                    prevFaceCase = 1;
                    prevFaceIDs = tuple<int, int, int>(faceID, newFaceIDs.first, newFaceIDs.second);
                    
                    if (firstPointMatch) {
                        faceID = face.connected_face_index[y];
                        
                        prevSplitPoint = splitPoints.first;
                        prevSplitPointIndex = newVertexIndices.first;
                        prevSplitPointPrimeIndex = newVertexPrimeIndices.first;
                        pointOnPrevFirstEdge = true;
                    } else {
                        faceID = face.connected_face_index[x];
                        
                        prevSplitPoint = splitPoints.second;
                        prevSplitPointIndex = newVertexIndices.second;
                        prevSplitPointPrimeIndex = newVertexPrimeIndices.second;
                        pointOnPrevFirstEdge = true;
                    }
                    
                    numSplitPoints = findSplitPoints(mesh, faceID, intersectingPolyRef, splitPoints);
                }
            }
        }
    }
}

//void VolumeDecomposer::splitFaces(Mesh& mesh, PolygonRef intersectingPolyRef, int faceID, int numSplitPoints, pair<Point3, Point3>& splitPoints) {
//    if (numSplitPoints == 0) return;
//
//    // Grab the actual MeshFace
//    MeshFace& face = mesh.faces[faceID];
//
//    // If there's only one splitpoint, or the splitpoints are the same, draw a line from that point
//    // up and see if an intersect with another of the face's sides is found, then you've found your split
//    //
//    if (numSplitPoints == 1 || splitPoints.first == splitPoints.second) {
//        //TODO ahhhh what do we do here???
//    } else {
//        //save original vertex indices for future reference
//        int originalVertexIndices[3] = {-1};
//        originalVertexIndices[0] = face.vertex_index[0];
//        originalVertexIndices[1] = face.vertex_index[1];
//        originalVertexIndices[2] = face.vertex_index[2];
//
//        //determine which edges split points are on
//        pair<int, int> splitPointEdgeIndices(-1, -1);
//        FPoint3 edges[3];
//        edges[0] = mesh.vertices[face.vertex_index[1]].p - mesh.vertices[face.vertex_index[0]].p;
//        edges[1] = mesh.vertices[face.vertex_index[2]].p - mesh.vertices[face.vertex_index[1]].p;
//        edges[2] = mesh.vertices[face.vertex_index[0]].p - mesh.vertices[face.vertex_index[2]].p;
//
//        if (edges[0].cross(splitPoints.first - mesh.vertices[face.vertex_index[0]].p).vSize2() == 0) {
//            splitPointEdgeIndices.first = 0;
//        } else if (edges[1].cross(splitPoints.first - mesh.vertices[face.vertex_index[1]].p).vSize2() == 0) {
//            splitPointEdgeIndices.first = 1;
//        } else if (edges[2].cross(splitPoints.first - mesh.vertices[face.vertex_index[2]].p).vSize2() == 0) {
//            splitPointEdgeIndices.first = 2;
//        } else {
//            log("[ERROR] split point was not found to be on any edge of face");
//            return;
//        }
//
//        if (splitPointEdgeIndices.first == splitPointEdgeIndices.second) {
//            log("[ERROR] split points are on same edge");
//            return;
//        }
//
//        //if order is either 1-0, 2-0, or 2-1 switch it to 0-1, 0-2, or 1-2 repsectively so there's only 3 scenarios
//        bool switched = false;
//        if ((splitPointEdgeIndices.first == 1) && (splitPointEdgeIndices.second == 0)) {
//            splitPointEdgeIndices = pair<int, int>(0, 1);
//            switched = true;
//        } else if ((splitPointEdgeIndices.first == 2) && (splitPointEdgeIndices.first == 1)) {
//            splitPointEdgeIndices = pair<int, int>(1, 2);
//            switched = true;
//        } else if ((splitPointEdgeIndices.second == 0) && (splitPointEdgeIndices.first == 2)) {
//            splitPointEdgeIndices = pair<int, int>(2, 0);
//            switched = true;
//        }
//
//        if (switched) {
//            Point3 temp = splitPoints.first;
//            splitPoints.first = splitPoints.second;
//            splitPoints.second = temp;
//        }
//
//        //at this point, the edges that the split points are on are determined
//
//        int x, y, z;
//        if ((splitPointEdgeIndices.first == 0) && (splitPointEdgeIndices.second == 1)) {
//            x = 0;
//            y = 1;
//            z = 2;
//        } else if ((splitPointEdgeIndices.first == 1) && (splitPointEdgeIndices.second == 2)) {
//            x = 1;
//            y = 2;
//            z = 0;
//        } else if ((splitPointEdgeIndices.first == 2) && (splitPointEdgeIndices.second == 0)) {
//            x = 2;
//            y = 0;
//            z = 1;
//        } else {
//            log("[ERROR] split points are on impossible order of edges");
//            return;
//        }
//
//        bool trapezoidInOverhang = intersectingPolyRef.inside(face.vertex_index[y]); //y vertex is connected to base volume
//
//        //add split points to vertex list
//        mesh.vertices.push_back(MeshVertex(splitPoints.first));
//        mesh.vertices.push_back(MeshVertex(splitPoints.second));
//        pair<int, int> newVertexIndices(mesh.vertices.size() - 2, mesh.vertices.size() - 1); //indexes of new vertices in mesh
//
//        mesh.vertices.push_back(MeshVertex(splitPoints.first));
//        mesh.vertices.push_back(MeshVertex(splitPoints.second));
//        pair<int, int> newVertexPrimeIndices(mesh.vertices.size() - 2, mesh.vertices.size() - 1); //indexes of new vertices in mesh
//
//        //add new faces
//        mesh.faces.push_back(MeshFace());
//        mesh.faces.push_back(MeshFace());
//        pair<int, int> newFaceIDs(mesh.faces.size() - 2, mesh.faces.size() - 1);
//        pair<MeshFace&, MeshFace&> newFaces(mesh.faces[newFaceIDs.first], mesh.faces[newFaceIDs.second]);
//
//        if (trapezoidInOverhang) {
//            //alter original face
//            face.vertex_index[x] = newVertexIndices.first;
//            face.vertex_index[z] = newVertexIndices.second;
//
//            //add first face
//            newFaces.first.vertex_index[0] = originalVertexIndices[x];
//            newFaces.first.vertex_index[1] = newVertexPrimeIndices.first;
//            newFaces.first.vertex_index[2] = newVertexPrimeIndices.second;
//
//            //add second face
//            newFaces.second.vertex_index[0] = originalVertexIndices[z];
//            newFaces.second.vertex_index[1] = originalVertexIndices[x];
//            newFaces.second.vertex_index[2] = newVertexPrimeIndices.second;
//        } else {
//            //alter original face
//            face.vertex_index[x] = newVertexPrimeIndices.first;
//            face.vertex_index[z] = newVertexPrimeIndices.second;
//
//            //add first face
//            newFaces.first.vertex_index[0] = originalVertexIndices[x];
//            newFaces.first.vertex_index[1] = newVertexIndices.first;
//            newFaces.first.vertex_index[2] = newVertexIndices.second;
//
//            //add second face
//            newFaces.second.vertex_index[0] = originalVertexIndices[z];
//            newFaces.second.vertex_index[1] = originalVertexIndices[x];
//            newFaces.second.vertex_index[2] = newVertexIndices.second;
//        }
//
//        //add connected faces to new vertices
//        if (trapezoidInOverhang) {
//            mesh.vertices[newVertexIndices.first].connected_faces.push_back(faceID);
//            mesh.vertices[newVertexIndices.second].connected_faces.push_back(faceID);
//            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(newFaceIDs.first);
//            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(newFaceIDs.first);
//            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(newFaceIDs.second);
//        } else {
//            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(faceID);
//            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(faceID);
//            mesh.vertices[newVertexIndices.first].connected_faces.push_back(newFaceIDs.first);
//            mesh.vertices[newVertexIndices.second].connected_faces.push_back(newFaceIDs.first);
//            mesh.vertices[newVertexIndices.second].connected_faces.push_back(newFaceIDs.second);
//        }
//
//        //--fix adjacent faces--
//
//        //remove connected faces from existing vertices
//
//        vector<uint32_t>::iterator it; //index of faceID in vector
//
//        //remove original face from xth vertex
//        it = find(mesh.vertices[originalVertexIndices[x]].connected_faces.begin(), mesh.vertices[originalVertexIndices[x]].connected_faces.end(), faceID);
//        if (it != mesh.vertices[originalVertexIndices[x]].connected_faces.end()) {
//            mesh.vertices[originalVertexIndices[x]].connected_faces.erase(it);
//        }
//        //add first and second added face to xth vertex
//        mesh.vertices[originalVertexIndices[x]].connected_faces.push_back(newFaceIDs.first);
//        mesh.vertices[originalVertexIndices[x]].connected_faces.push_back(newFaceIDs.second);
//
//        //remove original face from zth vertex
//        it = find(mesh.vertices[originalVertexIndices[z]].connected_faces.begin(), mesh.vertices[originalVertexIndices[z]].connected_faces.end(), faceID);
//        if (it != mesh.vertices[originalVertexIndices[z]].connected_faces.end()) {
//            mesh.vertices[originalVertexIndices[z]].connected_faces.erase(it);
//        }
//        //add just second added face to xth vertex
//        mesh.vertices[originalVertexIndices[z]].connected_faces.push_back(newFaceIDs.second);
//
//
//        //updated index of adjacent face
//        if (mesh.faces[face.connected_face_index[z]].connected_face_index[0] == faceID) {
//            mesh.faces[face.connected_face_index[z]].connected_face_index[0] = newFaceIDs.second;
//        } else if (mesh.faces[face.connected_face_index[z]].connected_face_index[1] == faceID) {
//            mesh.faces[face.connected_face_index[z]].connected_face_index[1] = newFaceIDs.second;
//        } else if (mesh.faces[face.connected_face_index[z]].connected_face_index[2] == faceID) {
//            mesh.faces[face.connected_face_index[z]].connected_face_index[2] = newFaceIDs.second;
//        } else {
//            log("[ERROR] original face is not listed as an adjacent face of adjacent face");
//            return;
//        }
//        newFaces.first.connected_face_index[2] = newFaceIDs.second;
//        newFaces.second.connected_face_index[1] = newFaceIDs.first;
//        newFaces.second.connected_face_index[0] = face.connected_face_index[z];
//        face.connected_face_index[z] = -1;
//
//        //call recursive function
//        int nextFaceID = face.connected_face_index[x];
//
//        pair<Point3, Point3> nextSplitPoints;
//        int nextNumSplitPoints = findSplitPoints(mesh, nextFaceID, intersectingPolyRef, nextSplitPoints);
//
//        splitFaceRecursiveA(mesh, intersectingPolyRef, face.connected_face_index[0], nextNumSplitPoints, nextSplitPoints, splitPoints.first, newVertexIndices.first, newVertexPrimeIndices.first, true, faceID, newFaceIDs.first, newFaceIDs.second, faceID, newFaceIDs.first, newFaceIDs.second);
//    }
//}
//
////TODO this may not need to be recursive - look into
////this should only be called if the previous split point does NOT lie on a vertex
//void VolumeDecomposer::splitFaceRecursiveA(Mesh& mesh, PolygonRef intersectingPolyRef, int faceID, int numSplitPoints, pair<Point3, Point3>& splitPoints, Point3 prevSplitPoint, int prevSplitPointIndex, int prevSplitPointPrimeIndex, bool pointOnPrevFirstEdge, int prevFaceIDA, int prevFaceIDB, int prevFaceIDC, int originalFaceIDA, int originalFaceIDB, int originalFaceIDC) {
//
//    if (numSplitPoints == 0) {
//        log("[ERROR] adjacent face of split face has no split points");
//        return;
//    }
//
//    // Grab the actual MeshFace
//    MeshFace& face = mesh.faces[faceID];
//
//    //save original vertex indices for future reference
//    int originalVertexIndices[3] = {-1};
//    originalVertexIndices[0] = face.vertex_index[0];
//    originalVertexIndices[1] = face.vertex_index[1];
//    originalVertexIndices[2] = face.vertex_index[2];
//
//    pair<int, int> splitPointVertexIntersectionIndices(-1, -1);
//
//    if (splitPoints.first == mesh.vertices[face.vertex_index[0]].p)
//        splitPointVertexIntersectionIndices.first = 0;
//    else if (splitPoints.first == mesh.vertices[face.vertex_index[1]].p)
//        splitPointVertexIntersectionIndices.first = 1;
//    else if (splitPoints.first == mesh.vertices[face.vertex_index[2]].p) {
//        splitPointVertexIntersectionIndices.first = 2;
//    }
//
//    if (splitPoints.second == mesh.vertices[face.vertex_index[0]].p)
//        splitPointVertexIntersectionIndices.second = 0;
//    else if (splitPoints.second == mesh.vertices[face.vertex_index[1]].p)
//        splitPointVertexIntersectionIndices.second = 1;
//    else if (splitPoints.second == mesh.vertices[face.vertex_index[2]].p) {
//        splitPointVertexIntersectionIndices.second = 2;
//    }
//
//    if ((splitPointVertexIntersectionIndices.first >= 0) && (splitPointVertexIntersectionIndices.second >= 0)) {
//        log("[ERROR] splitFaceRecursiveA() has two split points intersecting vertices");
//        return;
//    }
//
//    if ((splitPointVertexIntersectionIndices.first >= 0) || (splitPointVertexIntersectionIndices.second >= 0)) { //this means one of the split points is intersecting a vertex
//
//        bool switched = false;
//        if (splitPointVertexIntersectionIndices.second >= 0) {
//            switched = true;
//
//            int tempInt = splitPointVertexIntersectionIndices.first;
//            splitPointVertexIntersectionIndices.first = splitPointVertexIntersectionIndices.second;
//            splitPointVertexIntersectionIndices.second = tempInt;
//
//            Point3 tempPoint = splitPoints.first;
//            splitPoints.first = splitPoints.second;
//            splitPoints.second = tempPoint;
//        }
//
//        int x, y, z;
//        if (splitPointVertexIntersectionIndices.first == 0) {
//            x = 0;
//            y = 1;
//            z = 2;
//        } else if (splitPointVertexIntersectionIndices.first == 1) {
//            x = 1;
//            y = 2;
//            z = 0;
//        } else if (splitPointVertexIntersectionIndices.first == 2) {
//            x = 2;
//            y = 0;
//            z = 1;
//        } else {
//            log("[ERROR] split points vertex intersection index is not 0, 1, or 2");
//            return;
//        }
//
//        int newVertexIndex = prevSplitPointIndex;
//
//        mesh.vertices.push_back(MeshVertex(splitPoints.second));
//        pair<int, int> newVertexPrimeIndices(prevSplitPointPrimeIndex, mesh.vertices.size() - 1);
//
//        mesh.faces.push_back(MeshFace());
//        int newFaceID = mesh.faces.size() - 1;
//        MeshFace& newFace = mesh.faces[newFaceID];
//
//        if (intersectingPolyRef.inside(Point(mesh.vertices[face.vertex_index[y]].p.x, mesh.vertices[face.vertex_index[y]].p.y))) {
//            newFace.vertex_index[0] = newVertexPrimeIndices.second;
//            newFace.vertex_index[1] = face.vertex_index[y];
//            newFace.vertex_index[2] = newVertexPrimeIndices.first;
//            newFace.connected_face_index[0] = face.connected_face_index[0];
//
//            face.vertex_index[y] = newVertexIndex;
//            face.connected_face_index[x] = -1;
//
//            if (pointOnPrevFirstEdge) {
//                //conect A and prevB
//                face.connected_face_index[1] = prevFaceIDB;
//                mesh.faces[prevFaceIDB].connected_face_index[0] = faceID;
//
//                //connect B and prevA
//                newFace.connected_face_index[1] = prevFaceIDA;
//                mesh.faces[prevFaceIDA].connected_face_index[0] = newFaceID;
//
//                //call recursively on xth edge of newFace
//            }
//
//        } else {
//            newFace.vertex_index[0] = newVertexPrimeIndices.second;
//            newFace.vertex_index[1] = newVertexPrimeIndices.first;
//            newFace.vertex_index[2] = face.vertex_index[z];
//            newFace.connected_face_index[2] = face.connected_face_index[2];
//
//            face.vertex_index[z] = newVertexIndex;
//            face.connected_face_index[z] = -1;
//
//            if (pointOnPrevFirstEdge) {
//                //connect A and prevA
//                face.connected_face_index[1] = prevFaceIDA;
//                mesh.faces[prevFaceIDA].connected_face_index[0] = faceID;
//
//                //conect B and prevB
//                newFace.connected_face_index[1] = prevFaceIDB;
//                mesh.faces[prevFaceIDB].connected_face_index[0] = newFaceID;
//
//                //call recursively on zth edge of newFace
//            }
//        }
//
//    } else {
//
//        //determine which edges split points are on
//        pair<int, int> splitPointEdgeIndices(-1, -1);
//
//        FPoint3 edges[3];
//        edges[0] = mesh.vertices[face.vertex_index[1]].p - mesh.vertices[face.vertex_index[0]].p;
//        edges[1] = mesh.vertices[face.vertex_index[2]].p - mesh.vertices[face.vertex_index[1]].p;
//        edges[2] = mesh.vertices[face.vertex_index[0]].p - mesh.vertices[face.vertex_index[2]].p;
//
//        if (edges[0].cross(splitPoints.first - mesh.vertices[face.vertex_index[0]].p).vSize2() == 0) {
//            splitPointEdgeIndices.first = 0;
//        } else if (edges[1].cross(splitPoints.first - mesh.vertices[face.vertex_index[1]].p).vSize2() == 0) {
//            splitPointEdgeIndices.first = 1;
//        } else if (edges[2].cross(splitPoints.first - mesh.vertices[face.vertex_index[2]].p).vSize2() == 0) {
//            splitPointEdgeIndices.first = 2;
//        } else {
//            log("[ERROR] split point was not found to be on any edge of face");
//            return;
//        }
//
//        if (splitPointEdgeIndices.first == splitPointEdgeIndices.second) {
//            log("[ERROR] split points are on same edge");
//            return;
//        }
//
//        //if order is either 1-0, 2-0, or 2-1 switch it to 0-1, 0-2, or 1-2 repsectively so there's only 3 scenarios
//        bool switched = false;
//        if ((splitPointEdgeIndices.first == 1) && (splitPointEdgeIndices.second == 0)) {
//            splitPointEdgeIndices = pair<int, int>(0, 1);
//            switched = true;
//        } else if ((splitPointEdgeIndices.first == 2) && (splitPointEdgeIndices.first == 1)) {
//            splitPointEdgeIndices = pair<int, int>(1, 2);
//            switched = true;
//        } else if ((splitPointEdgeIndices.second == 0) && (splitPointEdgeIndices.first == 2)) {
//            splitPointEdgeIndices = pair<int, int>(2, 0);
//            switched = true;
//        }
//
//        if (switched) {
//            Point3 temp = splitPoints.first;
//            splitPoints.first = splitPoints.second;
//            splitPoints.second = temp;
//        }
//
//        //at this point, the edges that the split points are on are determined
//
//        int x, y, z;
//        if ((splitPointEdgeIndices.first == 0) && (splitPointEdgeIndices.second == 1)) {
//            x = 0;
//            y = 1;
//            z = 2;
//        } else if ((splitPointEdgeIndices.first == 1) && (splitPointEdgeIndices.second == 2)) {
//            x = 1;
//            y = 2;
//            z = 0;
//        } else if ((splitPointEdgeIndices.first == 2) && (splitPointEdgeIndices.second == 0)) {
//            x = 2;
//            y = 0;
//            z = 1;
//        } else {
//            log("[ERROR] split points are on impossible order of edges");
//            return;
//        }
//
//        //if this is the last triangle before return to original face where cut was started
//        bool isFinalFaceInRecurse = (((face.connected_face_index[x] == originalFaceIDA) || (face.connected_face_index[y] == originalFaceIDA)) && (prevFaceIDA != originalFaceIDA)); //oh shit it's OG!!
//
//        //whether or not the trapezoid created by the cut is in the overhang
//        bool trapezoidInOverhang = intersectingPolyRef.inside(face.vertex_index[y]); //y vertex is connected to base volume
//
//        //whether or not the first split point is the split point from the previous face
//        bool firstPointMatch = true;
//
//        pair<int, int> newVertexIndices, newVertexPrimeIndices;
//        if (isFinalFaceInRecurse) {
//            if (splitPoints.first == prevSplitPoint) {
//                firstPointMatch = true;
//
//                if (trapezoidInOverhang) {
//                    newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.faces[originalFaceIDA].vertex_index[2] - 2); //indexes of new vertices in mesh
//                    newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.faces[originalFaceIDA].vertex_index[2]); //indexes of new vertices in mesh
//                } else {
//                    newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.faces[originalFaceIDA].vertex_index[2]); //indexes of new vertices in mesh
//                    newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.faces[originalFaceIDA].vertex_index[2] + 2); //indexes of new vertices in mesh
//                }
//            } else if (splitPoints.second == prevSplitPoint) {
//                firstPointMatch = false;
//
//                if (trapezoidInOverhang) {
//                    newVertexIndices = pair<int, int>(mesh.faces[originalFaceIDA].vertex_index[2], prevSplitPointIndex); //indexes of new vertices in mesh
//                    newVertexPrimeIndices = pair<int, int>(mesh.faces[originalFaceIDA].vertex_index[2] + 2, prevSplitPointPrimeIndex); //indexes of new vertices in mesh
//                } else {
//                    newVertexIndices = pair<int, int>(mesh.faces[originalFaceIDA].vertex_index[2] - 2, prevSplitPointIndex); //indexes of new vertices in mesh
//                    newVertexPrimeIndices = pair<int, int>(mesh.faces[originalFaceIDA].vertex_index[2], prevSplitPointPrimeIndex); //indexes of new vertices in mesh
//                }
//            } else {
//                log("[ERROR] adjacent face to split face does not have a matching split point");
//                return;
//            }
//        } else {
//            if (splitPoints.first == prevSplitPoint) {
//                firstPointMatch = true;
//
//                //add split points to vertex list
//                mesh.vertices.push_back(MeshVertex(splitPoints.second));
//                newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.vertices.size() - 1); //indexes of new vertices in mesh
//
//                mesh.vertices.push_back(MeshVertex(splitPoints.second));
//                newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.vertices.size() - 1); //indexes of new vertices in mesh
//            } else if (splitPoints.second == prevSplitPoint) {
//                firstPointMatch = false;
//
//                mesh.vertices.push_back(MeshVertex(splitPoints.first));
//                newVertexIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointIndex); //indexes of new vertices in mesh
//
//                mesh.vertices.push_back(MeshVertex(splitPoints.first));
//                newVertexPrimeIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointPrimeIndex); //indexes of new vertices in mesh
//            } else {
//                log("[ERROR] adjacent face to split face does not have a matching split point");
//                return;
//            }
//        }
//
//        //add new faces
//        mesh.faces.push_back(MeshFace());
//        mesh.faces.push_back(MeshFace());
//        pair<int, int> newFaceIDs(mesh.faces.size() - 2, mesh.faces.size() - 1);
//        pair<MeshFace&, MeshFace&> newFaces(mesh.faces[newFaceIDs.first], mesh.faces[newFaceIDs.second]);
//
//        if (trapezoidInOverhang) {
//            //alter original face
//            face.vertex_index[x] = newVertexIndices.first;
//            face.vertex_index[z] = newVertexIndices.second;
//
//            //add first face
//            newFaces.first.vertex_index[0] = originalVertexIndices[x];
//            newFaces.first.vertex_index[1] = newVertexPrimeIndices.first;
//            newFaces.first.vertex_index[2] = newVertexPrimeIndices.second;
//
//            //add second face
//            newFaces.second.vertex_index[0] = originalVertexIndices[z];
//            newFaces.second.vertex_index[1] = originalVertexIndices[x];
//            newFaces.second.vertex_index[2] = newVertexPrimeIndices.second;
//        } else {
//            //alter original face
//            face.vertex_index[x] = newVertexPrimeIndices.first;
//            face.vertex_index[z] = newVertexPrimeIndices.second;
//
//            //add first face
//            newFaces.first.vertex_index[0] = originalVertexIndices[x];
//            newFaces.first.vertex_index[1] = newVertexIndices.first;
//            newFaces.first.vertex_index[2] = newVertexIndices.second;
//
//            //add second face
//            newFaces.second.vertex_index[0] = originalVertexIndices[z];
//            newFaces.second.vertex_index[1] = originalVertexIndices[x];
//            newFaces.second.vertex_index[2] = newVertexIndices.second;
//        }
//
//        //add connected faces to new vertices
//        if (trapezoidInOverhang) {
//            mesh.vertices[newVertexIndices.first].connected_faces.push_back(faceID);
//            mesh.vertices[newVertexIndices.second].connected_faces.push_back(faceID);
//            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(newFaceIDs.first);
//            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(newFaceIDs.first);
//            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(newFaceIDs.second);
//        } else {
//            mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(faceID);
//            mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(faceID);
//            mesh.vertices[newVertexIndices.first].connected_faces.push_back(newFaceIDs.first);
//            mesh.vertices[newVertexIndices.second].connected_faces.push_back(newFaceIDs.first);
//            mesh.vertices[newVertexIndices.second].connected_faces.push_back(newFaceIDs.second);
//        }
//
//        //--fix adjacent faces--
//
//        //remove connected faces from existing vertices
//
//        vector<uint32_t>::iterator it; //index of faceID in vector
//
//        //remove original face from xth vertex
//        it = find(mesh.vertices[originalVertexIndices[x]].connected_faces.begin(), mesh.vertices[originalVertexIndices[x]].connected_faces.end(), faceID);
//        if (it != mesh.vertices[originalVertexIndices[x]].connected_faces.end()) {
//            mesh.vertices[originalVertexIndices[x]].connected_faces.erase(it);
//        }
//        //add first and second added face to xth vertex
//        mesh.vertices[originalVertexIndices[x]].connected_faces.push_back(newFaceIDs.first);
//        mesh.vertices[originalVertexIndices[x]].connected_faces.push_back(newFaceIDs.second);
//
//        //remove original face from zth vertex
//        it = find(mesh.vertices[originalVertexIndices[z]].connected_faces.begin(), mesh.vertices[originalVertexIndices[z]].connected_faces.end(), faceID);
//        if (it != mesh.vertices[originalVertexIndices[z]].connected_faces.end()) {
//            mesh.vertices[originalVertexIndices[z]].connected_faces.erase(it);
//        }
//        //add just second added face to xth vertex
//        mesh.vertices[originalVertexIndices[z]].connected_faces.push_back(newFaceIDs.second);
//
//        //updated index of adjacent face
//        if (mesh.faces[face.connected_face_index[z]].connected_face_index[0] == faceID) {
//            mesh.faces[face.connected_face_index[z]].connected_face_index[0] = newFaceIDs.second;
//        } else if (mesh.faces[face.connected_face_index[z]].connected_face_index[1] == faceID) {
//            mesh.faces[face.connected_face_index[z]].connected_face_index[1] = newFaceIDs.second;
//        } else if (mesh.faces[face.connected_face_index[z]].connected_face_index[2] == faceID) {
//            mesh.faces[face.connected_face_index[z]].connected_face_index[2] = newFaceIDs.second;
//        } else {
//            log("[ERROR] original face is not listed as an adjacent face of adjacent face");
//            return;
//        }
//        newFaces.first.connected_face_index[2] = newFaceIDs.second;
//        newFaces.second.connected_face_index[1] = newFaceIDs.first;
//        newFaces.second.connected_face_index[0] = face.connected_face_index[z];
//        face.connected_face_index[z] = -1;
//
//        //----ADDITION RECURSIVE CODE----
//
//        if (firstPointMatch) {
//            if (pointOnPrevFirstEdge) {
//                //connect A and prevB
//                face.connected_face_index[0] = prevFaceIDB;
//                mesh.faces[prevFaceIDB].connected_face_index[0] = faceID;
//
//                //connect B and prevA
//                newFaces.first.connected_face_index[0] = prevFaceIDA;
//                mesh.faces[prevFaceIDA].connected_face_index[0] = newFaceIDs.first;
//
//                if (trapezoidInOverhang) {
//                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(prevFaceIDB);
//                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(prevFaceIDA);
//                } else {
//                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(prevFaceIDB);
//                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(prevFaceIDA);
//                }
//
//            } else {
//                //connect A and prevA
//                face.connected_face_index[0] = prevFaceIDA;
//                mesh.faces[prevFaceIDA].connected_face_index[1] = faceID;
//
//                //connect B and prevC
//                newFaces.first.connected_face_index[0] = prevFaceIDC;
//                mesh.faces[prevFaceIDC].connected_face_index[2] = newFaceIDs.first;
//
//                if (trapezoidInOverhang) {
//                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(prevFaceIDA);
//                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(prevFaceIDB);
//                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(prevFaceIDC);
//                } else {
//                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(prevFaceIDA);
//                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(prevFaceIDB);
//                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(prevFaceIDC);
//                }
//            }
//        } else {
//            if (pointOnPrevFirstEdge) {
//                //connect A and prevA
//                face.connected_face_index[1] = prevFaceIDA;
//                mesh.faces[prevFaceIDA].connected_face_index[0] = faceID;
//
//                //connect C and prevB
//                newFaces.second.connected_face_index[2] = prevFaceIDB;
//                mesh.faces[prevFaceIDB].connected_face_index[0] = newFaceIDs.second;
//
//                if (trapezoidInOverhang) {
//                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(prevFaceIDA);
//                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(prevFaceIDB);
//                } else {
//                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(prevFaceIDB);
//                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(prevFaceIDA);
//                }
//
//            } else {
//                //connect A and prevC
//                face.connected_face_index[1] = prevFaceIDC;
//                mesh.faces[prevFaceIDC].connected_face_index[2] = faceID;
//
//                //connect C and prevA
//                newFaces.second.connected_face_index[2] = prevFaceIDA;
//                mesh.faces[prevFaceIDA].connected_face_index[1] = newFaceIDs.second;
//
//                if (trapezoidInOverhang) {
//                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(prevFaceIDB);
//                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(prevFaceIDC);
//                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(prevFaceIDA);
//                } else {
//                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(prevFaceIDB);
//                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(prevFaceIDC);
//                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(prevFaceIDA);
//                }
//
//            }
//        }
//
//        if (isFinalFaceInRecurse) {
//            if (firstPointMatch) {
//                if (trapezoidInOverhang) {
//                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(originalFaceIDB);
//                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(originalFaceIDC);
//                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(originalFaceIDA);
//                } else {
//                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(originalFaceIDB);
//                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(originalFaceIDC);
//                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(originalFaceIDA);
//                }
//
//                //connect A to originalC
//                face.connected_face_index[1] = originalFaceIDC;
//                mesh.faces[originalFaceIDC].connected_face_index[2] = faceID;
//
//                //connect C to originalA
//                newFaces.second.connected_face_index[2] = originalFaceIDA;
//                mesh.faces[originalFaceIDA].connected_face_index[1] = newFaceIDs.second;
//
//            } else {
//                if (trapezoidInOverhang) {
//                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(originalFaceIDA);
//                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(originalFaceIDB);
//                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(originalFaceIDC);
//                } else {
//                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(originalFaceIDA);
//                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(originalFaceIDB);
//                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(originalFaceIDC);
//                }
//
//                //connect A to connectA
//                face.connected_face_index[0] = originalFaceIDA;
//                mesh.faces[originalFaceIDA].connected_face_index[0] = faceID;
//
//                //conect B to connectC
//                newFaces.first.connected_face_index[0] = originalFaceIDC;
//                mesh.faces[originalFaceIDC].connected_face_index[0] = newFaceIDs.first;
//            }
//        } else {
//
//            if (firstPointMatch) {
//                int nextFaceID = face.connected_face_index[y];
//
//                pair<Point3, Point3> nextSplitPoints;
//                int nextNumSplitPoints = findSplitPoints(mesh, nextFaceID, intersectingPolyRef, nextSplitPoints);
//
//                splitFaceRecursiveA(mesh, intersectingPolyRef, nextFaceID, nextNumSplitPoints, nextSplitPoints, splitPoints.second, newVertexIndices.second, newVertexPrimeIndices.second, false, faceID, newFaceIDs.first, newFaceIDs.second, originalFaceIDA, originalFaceIDB, originalFaceIDC);
//            } else {
//                int nextFaceID = face.connected_face_index[x];
//
//                pair<Point3, Point3> nextSplitPoints;
//                int nextNumSplitPoints = findSplitPoints(mesh, nextFaceID, intersectingPolyRef, nextSplitPoints);
//
//                splitFaceRecursiveA(mesh, intersectingPolyRef, nextFaceID, nextNumSplitPoints, nextSplitPoints, splitPoints.first, newVertexIndices.first, newVertexPrimeIndices.first, true, faceID, newFaceIDs.first, newFaceIDs.second, originalFaceIDA, originalFaceIDB, originalFaceIDC);
//            }
//        }
//    }
//
//    //----END ADDITION RECURSIVE CODE----
//}
//
//void VolumeDecomposer::splitFaceRecursiveB(Mesh& mesh, PolygonRef intersectingPolyRef, int faceID, int numSplitPoints, pair<Point3, Point3>& splitPoints, Point3 prevSplitPoint, int prevSplitPointIndex, int prevSplitPointPrimeIndex, bool pointOnPrevFirstEdge, int prevFaceID, int originalFaceIDA, int originalFaceIDB, int originalFaceIDC) {
//
//    if (splitPoints.first == splitPoints.second) {
//        //this is assuming that the face is on the overhang side
//
//        // Grab the actual MeshFace
//        MeshFace& face = mesh.faces[faceID];
//
//        //save original vertex indices for future reference
//        int originalVertexIndices[3] = {-1};
//        originalVertexIndices[0] = face.vertex_index[0];
//        originalVertexIndices[1] = face.vertex_index[1];
//        originalVertexIndices[2] = face.vertex_index[2];
//
//        int x, y, z;
//        if (splitPoints.first == mesh.vertices[face.vertex_index[0]].p) {
//            x = 0;
//            y = 1;
//            z = 2;
//        } else if (splitPoints.first == mesh.vertices[face.vertex_index[1]].p) {
//            x = 1;
//            y = 2;
//            z = 0;
//        } else if (splitPoints.first == mesh.vertices[face.vertex_index[2]].p) {
//            x = 2;
//            y = 0;
//            z = 1;
//        } else {
//            log("[ERROR] splitFaceRecursiveB() has matching split points not on a vertex");
//            return;
//        }
//
//        vector<uint32_t>::iterator it; //index of faceID in vector
//
//        //remove original face from xth vertex
//        it = find(mesh.vertices[originalVertexIndices[x]].connected_faces.begin(), mesh.vertices[originalVertexIndices[x]].connected_faces.end(), faceID);
//        if (it != mesh.vertices[originalVertexIndices[x]].connected_faces.end()) {
//            mesh.vertices[originalVertexIndices[x]].connected_faces.erase(it);
//        }
//
//        //add face to overhang vertex
//        mesh.vertices[prevSplitPointPrimeIndex].connected_faces.push_back(faceID);
//
//        int nextFaceID = -1;
//
//        if (face.connected_face_index[x] == prevFaceID) {
//            nextFaceID = z;
//        } else if (face.connected_face_index[z] == prevFaceID) {
//            nextFaceID = x;
//        }
//
//        pair<Point3, Point3> nextSplitPoints;
//        int nextNumSplitPoints = findSplitPoints(mesh, nextFaceID, intersectingPolyRef, nextSplitPoints);
//
//        //TODO call recursively
//
//    } else {
//
//    }
//
//}

bool VolumeDecomposer::isOn(Point a, Point b, Point c) {
    // Return true iff point c intersects the line segment from a to b.
    // (or the degenerate case that all 3 points are coincident)
    if (!collinear(a, b, c)) return false;
    
    if (a.X != b.X) {
        return within(a.X, c.X, b.X);
    } else if (a.Y != b.Y) {
        return within(a.Y, c.Y, b.Y);
    } else {
        return false;
    }
}

bool VolumeDecomposer::collinear(Point a, Point b, Point c) {
    // Return true iff a, b, and c all lie on the same line.
    return (b.X - a.X) * (c.Y - a.Y) == (c.X - a.X) * (b.Y - a.Y);
}

bool VolumeDecomposer::within(double p, double q, double r) {
    // Return true iff q is between p and r (inclusive).
    return (p <= q && q <= r) || (r <= q && q <= p);
}

Polygons VolumeDecomposer::buildFacePolygon(Point3 p0, Point3 p1, Point3 p2) {
    ClipperLib::Path facePolyPath;
    Point flat_p0, flat_p1, flat_p2;
    flat_p0 = Point(p0.x, p0.y);
    flat_p1 = Point(p1.x, p1.y);
    flat_p2 = Point(p2.x, p2.y);
    
    // First three checks are for faces that are parallel to the build direction and therefore
    // flatten into a polyline rather than a polygon
    if (isOn(flat_p0, flat_p1, flat_p2)) {
        facePolyPath.push_back(Point(p1.x, p1.y));
        facePolyPath.push_back(Point(p0.x, p0.y));
    }
    else if (isOn(flat_p0, flat_p2, flat_p1)) {
        facePolyPath.push_back(Point(p2.x, p2.y));
        facePolyPath.push_back(Point(p0.x, p0.y));
    }
    else if (isOn(flat_p1, flat_p2, flat_p0)) {
        facePolyPath.push_back(Point(p2.x, p2.y));
        facePolyPath.push_back(Point(p1.x, p1.y));
    } else {
        facePolyPath.push_back(Point(p2.x, p2.y));
        facePolyPath.push_back(Point(p1.x, p1.y));
        facePolyPath.push_back(Point(p0.x, p0.y));
    }
    PolygonRef facePolyRef = PolygonRef(facePolyPath);
    double facePolyArea = facePolyRef.area();
    
    // Make sure the face is not being interpreted as a hole
    if (facePolyArea < 0) {
        facePolyRef.reverse();
    }
    
    // Create the polygons we'll be performing difference operations on
    Polygons facePoly = Polygons();
    facePoly.add(facePolyRef);
    
    return facePoly;
}

unsigned int VolumeDecomposer::findSplitPoints(Mesh& mesh, int faceID, PolygonRef intersectingPolyRef, pair<Point3, Point3>& result) {
    // The three points of the vertices of the face
    Point3 p0, p1, p2;
    
    // Gather face and vertex references
    const MeshFace& face = mesh.faces[faceID];
    const MeshVertex& v0 = mesh.vertices[face.vertex_index[0]];
    const MeshVertex& v1 = mesh.vertices[face.vertex_index[1]];
    const MeshVertex& v2 = mesh.vertices[face.vertex_index[2]];
    
    // Build out the face polygon and retrieve area
    p0 = v0.p;
    p1 = v1.p;
    p2 = v2.p;
    Polygons facePoly = buildFacePolygon(p0, p1, p2);
    Polygons intersectingPoly = Polygons();
    intersectingPoly.add(intersectingPolyRef);
    double facePolyArea = facePoly[0].area();
    
    // Keep the zeros consistent
    if (facePolyArea == -0) {
        facePolyArea = 0;
    }
    
    // log("INTERSECTING, number polys: (%d, %d)\n", facePoly.size(), intersectingPoly.size());
    
    // log("Face poly:\n");
    // for (unsigned int i = 0; i < facePoly[0].size(); ++i) {
    // 	log("\tPoint #%d: <%d, %d>\n", i, facePoly[0][i].X, facePoly[0][i].Y);
    // }
    // log("Intersecting poly:\n");
    // for (unsigned int i = 0; i < intersectingPoly[0].size(); ++i) {
    // 	log("\tPoint #%d: <%d, %d>\n", i, intersectingPoly[0][i].X, intersectingPoly[0][i].Y);
    // }
    
    Polygons diff;
    
    // If the face is a simple polyline and that polyline is on the egde of the intersecting
    // polygon, then the line may not be correctly cut, so we expand the polygon by 1
    // and hope the small inaccuracy in resulting coordinates doesn't matter.
    // Later code also compensates for this by checking for equality within +/- 1 of each coord
    // TODO: Fix this
    intersectingPoly = intersectingPoly.offset(1);
    if (facePolyArea == 0) {
        ClipperLib::PolyTree diffTree = facePoly.lineSegmentDifference(intersectingPoly);
        PolygonRef diffRef = PolygonRef(diffTree.GetFirst()->Contour);
        // log("[INFO] Number of nodes in tree: %d\n", diffTree.Total());
        // log("[INFO] %s\n", polygonRefToString(diffRef).c_str());
        
        // ClipperLib::PolyNode* pn = diffTree.GetFirst();
        // while (pn) {
        // 	PolygonRef pref = PolygonRef(pn->Contour);
        // 	log("%s", polygonRefToString(pref).c_str());
        // 	pn = pn->GetNext();
        // }
        
        diff = Polygons();
        diff.add(diffRef);
    } else {
        diff = facePoly.difference(intersectingPoly);
    }
    
    // log("Number of polys in diff: %d\n", diff.size());
    
    // Look for the (up to) two new points, which will be the split point
    Point* splitPoints[2] = { 0, 0 };
    unsigned int numSplitPoints = 0;
    const PolygonRef cutPoly = diff[0];
    for (unsigned int i = 0; i < cutPoly.size() && numSplitPoints < 2; ++i) {
        Point* cutPolyPt = &(cutPoly[i]);
        
        if (intersectingPoly.inside(*cutPolyPt, true) && !intersectingPoly.inside(*cutPolyPt, false)) {
            splitPoints[numSplitPoints] = cutPolyPt;
            ++numSplitPoints;
        }
        
    }
    
    switch (numSplitPoints) {
            // Error state
        case 0:
            log("[ERROR] (VolumeDecomposer::findSplitPoints) The cut face did not have any points in common with the intersecting polygon");
            return 0;
            break;
            // This is either:
            //	* A face parallel to the +z-axis
            //	* A non-parallel face that intersects with the intersecting polygon only by a single vertex
        case 1: {
            log("[INFO] In numSplitPoints = 1 case\n");
            Point splitPoint = *(splitPoints[0]);
            unsigned int numCommonVertices = 0;
            const MeshVertex* commonVertices[2];
            const MeshVertex* otherVertices[3];
            
            // Determine the vertices, if any, this point coincides with on the face
            if	((splitPoint.X <= v0.p.x + 1 && splitPoint.X >= v0.p.x - 1) &&
                 (splitPoint.Y <= v0.p.y + 1 && splitPoint.Y >= v0.p.y - 1)) {
                commonVertices[numCommonVertices] = &v0;
                numCommonVertices++;
            } else {
                otherVertices[0] = &v0;
            }
            
            if	((splitPoint.X <= v1.p.x + 1 && splitPoint.X >= v1.p.x - 1) &&
                 (splitPoint.Y <= v1.p.y + 1 && splitPoint.Y >= v1.p.y - 1)) {
                commonVertices[numCommonVertices] = &v1;
                numCommonVertices++;
            } else {
                if (numCommonVertices == 0) otherVertices[1] = &v1;
                else otherVertices[0] = &v1;
            }
            
            if	((splitPoint.X <= v2.p.x + 1 && splitPoint.X >= v2.p.x - 1) &&
                 (splitPoint.Y <= v2.p.y + 1 && splitPoint.Y >= v2.p.y - 1)) {
                if (numCommonVertices == 2) {
                    log("[ERROR] (VolumeDecomposer::findSplitPoints) The splitpoint is in common with all three of the face's vertices, this should be impossible. Ignoring third vertex and continuing.");
                } else {
                    commonVertices[numCommonVertices] = &v2;
                    numCommonVertices++;
                }
            } else {
                otherVertices[2 - numCommonVertices] = &v2;
            }
            
            log("[INFO] numCommonVertices = %d\n", numCommonVertices);
            // If there are 2 common vertices, that means this is a face parallel to the +z-axis
            // and the split is along one of the sides of the face
            if (numCommonVertices == 2) {
                result = make_pair(commonVertices[0]->p, commonVertices[1]->p);
                numSplitPoints = 2;
            }
            // If there is one common vertex, this is either a non-parallel face
            // that barely touches the polygon with one vertex, or a parallel face
            // that may coincide from the vertex to somewhere on an edge in the +z direction
            else if (numCommonVertices == 1) {
                result = make_pair(commonVertices[0]->p, Point3(0, 0, 0));
                bool isEdgeVertex = false;
                for (unsigned int i = 0; i < facePoly[0].size(); ++i) {
                    if ((splitPoint.X <= facePoly[0][i].X + 1 && splitPoint.X >= facePoly[0][i].X - 1) &&
                        (splitPoint.Y <= facePoly[0][i].Y + 1 && splitPoint.Y >= facePoly[0][i].Y - 1)) {
                        isEdgeVertex = true;
                    }
                }
                
                // If the single point is not one of the endpoints of the polygon, then this face is guaranteed
                // to be parallel to the +z-axis and the common vertex is in the middle of the flattened polyline,
                // therefore we find if the point is above or below the two other
                if (!isEdgeVertex) {
                    Point3 secondPoint;
                    if (findZValueOf2DPointon3DLine(otherVertices[0]->p, otherVertices[1]->p, Point(result.first.x, result.first.y), secondPoint)) {
                        result.second = secondPoint;
                        numSplitPoints = 2;
                    }
                }
            }
            // If there are no common vertices, this is a parallel face where the intersecting points
            // are in the middle of the triangle somewhere
            else {
                Point3 firstPoint;
                Point3 secondPoint;
                unsigned int numPointsFound = 0;
                result = make_pair(Point3(0, 0, 0), Point3(0, 0, 0));
                
                if (findZValueOf2DPointon3DLine(v0.p, v1.p, splitPoint, firstPoint)) {
                    result.first = firstPoint;
                    numPointsFound++;
                }
                if (findZValueOf2DPointon3DLine(v1.p, v2.p, splitPoint, (numPointsFound ? secondPoint : firstPoint))) {
                    if (numPointsFound) {
                        result.second = secondPoint;
                    } else {
                        result.first = firstPoint;
                    }
                    numPointsFound++;
                }
                
                if (numPointsFound < 2) {
                    if (findZValueOf2DPointon3DLine(v0.p, v2.p, splitPoint, secondPoint)) {
                        result.second = secondPoint;
                        numPointsFound++;
                    }
                }
                
                if (numPointsFound < 2) {
                    log("[ERROR] There were two split points but only %d 3D-points found to match them\n", numPointsFound);
                } else {
                    numSplitPoints = 2;
                }
            }
            break;
        }
        case 2:
            break;
        default:
            break;
    }
    
    // for (unsigned int i = 0; i < numSplitPoints; ++i) {
    // 	// log("Point #%d: <%d, %d>\n", i, splitPoints[i]->X, splitPoints[i]->Y);
    
    // 	if (i == 0) result.first = *(splitPoints[i]);
    // 	else result.second = *(splitPoints[i]);
    // }
    
    return numSplitPoints;
}

bool VolumeDecomposer::findZValueOf2DPointon3DLine(const Point3& P3_0, const Point3& P3_1, const Point& startPoint, Point3& resultPoint) {
    const Point flat_p0 = Point(P3_0.x, P3_0.y);
    const Point flat_p1 = Point(P3_1.x, P3_1.y);
    
    if (isOn(flat_p0, flat_p1, startPoint)) {
        const Point3 P3_vertexDiff = Point3(P3_1.x, P3_1.y, 0) - Point3(P3_0.x, P3_0.y, 0);
        const Point3 P3_intersectDiff = Point3(startPoint.X, startPoint.Y, 0) - Point3(P3_0.x, P3_0.y, 0);
        double t = (double)P3_intersectDiff.vSize() / (double)P3_vertexDiff.vSize();
        
        // int intersection_z = (1 - t) * P3_0.z + t * P3_1.z;
        int intersection_z = (P3_1.z - P3_0.z) * t + P3_0.z;
        
        resultPoint = Point3(startPoint.X, startPoint.Y, intersection_z);
        return true;
    }
    
    return false;
}

pair<bool, int> VolumeDecomposer::faceIsOverhangIntersect(Mesh& mesh, int faceID, Polygons& comparisonPolys) {
    const MeshFace& face = mesh.faces[faceID];
    const MeshVertex& v0 = mesh.vertices[face.vertex_index[0]];
    const MeshVertex& v1 = mesh.vertices[face.vertex_index[1]];
    const MeshVertex& v2 = mesh.vertices[face.vertex_index[2]];
    
    const Point p0 = Point(v0.p.x, v0.p.y);
    const Point p1 = Point(v1.p.x, v1.p.y);
    const Point p2 = Point(v2.p.x, v2.p.y);
    
    for (unsigned int poly_idx = 0; poly_idx < comparisonPolys.size(); ++poly_idx) {
        PolygonRef poly = comparisonPolys[poly_idx];
        bool p0_inside = poly.inside(p0, true);
        bool p1_inside = poly.inside(p1, true);
        bool p2_inside = poly.inside(p2, true);
        
        if (!((p0_inside && p1_inside && p2_inside) || (!p0_inside && !p1_inside && !p2_inside))) {
            return make_pair(true, poly_idx);
        }
    }
    
    return make_pair(false, -1);
}

FPoint3 VolumeDecomposer::findPolyFaceIntersection(Mesh& mesh, int faceID, PolygonRef intersectingPoly, int sliceHeight) {
    const MeshFace& face = mesh.faces[faceID];
    const MeshVertex& faceVert = mesh.vertices[face.vertex_index[0]];
    const FPoint3 faceNorm = faceNormal(mesh, face);
    
    double d, t;
    d = (faceNorm.x * faceVert.p.x) + (faceNorm.y * faceVert.p.y) + (faceNorm.z * faceVert.p.z);
    
    for (unsigned int i = 0; i < intersectingPoly.size(); ++i) {
        Point p1, p2;
        p1 = intersectingPoly[i];
        p2 = intersectingPoly[i + 1];
        
        int p1_x, p1_y, p1_z,
        p2_x, p2_y;
        p1_x = p1.X;
        p1_y = p1.Y;
        p1_z = sliceHeight;
        p2_x = p2.X;
        p2_y = p2.Y;
        
        int dX = p2_x - p1_x;
        int dY = p2_y - p1_y;
        int dZ = 0;
        
        t = (d - p1_x - p1_y - p1_z) / (dX + dY + dZ);
        
        if (t >= 0 && t <= 1) {
            double newX = p1_x + dX * t;
            double newY = p1_y + dY * t;
            double newZ = p1_z + dZ * t;
            FPoint3 intersectPoint = FPoint3(static_cast<float>(newX), static_cast<float>(newY), static_cast<float>(newZ));
            
            return intersectPoint;
        }
    }
    
    log("When trying to find the intersection between a face and a polygon, no intersection was found");
    exit(0);
}

string VolumeDecomposer::faceToString(Mesh & mesh, int id) {
    const MeshFace& face = mesh.faces[id];
    const MeshVertex& v0 = mesh.vertices[face.vertex_index[0]];
    const MeshVertex& v1 = mesh.vertices[face.vertex_index[1]];
    const MeshVertex& v2 = mesh.vertices[face.vertex_index[2]];
    string faceString = "v0: [";
    faceString += to_string(v0.p.x)  + ", " + to_string(v0.p.y) + ", " + to_string(v0.p.z) + "], v1: [" + to_string(v1.p.x) + ", " + to_string(v1.p.y) + ", " + to_string(v1.p.z) + "], v2: [" + to_string(v2.p.x) + ", " + to_string(v2.p.y) + ", " + to_string(v2.p.z) + "]";
    return faceString;
}

string VolumeDecomposer::polygonRefToString(const PolygonRef& pref) {
    string ret = "Polygon points:\n";
    for (unsigned int i = 0; i < pref.size(); ++i) {
        const Point p = pref[i];
        
        ret += "\t#" + to_string(i) + ": <" + to_string(p.X) + ", " + to_string(p.Y) + ">";
        if (i < pref.size() - 1) {
            ret += "\n";
        }
    }
    return ret;
}
