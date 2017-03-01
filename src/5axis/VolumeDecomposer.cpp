#include "VolumeDecomposer.hpp"

#include <algorithm>

#include "../utils/polygon.h"
#include "../utils/intpoint.h"
#include "comms/SerialComms.hpp"
#include "MeshToSTL.hpp"
#include "Utility.hpp"
#include "BuildMap.hpp"

using namespace std;
namespace cura {
    
    VolumeDecomposer::VolumeDecomposer(Mesh& mesh) {
        decompose(mesh, true);
    }
    
    void VolumeDecomposer::decompose(Mesh& mesh, bool first){
        
        if(first){
            SeqNode parentNode = SeqNode(mesh);
            sequenceGraph.addNode(parentNode);
        }
        long int parentIndex = sequenceGraph.size()-1;
        
        int model_max = mesh.getAABB().max.z;
        long long int initial_layer_thickness = mesh.getSettingInMicrons("layer_height_0");
        long long int layer_thickness = mesh.getSettingInMicrons("layer_height");
        long long int initial_slice_z = initial_layer_thickness - layer_thickness / 2;
        long long int slice_layer_count = (model_max - initial_slice_z) / layer_thickness + 1;
        
        Slicer* slicer = new Slicer(&mesh, initial_slice_z, layer_thickness, slice_layer_count, mesh.getSettingBoolean("meshfix_keep_open_polygons"), mesh.getSettingBoolean("meshfix_extensive_stitching"));
        
        
        // SerialComms sc = SerialComms("/dev/ttyACM0");
        std::vector<SlicerLayer> & layers = slicer->layers;
        
        // Initialize all of our comparison vars
        SlicerLayer & comparisonSlice = layers[0];
        Polygons & comparisonPolys = comparisonSlice.polygons;
        
        unsigned long int numLayers = layers.size();
        log("Progress: [", 0);
        
        for (unsigned int layer_idx = 1; layer_idx < layers.size(); ++layer_idx) {
            SlicerLayer & slice = layers[layer_idx];
            Polygons & polys = slice.polygons;
            Polygons & openPolys = slice.openPolylines;
            std::vector<std::vector<int>> polyFaces = slice.polyFaces;
            
            if ((int)(100.0 * ((double)layer_idx / (double)(numLayers - 1))) % 5 == 0) {
                log("=");
            }
            /*
             // Main loop
             for (unsigned int polyfaces_idx = 0; polyfaces_idx < polyFaces.size(); ++polyfaces_idx) {
             std::vector<int> faces = polyFaces[polyfaces_idx];
             bool intersectingOverhang = faceIsOverhangIntersect(mesh, faceID, comparisonPolys[comparisonPolys_idx]);
             if (intersectingOverhang) {
             std::pair<Point3, Point3> splitPoints;
             int numSplitPoints = findSplitPoints(mesh, faceID, comparisonPolys[comparisonPolys_idx], splitPoints);
             log("[INFO] numSplitPoints = %d, <%d, %d, %d>, <%d, %d, %d>\n", numSplitPoints, splitPoints.first.x, splitPoints.first.y, splitPoints.first.z, splitPoints.second.x, splitPoints.second.y, splitPoints.second.z);
             splitFace(mesh, faceID, numSplitPoints, splitPoints);
             }
             }
             }
             }
             
             comparisonPolys = polys;
             */
        }
        
        //Creating the graph by recursivley decomposing meshes
        
        //TODO:get these from decomp
        std::vector<int> seeds;
        seeds.push_back(19);
        seeds.push_back(18);
        
        
        MeshSequence sub_graph = separateMesh(mesh, seeds);
        for( Mesh child : sub_graph.children){
            SeqNode childNode = SeqNode(child);
            BuildMap buildmap = BuildMap(mesh);
            FPoint3 buildVector = buildmap.findBestVector();
            
            sequenceGraph.addNode(childNode);
            
            long int childIndex = sequenceGraph.size()-1;
            sequenceGraph.addGeometricChild(parentIndex, childIndex);
            
            //decompose(child, false);
            //call volume decomp
        }
        
        //set the parent node mesh to be the parent of the output of mesh separation
        sequenceGraph.graphNodes[parentIndex].mesh = sub_graph.parent;
    }
    
    int VolumeDecomposer::splitFaces(Mesh& mesh, int faceID, PolygonRef intersectingPoly, pair<Point3, Point3> splitPoints) {
        log("[INFO] Starting splitFaces() on face %d\n", faceID);
        
        vector<pair<Point3, Point3>> splitPointsVector;
        
        int seedVertex = -1;
        
        Point3 prevSplitPoint;
        int prevSplitPointIndex;
        int prevSplitPointPrimeIndex;
        bool pointOnPrevFirstEdge = true;
        tuple<int, int, int> prevFaceIDs;
        int prevFaceCase = 0;
        
        tuple<int, int, int> firstFaceIDs;
        
        bool firstCycle = true;
        int firstFaceCase = 0;
        
        /*
         case I: neither split point is on a vertex
         case II: exactly one split points is on a vertex
         case III: both split point are on the same vertex
         case IV: each split point is on a different vertex
         */
        
        while (true) {
            // Grab the actual MeshFace
            MeshFace& face = mesh.faces[faceID];
            
            log("[INFO] --Splitting face %d--\n", faceID);
            log("[INFO] Vertex[0]: <%d, %d, %d>\n", mesh.vertices[face.vertex_index[0]].p.x, mesh.vertices[face.vertex_index[0]].p.y, mesh.vertices[face.vertex_index[0]].p.z);
            log("[INFO] Vertex[1]: <%d, %d, %d>\n", mesh.vertices[face.vertex_index[1]].p.x, mesh.vertices[face.vertex_index[1]].p.y, mesh.vertices[face.vertex_index[1]].p.z);
            log("[INFO] Vertex[2]: <%d, %d, %d>\n", mesh.vertices[face.vertex_index[2]].p.x, mesh.vertices[face.vertex_index[2]].p.y, mesh.vertices[face.vertex_index[2]].p.z);
            
            log("[INFO] Split point[0]: <%d, %d, %d>\n", splitPoints.first.x, splitPoints.first.y, splitPoints.first.z);
            log("[INFO] Split point[1]: <%d, %d, %d>\n", splitPoints.second.x, splitPoints.second.y, splitPoints.second.z);
            
            log("[INFO] Adjacent Face[0]: %d\n", face.connected_face_index[0]);
            log("[INFO] Adjacent Face[1]: %d\n", face.connected_face_index[1]);
            log("[INFO] Adjacent Face[2]: %d\n", face.connected_face_index[2]);
            
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
            
            if ((splitPointVertexIntersectionIndices.first >= 0) && (splitPointVertexIntersectionIndices.second >= 0)) { //case III or case IV
                
                if (splitPointVertexIntersectionIndices.first == splitPointVertexIntersectionIndices.second) { //case III
                    
                    log("[INFO] Case III detected\n", faceID);
                    
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
                        log("[ERROR] Found case II with matching split points not on a vertex\n");
                        return seedVertex;
                    }
                    
                    vector<uint32_t>::iterator it; //index of faceID in vector
                    
                    if (firstCycle) {
                        if (intersectingPoly.inside(Point(mesh.vertices[face.vertex_index[y]].p.x, mesh.vertices[face.vertex_index[y]].p.y))) { //face is inside overhang, this should not be our first face
                            //go to the xth face and rerun cycle to see if face is not entirely on overhang
                            faceID = face.connected_face_index[x];
                            
                            splitPointsVector.clear();
                            findSplitPoints(mesh, faceID, intersectingPoly, splitPointsVector);
                            splitPoints = splitPointsVector[0];
                            
                            continue;
                        }
                        
                        firstFaceCase = 3;
                        
                        prevSplitPointIndex = face.vertex_index[x];
                        
                        mesh.vertices.push_back(MeshVertex(mesh.vertices[face.vertex_index[x]].p));
                        prevSplitPointPrimeIndex = mesh.vertices.size() - 1;
                        
                        firstFaceIDs = tuple<int, int, int>(faceID, -1, -1);
                        
                        seedVertex = prevSplitPointPrimeIndex;
                    }
                    
                    //remove original face from xth vertex
                    it = find(mesh.vertices[originalVertexIndices[x]].connected_faces.begin(), mesh.vertices[originalVertexIndices[x]].connected_faces.end(), faceID);
                    if (it != mesh.vertices[originalVertexIndices[x]].connected_faces.end()) {
                        mesh.vertices[originalVertexIndices[x]].connected_faces.erase(it);
                    }
                    
                    //add face to overhang vertex
                    mesh.vertices[prevSplitPointPrimeIndex].connected_faces.push_back(faceID);
                    
                    //reset variables
                    int tempFaceID = faceID;
                    prevFaceCase = 3;
                    
                    if (firstCycle) {
                        firstCycle = false;
                        faceID = face.connected_face_index[x];
                    } else {
                        if ((face.connected_face_index[x] == get<0>(prevFaceIDs)) || (face.connected_face_index[x] == get<1>(prevFaceIDs))) {
                            faceID = face.connected_face_index[z];
                        } else if ((face.connected_face_index[z] == get<0>(prevFaceIDs)) || (face.connected_face_index[z] == get<1>(prevFaceIDs))) {
                            faceID = face.connected_face_index[x];
                        } else {
                            log("[ERROR] Neither adjacent face matches previous face\n");
                            return seedVertex;
                        }
                    }
                    pointOnPrevFirstEdge = true;
                    
                    prevFaceIDs = tuple<int, int, int>(tempFaceID, -1, -1);
                    
                    if (faceID == get<0>(firstFaceIDs)) {
                        if ((firstFaceCase == 2) || (firstFaceCase == 3) || (firstFaceCase == 4)) {
                            return seedVertex; //nothing more needs to be done here
                        } else {
                            log("[ERROR] splitFaces() ended on face that is not a case II/III/IV from a case III face\n");
                            return seedVertex;
                        }
                    } else if (faceID == get<1>(firstFaceIDs)) {
                        if (firstFaceCase == 2) {
                            return seedVertex; //nothing more needs to be done here
                        } else {
                            log("[ERROR] splitFaces() ended on face that is not a case II from a case III face\n");
                            return seedVertex;
                        }
                    }
                    
                    splitPointsVector.clear();
                    findSplitPoints(mesh, faceID, intersectingPoly, splitPointsVector);
                    splitPoints = splitPointsVector[0];
                    
                } else { //case IV
                    
                    log("[INFO] Case IV detected\n", faceID);
                    
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
                        log("[ERROR] Split points are on impossible order of edges: (%d:%d)\n", splitPointVertexIntersectionIndices.first, splitPointVertexIntersectionIndices.second);
                        return seedVertex;
                    }
                    
                    //whether or not the first split point is the split point from the previous face
                    bool firstPointMatch;
                    
                    pair<int, int> newVertexIndices, newVertexPrimeIndices; //indices of vertices in mesh (prime are the vertices on the overhang)
                    
                    if (firstCycle) {
                        if (intersectingPoly.inside(Point(mesh.vertices[face.vertex_index[z]].p.x, mesh.vertices[face.vertex_index[z]].p.y))) { //if face is entirely not in overhang, this should not be our first face
                            faceID = face.connected_face_index[x];
                            
                            splitPointsVector.clear();
                            findSplitPoints(mesh, faceID, intersectingPoly, splitPointsVector);
                            splitPoints = splitPointsVector[0];
                            
                            continue;
                        }
                        
                        firstCycle = true;
                        firstFaceCase = 4;
                        
                        firstPointMatch = true;
                        
                        mesh.vertices.push_back(MeshVertex(mesh.vertices[face.vertex_index[x]].p));
                        mesh.vertices.push_back(MeshVertex(mesh.vertices[face.vertex_index[y]].p));
                        newVertexPrimeIndices = pair<int, int>(mesh.vertices.size() - 2, mesh.vertices.size() - 1);
                        
                        prevSplitPointIndex = face.vertex_index[x];
                        prevSplitPointPrimeIndex = mesh.vertices.size() - 1;
                        
                        firstFaceIDs = tuple<int, int, int>(faceID, -1, -1);
                        
                        seedVertex = prevSplitPointPrimeIndex;
                    } else {
                        if (Point3Equals(splitPoints.first, prevSplitPoint, 1)) { //TODO temp hack fix
                            firstPointMatch = true;
                            
                            mesh.vertices.push_back(MeshVertex(splitPoints.second));
                            newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.vertices.size() - 1); //indexes of new vertices in mesh
                        } else if (Point3Equals(splitPoints.second, prevSplitPoint, 1)) { //TODO temp hack fix
                            firstPointMatch = false;
                            
                            mesh.vertices.push_back(MeshVertex(splitPoints.first));
                            newVertexPrimeIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointPrimeIndex); //indexes of new vertices in mesh
                        } else {
                            log("[ERROR] Adjacent face to split face does not have a matching split point\n");
                            return seedVertex;
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
                        log("[ERROR] Adjacent face does not list current face as an adjacent face\n");
                        return seedVertex;
                    }
                    
                    //remove adjacent face on other side of split from face
                    face.connected_face_index[x] = -1;
                    
                    //reset variables
                    prevFaceCase = 4;
                    prevFaceIDs = tuple<int, int, int>(faceID, -1, -1);
                    if (firstPointMatch) {
                        faceID = face.connected_face_index[y];
                        prevSplitPoint = splitPoints.second;
                        prevSplitPointIndex = originalVertexIndices[y];
                        prevSplitPointPrimeIndex = newVertexPrimeIndices.second;
                    } else {
                        faceID = face.connected_face_index[z];
                        prevSplitPoint = splitPoints.first;
                        prevSplitPointIndex = originalVertexIndices[x];
                        prevSplitPointPrimeIndex = newVertexPrimeIndices.first;
                    }
                    pointOnPrevFirstEdge = true;
                    
                    if (faceID == get<0>(firstFaceIDs)) {
                        if ((firstFaceCase == 2) || (firstFaceCase == 3) || (firstFaceCase == 4)) {
                            return seedVertex; //nothing more needs to be done here
                        } else {
                            log("[ERROR] splitFaces() ended on face that is not a case II/III/IV from a case IV face\n");
                            return seedVertex;
                        }
                    } else if (faceID == get<1>(firstFaceIDs)) {
                        if (firstFaceCase == 2) {
                            return seedVertex; //nothing more needs to be done here
                        } else {
                            log("[ERROR] splitFaces() ended on face that is not a case II from a case III face\n");
                            return seedVertex;
                        }
                    }
                    
                    splitPointsVector.clear();
                    findSplitPoints(mesh, faceID, intersectingPoly, splitPointsVector);
                    splitPoints = splitPointsVector[0];
                    
                }
                
            } else if ((splitPointVertexIntersectionIndices.first >= 0) || (splitPointVertexIntersectionIndices.second >= 0)) { //case II
                
                //the xth vertex is always the vertex being intersected by the split point
                //the first face ID is the face with the yth vertex
                //the second face ID is the face with the zth vertex
                
                log("[INFO] Case II detected\n", faceID);
                
                //whether or not the first split point is the split point from the previous face
                bool firstPointMatch = true;
                
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
                    log("[ERROR] Split points vertex intersection index is not 0, 1, or 2\n");
                    return seedVertex;
                }
                
                pair<int, int> newVertexIndices(-1, -1);
                pair<int, int> newVertexPrimeIndices(-1, -1);
                
                mesh.faces.push_back(MeshFace());
                int newFaceID = mesh.faces.size() - 1;
                MeshFace& newFace = mesh.faces[newFaceID];
                
                if (firstCycle) {
                    switched = false; //regardless of if the split points were switched, treat them as if they weren't for the first time
                    firstCycle = false;
                    firstFaceCase = 2;
                    
                    firstFaceIDs = tuple<int, int, int>(faceID, newFaceID, -1);
                    
                    mesh.vertices.push_back(MeshVertex(splitPoints.second));
                    newVertexIndices = pair<int, int>(face.vertex_index[x], mesh.vertices.size() - 1);
                    
                    mesh.vertices.push_back(MeshVertex(splitPoints.first));
                    mesh.vertices.push_back(MeshVertex(splitPoints.second));
                    newVertexPrimeIndices = pair<int, int>(mesh.vertices.size() - 2, mesh.vertices.size() - 1);
                    
                    seedVertex = newVertexPrimeIndices.first;
                    
                } else {
                    if (splitPoints.first == prevSplitPoint) {
                        firstPointMatch = true;
                    } else if (splitPoints.second == prevSplitPoint) {
                        firstPointMatch = false;
                    } else {
                        log("[ERROR] Adjacent face to split face does not have a matching split point to previous split point <%d, %d, %d>\n", prevSplitPoint.x, prevSplitPoint.y, prevSplitPoint.z);
                        return seedVertex;
                    }
                    
                    if (switched) {
                        mesh.vertices.push_back(MeshVertex(splitPoints.first));
                        newVertexIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointIndex);
                        
                        mesh.vertices.push_back(MeshVertex(splitPoints.first));
                        newVertexPrimeIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointPrimeIndex);
                    } else {
                        mesh.vertices.push_back(MeshVertex(splitPoints.second));
                        newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.vertices.size() - 1);
                        
                        mesh.vertices.push_back(MeshVertex(splitPoints.second));
                        newVertexIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.vertices.size() - 1);
                    }
                }
                
                //set 2nd adjacent face to adjacent face of first face
                newFace.connected_face_index[2] = face.connected_face_index[z];
                if (mesh.faces[newFace.connected_face_index[2]].connected_face_index[0] == faceID) {
                    mesh.faces[newFace.connected_face_index[2]].connected_face_index[0] = newFaceID;
                } else if (mesh.faces[newFace.connected_face_index[2]].connected_face_index[1] == faceID) {
                    mesh.faces[newFace.connected_face_index[2]].connected_face_index[1] = newFaceID;
                } else if (mesh.faces[newFace.connected_face_index[2]].connected_face_index[2] == faceID) {
                    mesh.faces[newFace.connected_face_index[2]].connected_face_index[2] = newFaceID;
                }
                
                //remove new face from adjacency list of first face
                face.connected_face_index[z] = -1;
                
                bool yVertexInOverhang = !intersectingPoly.inside(Point(mesh.vertices[face.vertex_index[y]].p.x, mesh.vertices[face.vertex_index[y]].p.y));
                
                if (yVertexInOverhang) {
                    newFace.vertex_index[0] = newVertexPrimeIndices.first;
                    newFace.vertex_index[1] = newVertexPrimeIndices.second;
                    newFace.vertex_index[2] = face.vertex_index[z];
                    
                    face.vertex_index[z] = newVertexIndices.second;
                } else {
                    newFace.vertex_index[0] = newVertexIndices.first;
                    newFace.vertex_index[1] = newVertexIndices.second;
                    newFace.vertex_index[2] = face.vertex_index[z];
                    
                    face.vertex_index[z] = newVertexPrimeIndices.second;
                }
                
                //connect previous faces
                if (switched) { //only needs to be done is previous split point was on edge
                    if (prevFaceCase == 1) {
                        if (pointOnPrevFirstEdge) {
                            //connect A and prevA
                            face.connected_face_index[y] = get<0>(prevFaceIDs);
                            //prevA already has A as it's adjacent face
                            
                            //connect B and prevB
                            int prevFaceIDsSecond = get<1>(prevFaceIDs);
                            newFace.connected_face_index[1] = prevFaceIDsSecond;
                            if (mesh.faces[prevFaceIDsSecond].connected_face_index[0] == faceID) {
                                mesh.faces[prevFaceIDsSecond].connected_face_index[0] = newFaceID;
                            } else if (mesh.faces[prevFaceIDsSecond].connected_face_index[1] == faceID) {
                                mesh.faces[prevFaceIDsSecond].connected_face_index[1] = newFaceID;
                            } else if (mesh.faces[prevFaceIDsSecond].connected_face_index[2] == faceID) {
                                mesh.faces[prevFaceIDsSecond].connected_face_index[2] = newFaceID;
                            }
                        } else {
                            //connect A and prevC
                            face.connected_face_index[y] = get<2>(prevFaceIDs);
                            mesh.faces[get<2>(prevFaceIDs)].connected_face_index[2] = faceID;
                            
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
                            
                        }
                    } else if (prevFaceCase == 2) {
                        //connect A and prevB
                        face.connected_face_index[y] = get<1>(prevFaceIDs);
                        mesh.faces[get<1>(prevFaceIDs)].connected_face_index[1] = faceID;
                    } else {
                        log("[ERROR] Case II has previous split point on edge from neither a case I or a case II\n");
                        return seedVertex;
                    }
                }
                
                //reset variables
                prevFaceCase = 2;
                prevFaceIDs = tuple<int, int, int>(faceID, newFaceID, -1);
                pointOnPrevFirstEdge = !switched;
                
                
                if (firstPointMatch) {
                    faceID = face.connected_face_index[y];
                    prevSplitPoint = splitPoints.second;
                    prevSplitPointIndex = newVertexIndices.second;
                    prevSplitPointPrimeIndex = newVertexPrimeIndices.second;
                } else {
                    if (yVertexInOverhang) {
                        faceID = newFace.connected_face_index[2];
                    } else {
                        faceID = face.connected_face_index[x];
                    }
                    
                    prevSplitPoint = splitPoints.first;
                    prevSplitPointIndex = newVertexIndices.first;
                    prevSplitPointPrimeIndex = newVertexPrimeIndices.first;
                }
                
                splitPointsVector.clear();
                findSplitPoints(mesh, faceID, intersectingPoly, splitPointsVector);
                splitPoints = splitPointsVector[0];
                
            } else { //case I
                
                //the face is always split so the side that is a triangle is left as the original face in the face array
                //the trapezoid side is always split (from the perspective of the larger base being on the bottom) from top-left to bottom-right
                //triangle side is always first ID, top triangle of trapezoid is second, and bottom triangle of trapezoid is third (regardless of which side is the overhang)
                
                log("[INFO] Case I detected\n", faceID);
                
                
                //determine which edges split points are on
                pair<int, int> splitPointEdgeIndices(-1, -1);
                
                FPoint3 edges[3];
                edges[0] = mesh.vertices[face.vertex_index[1]].p - mesh.vertices[face.vertex_index[0]].p;
                edges[1] = mesh.vertices[face.vertex_index[2]].p - mesh.vertices[face.vertex_index[1]].p;
                edges[2] = mesh.vertices[face.vertex_index[0]].p - mesh.vertices[face.vertex_index[2]].p;
                
                log("[INFO] Cross product of first split point and edge 0: %f\n", (edges[0].cross(splitPoints.first - mesh.vertices[face.vertex_index[0]].p).vSize2()));
                log("[INFO] Cross product of first split point and edge 1: %f\n", (edges[1].cross(splitPoints.first - mesh.vertices[face.vertex_index[1]].p).vSize2()));
                log("[INFO] Cross product of first split point and edge 2: %f\n", (edges[2].cross(splitPoints.first - mesh.vertices[face.vertex_index[2]].p).vSize2()));
                
                log("[INFO] Cross product of second split point and edge 0: %f\n", (edges[0].cross(splitPoints.second - mesh.vertices[face.vertex_index[0]].p).vSize2()));
                log("[INFO] Cross product of second split point and edge 1: %f\n", (edges[1].cross(splitPoints.second - mesh.vertices[face.vertex_index[1]].p).vSize2()));
                log("[INFO] Cross product of second split point and edge 2: %f\n", (edges[2].cross(splitPoints.second - mesh.vertices[face.vertex_index[2]].p).vSize2()));
                
                if (fabs(edges[0].cross(splitPoints.first - mesh.vertices[face.vertex_index[0]].p).vSize2()) <= 0.01) { //allow for tolerance
                    splitPointEdgeIndices.first = 0;
                } else if (fabs(edges[1].cross(splitPoints.first - mesh.vertices[face.vertex_index[1]].p).vSize2()) <= 0.01) {
                    splitPointEdgeIndices.first = 1;
                } else if (fabs(edges[2].cross(splitPoints.first - mesh.vertices[face.vertex_index[2]].p).vSize2()) <= 0.01) {
                    splitPointEdgeIndices.first = 2;
                } else {
                    log("[ERROR] First split point was not found to be on any edge of face\n");
                    return seedVertex;
                }
                
                if (fabs(edges[0].cross(splitPoints.second - mesh.vertices[face.vertex_index[0]].p).vSize2()) <= 0.01) {
                    splitPointEdgeIndices.second = 0;
                } else if (fabs(edges[1].cross(splitPoints.second - mesh.vertices[face.vertex_index[1]].p).vSize2()) <= 0.01) {
                    splitPointEdgeIndices.second = 1;
                } else if (fabs(edges[2].cross(splitPoints.second - mesh.vertices[face.vertex_index[2]].p).vSize2()) <= 0.01) {
                    splitPointEdgeIndices.second = 2;
                } else {
                    log("[ERROR] Second split point was not found to be on any edge of face\n");
                    return seedVertex;
                }
                
                if (splitPointEdgeIndices.first == splitPointEdgeIndices.second) {
                    log("[ERROR] Split points are on same edge\n");
                    return seedVertex;
                }
                
                //if order is either 1-0, 2-0, or 2-1 switch it to 0-1, 0-2, or 1-2 repsectively so there's only 3 scenarios
                bool switched = false;
                if ((splitPointEdgeIndices.first == 1) && (splitPointEdgeIndices.second == 0)) {
                    splitPointEdgeIndices = pair<int, int>(0, 1);
                    switched = true;
                } else if ((splitPointEdgeIndices.first == 2) && (splitPointEdgeIndices.second == 1)) {
                    splitPointEdgeIndices = pair<int, int>(1, 2);
                    switched = true;
                } else if ((splitPointEdgeIndices.first == 0) && (splitPointEdgeIndices.second == 2)) {
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
                    log("[ERROR] Split points are on impossible order of edges: (%d:%d)\n", splitPointEdgeIndices.first, splitPointEdgeIndices.second);
                    return seedVertex;
                }
                
                //if this is the last triangle before return to original face where cut was started
                bool isFinalFaceInCycle = (((face.connected_face_index[x] == get<0>(firstFaceIDs)) || (face.connected_face_index[y] == get<0>(firstFaceIDs))) && (get<0>(prevFaceIDs) != get<0>(firstFaceIDs))); //oh shit it's OG!!
                if (isFinalFaceInCycle) {
                    log("[INFO] Last face is split face cycle\n");
                }
                
                //whether or not the trapezoid created by the cut is in the overhang
                bool trapezoidInOverhang = intersectingPoly.inside(face.vertex_index[y]); //y vertex is connected to base volume
                
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
                } else if (isFinalFaceInCycle) {
                    if (Point3Equals(splitPoints.first, prevSplitPoint, 1)) { //TODO temp hack fix
                        firstPointMatch = true;
                        
                        if (trapezoidInOverhang) {
                            newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.faces[get<0>(firstFaceIDs)].vertex_index[2] - 2); //indexes of new vertices in mesh
                            newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.faces[get<0>(firstFaceIDs)].vertex_index[2]); //indexes of new vertices in mesh
                        } else {
                            newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.faces[get<0>(firstFaceIDs)].vertex_index[2]); //indexes of new vertices in mesh
                            newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.faces[get<0>(firstFaceIDs)].vertex_index[2] + 2); //indexes of new vertices in mesh
                        }
                    } else if (Point3Equals(splitPoints.second, prevSplitPoint, 1)) { //TODO temp hack fix
                        firstPointMatch = false;
                        
                        if (trapezoidInOverhang) {
                            newVertexIndices = pair<int, int>(mesh.faces[get<0>(firstFaceIDs)].vertex_index[2], prevSplitPointIndex); //indexes of new vertices in mesh
                            newVertexPrimeIndices = pair<int, int>(mesh.faces[get<0>(firstFaceIDs)].vertex_index[2] + 2, prevSplitPointPrimeIndex); //indexes of new vertices in mesh
                        } else {
                            newVertexIndices = pair<int, int>(mesh.faces[get<0>(firstFaceIDs)].vertex_index[2] - 2, prevSplitPointIndex); //indexes of new vertices in mesh
                            newVertexPrimeIndices = pair<int, int>(mesh.faces[get<0>(firstFaceIDs)].vertex_index[2], prevSplitPointPrimeIndex); //indexes of new vertices in mesh
                        }
                    } else {
                        log("[ERROR] Adjacent face to split face does not have a matching split point to previous split point <%d, %d, %d>\n", prevSplitPoint.x, prevSplitPoint.y, prevSplitPoint.z);
                        return seedVertex;
                    }
                } else {
                    if (Point3Equals(splitPoints.first, prevSplitPoint, 1)) { //TODO temp hack fix
                        firstPointMatch = true;
                        
                        //add split points to vertex list
                        mesh.vertices.push_back(MeshVertex(splitPoints.second));
                        newVertexIndices = pair<int, int>(prevSplitPointIndex, mesh.vertices.size() - 1); //indexes of new vertices in mesh
                        
                        mesh.vertices.push_back(MeshVertex(splitPoints.second));
                        newVertexPrimeIndices = pair<int, int>(prevSplitPointPrimeIndex, mesh.vertices.size() - 1); //indexes of new vertices in mesh
                    } else if (Point3Equals(splitPoints.second, prevSplitPoint, 1)) { //TODO temp hack fix
                        firstPointMatch = false;
                        
                        mesh.vertices.push_back(MeshVertex(splitPoints.first));
                        newVertexIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointIndex); //indexes of new vertices in mesh
                        
                        mesh.vertices.push_back(MeshVertex(splitPoints.first));
                        newVertexPrimeIndices = pair<int, int>(mesh.vertices.size() - 1, prevSplitPointPrimeIndex); //indexes of new vertices in mesh
                    } else {
                        log("[ERROR] Adjacent face to split face does not have a matching split point to previous split point <%d, %d, %d>\n", prevSplitPoint.x, prevSplitPoint.y, prevSplitPoint.z);
                        return seedVertex;
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
                    log("[ERROR] Original face is not listed as an adjacent face of adjacent face\n");
                    return seedVertex;
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
                    firstFaceIDs = prevFaceIDs = tuple<int, int, int>(faceID, newFaceIDs.first, newFaceIDs.second);
                    prevSplitPointIndex = newVertexIndices.first;
                    prevSplitPointPrimeIndex = newVertexPrimeIndices.first;
                    
                    pointOnPrevFirstEdge = true;
                    faceID = face.connected_face_index[x];
                    
                    splitPointsVector.clear();
                    findSplitPoints(mesh, faceID, intersectingPoly, splitPointsVector);
                    splitPoints = splitPointsVector[0];
                    
                    seedVertex = prevSplitPointPrimeIndex;
                    
                } else { //connect to previous faces
                    if (prevFaceCase == 1) {
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
                    } else if (prevFaceCase == 2) {
                        if (firstPointMatch) {
                            //connect A and prevA
                            face.connected_face_index[x] = get<0>(prevFaceIDs);
                            //prevA already has A as it's adjacent face
                            
                            //connect B and prevB
                            int prevFaceIDsSecond = get<1>(prevFaceIDs);
                            newFaces.first.connected_face_index[0] = prevFaceIDsSecond;
                            if (mesh.faces[prevFaceIDsSecond].connected_face_index[0] == faceID) {
                                mesh.faces[prevFaceIDsSecond].connected_face_index[0] = newFaceIDs.first;
                            } else if (mesh.faces[prevFaceIDsSecond].connected_face_index[1] == faceID) {
                                mesh.faces[prevFaceIDsSecond].connected_face_index[1] = newFaceIDs.first;
                            } else if (mesh.faces[prevFaceIDsSecond].connected_face_index[2] == faceID) {
                                mesh.faces[prevFaceIDsSecond].connected_face_index[2] = newFaceIDs.first;
                            }
                        } else {
                            //connect A and prevB
                            face.connected_face_index[y] = get<1>(prevFaceIDs);
                            //prevB already has A as it's adjacent face
                            
                            //connect C and prevA
                            int prevFaceIDsFirst = get<0>(prevFaceIDs);
                            newFaces.second.connected_face_index[2] = prevFaceIDsFirst;
                            if (mesh.faces[prevFaceIDsFirst].connected_face_index[0] == faceID) {
                                mesh.faces[prevFaceIDsFirst].connected_face_index[0] = newFaceIDs.second;
                            } else if (mesh.faces[prevFaceIDsFirst].connected_face_index[1] == faceID) {
                                mesh.faces[prevFaceIDsFirst].connected_face_index[1] = newFaceIDs.second;
                            } else if (mesh.faces[prevFaceIDsFirst].connected_face_index[2] == faceID) {
                                mesh.faces[prevFaceIDsFirst].connected_face_index[2] = newFaceIDs.second;
                            }
                        }
                    } else {
                        log("[ERROR] Reached a case I when splitting faces from neither a case I or a case II\n");
                        return seedVertex;
                    }
                    
                    if (isFinalFaceInCycle) {
                        if (firstFaceCase == 1) {
                            if (firstPointMatch) {
                                if (trapezoidInOverhang) {
                                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<1>(firstFaceIDs));
                                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<2>(firstFaceIDs));
                                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<0>(firstFaceIDs));
                                } else {
                                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<1>(firstFaceIDs));
                                    mesh.vertices[newVertexPrimeIndices.second].connected_faces.push_back(get<2>(firstFaceIDs));
                                    mesh.vertices[newVertexIndices.second].connected_faces.push_back(get<0>(firstFaceIDs));
                                }
                                
                                //connect A to originalC
                                face.connected_face_index[y] = get<2>(firstFaceIDs);
                                mesh.faces[get<2>(firstFaceIDs)].connected_face_index[2] = faceID;
                                
                                //connect C to originalA
                                newFaces.second.connected_face_index[2] = get<0>(firstFaceIDs);
                                mesh.faces[get<0>(firstFaceIDs)].connected_face_index[1] = newFaceIDs.second;
                                
                            } else {
                                if (trapezoidInOverhang) {
                                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<0>(firstFaceIDs));
                                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<1>(firstFaceIDs));
                                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<2>(firstFaceIDs));
                                } else {
                                    mesh.vertices[newVertexPrimeIndices.first].connected_faces.push_back(get<0>(firstFaceIDs));
                                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<1>(firstFaceIDs));
                                    mesh.vertices[newVertexIndices.first].connected_faces.push_back(get<2>(firstFaceIDs));
                                }
                                
                                //connect A to connectA
                                face.connected_face_index[x] = get<0>(firstFaceIDs);
                                mesh.faces[get<0>(firstFaceIDs)].connected_face_index[0] = faceID;
                                
                                //conect B to connectC
                                newFaces.first.connected_face_index[0] = get<2>(firstFaceIDs);
                                mesh.faces[get<2>(firstFaceIDs)].connected_face_index[0] = newFaceIDs.first;
                            }
                        } else {
                            log("[ERROR] Case I face in splitFaces() is reaching final face that is not a case I\n");
                            return seedVertex;
                        }
                        
                        return seedVertex;
                    } else {
                        
                        //reset variables
                        prevFaceCase = 1;
                        prevFaceIDs = tuple<int, int, int>(faceID, newFaceIDs.first, newFaceIDs.second);
                        
                        if (firstPointMatch) {
                            faceID = face.connected_face_index[y];
                            
                            prevSplitPoint = splitPoints.second;
                            prevSplitPointIndex = newVertexIndices.second;
                            prevSplitPointPrimeIndex = newVertexPrimeIndices.second;
                            pointOnPrevFirstEdge = true;
                        } else {
                            faceID = face.connected_face_index[x];
                            
                            prevSplitPoint = splitPoints.first;
                            prevSplitPointIndex = newVertexIndices.first;
                            prevSplitPointPrimeIndex = newVertexPrimeIndices.first;
                            pointOnPrevFirstEdge = false;
                        }
                        
                        splitPointsVector.clear();
                        findSplitPoints(mesh, faceID, intersectingPoly, splitPointsVector);
                        splitPoints = splitPointsVector[0];
                    }
                }
            }
        }
    }
    
    bool VolumeDecomposer::isOn(Point a, Point b, Point c, unsigned int tolerance) {
        // Return true iff point c intersects the line segment from a to b.
        // (or the degenerate case that all 3 points are coincident)
        if (!collinear(a, b, c, tolerance)) return false;
        
        if (a.X != b.X) {
            return within(a.X, c.X, b.X);
        } else if (a.Y != b.Y) {
            return within(a.Y, c.Y, b.Y);
        } else {
            return false;
        }
    }
    
    bool VolumeDecomposer::collinear(Point a, Point b, Point c, unsigned int tolerance) {
        // Return true iff a, b, and c all lie on the same line.
        int left = abs((b.X - a.X) * (c.Y - a.Y));
        int right = abs((c.X - a.X) * (b.Y - a.Y));
        
        // log("SANITY CHECK = (%d - %d) * (%d - %d) = %llu\n", b.X, a.X, c.Y, a.Y, (uint64_t)abs((b.X - a.X) * (c.Y - a.Y)));
        
        bool areCollin = (right >= (left - (int)tolerance)) && (right <= (left + (int)tolerance));
        // log("[INFO] (collinear) areCollin = %d, a = <%d, %d>, b = <%d, %d>, c = <%d, %d>, left - tolerance = %d, right = %lld, left = %lld, tolerance = %d\n", areCollin, a.X, a.Y, b.X, b.Y, c.X, c.Y, left - tolerance, right, left, tolerance);
        // log("[INFO] (%d >= %d) && (%d <= %d) = %d, %d && %d\n", right, left - tolerance, right, left + tolerance, areCollin, right >= (left - (int)tolerance), right <= (left + (int)tolerance));
        return areCollin;
    }
    
    bool VolumeDecomposer::within(double p, double q, double r) {
        // Return true iff q is between p and r (inclusive).
        return (p <= q && q <= r) || (r <= q && q <= p);
    }
    
    Point VolumeDecomposer::closestPointOnLine(const Point start, const Point end, const Point pt) {
        Point lineVec = end - start;
        Point pntVec = pt - start;
        
        double lineLen = sqrt(vSize2f(lineVec));
        double lineUnitVec_x, lineUnitVec_y, pntVecScaled_x, pntVecScaled_y;
        lineUnitVec_x = double(lineVec.X) / double(lineLen);
        lineUnitVec_y = double(lineVec.Y) / double(lineLen);
        pntVecScaled_x = double(pntVec.X) / double(lineLen);
        pntVecScaled_y = double(pntVec.Y) / double(lineLen);
        
        double t = lineUnitVec_x * pntVecScaled_x + lineUnitVec_y * pntVecScaled_y;
        
        if (t < 0) t = 0;
        else if (t > 1) t = 1;
        
        Point nearestPt = Point(start.X + double(lineVec.X) * t + 0.5, start.Y + double(lineVec.Y) * t + 0.5);//lineVec / t;
        
        return nearestPt;
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
    
    unsigned int VolumeDecomposer::findSplitPoints(Mesh& mesh, int faceID, PolygonRef intersectingPolyRef, std::vector<std::pair<Point3, Point3>>& resultVect) {
        // The three points of the vertices of the face
        Point3 p0, p1, p2;
        
        // Gather face and vertex references
        const MeshFace& face = mesh.faces[faceID];
        const MeshVertex& v0 = mesh.vertices[face.vertex_index[0]];
        const MeshVertex& v1 = mesh.vertices[face.vertex_index[1]];
        const MeshVertex& v2 = mesh.vertices[face.vertex_index[2]];
        
        logAlways("[INFO] Vertices: <%d, %d, %d>, <%d, %d, %d>, <%d, %d, %d>\n", v0.p.x, v0.p.y, v0.p.z, v1.p.x, v1.p.y, v1.p.z, v2.p.x, v2.p.y, v2.p.z);
        
        // Build out the face polygon and retrieve area
        p0 = v0.p;
        p1 = v1.p;
        p2 = v2.p;
        Polygons facePoly = buildFacePolygon(p0, p1, p2);
        Polygons intersectingPoly = Polygons();
        intersectingPoly.add(intersectingPolyRef);
        double facePolyArea = facePoly[0].area();
        
        // log("[INFO] facePolyArea = %f\n", facePolyArea);
        // log("[INFO] face:\n%s\n", polygonRefToString(facePoly[0]).c_str());
        
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
        
        // The following section gets the diff polygon by subtracting the comparison poly from the face poly
        Polygons diff;
        
        // log("[INFO] pre-offset intersectingPoly:\n%s\n", polygonRefToString(intersectingPolyRef).c_str());
        
        if (facePolyArea == 0) {
            // If the face is a simple polyline and that polyline is on the egde of the intersecting
            // polygon, then the line may not be correctly cut, so we expand the polygon by 1
            // and hope the small inaccuracy in resulting coordinates doesn't matter.
            // Later code also compensates for this by checking for equality within +/- 1 of each coord
            // TODO: Fix this
            intersectingPoly = intersectingPoly.offset(1);
            diff = Polygons();
            ClipperLib::PolyTree diffTree = facePoly.lineSegmentDifference(intersectingPoly);
            
            if (diffTree.Total() > 0) {
                PolygonRef diffRef = PolygonRef(diffTree.GetFirst()->Contour);
                diff.add(diffRef);
            }
            intersectingPoly = intersectingPoly.offset(3);
        } else {
            diff = facePoly.difference(intersectingPoly);
            intersectingPoly = intersectingPoly.offset(4);
        }
        
        if (diff.size() == 0) { return 0; }
        
        unsigned int numPairsFound = 0;
        
        // Now that we've established the face is part of at least one overhang,
        // we iterate through all the diff polys in order to identify all pairs of
        // split points on it
        for (unsigned int cutPoly_idx = 0; cutPoly_idx < diff.size(); cutPoly_idx++) {
            // Look for the (up to) two new points, which will be the split point
            PolygonRef cutPoly = diff[0];
            // log("[INFO] diff size = %d\n", diff.size());
            // log("[INFO] pre-simplification:\n%s\n", polygonRefToString(cutPoly).c_str());
            
            // This step checks if the resulting diff poly is too small/insignificant to be properly split
            // It uses the ClipperLib simplify function and checks if the resulting poly still exists
            // If it doesn't, the poly is ignored
            // This is frequently caused by faces which are nearly, but not quite, vertical, thus producing
            // very small flattened shapes
            // TODO: Replace this step by setting a minimum angle between face and normal, or a maximum area
            if (facePolyArea > 0) {
                ClipperLib::Path cutPolyPath(cutPoly.begin(), cutPoly.end());
                PolygonRef cutPolyCopy(cutPolyPath);
                cutPolyCopy.simplify();
                if (cutPolyCopy.size() == 0) {
                    log("[INFO] Flattened face had non-zero area but was eliminated by simplification\n");
                    return 0;
                }
            }
            
            // log("[INFO] not simplified:\n%s\n", polygonRefToString(cutPoly).c_str());
            
            unsigned int numPointsFound = 0;
            Point3 pt;
            std::pair<Point3, Point3> result = std::make_pair(Point3(0, 0, 0), Point3(0, 0, 0));
            // Iterates through each point of the poly produced by subtracting the comparison poly
            // from the face poly and checks to see if that point is a split point
            for (unsigned int i = 0; i < cutPoly.size() && numPointsFound < 2; ++i) {
                Point cutPolyPt = cutPoly[i];
                bool foundPoint = false;
                
                // The point is a split point only if it's inside (i.e. on the edge) of the comparison
                // poly
                if (!intersectingPoly.inside(cutPolyPt, true)) continue;
                
                log("[INFO] Point being checked: <%d, %d>\n", cutPolyPt.X, cutPolyPt.Y);
                
                // Check if the split point is any of the face's vertices, in which case
                // we use the vertex value which is better than using the potentially
                // slightly off split point value
                // TODO: Quick and dirty hack to have multiple if statements all ligned up,
                //       change it so it's smarter
                if ((cutPolyPt.X <= v0.p.x + 1 && cutPolyPt.X >= v0.p.x - 1) &&
                    (cutPolyPt.Y <= v0.p.y + 1 && cutPolyPt.Y >= v0.p.y - 1)) {
                    if (numPointsFound == 0) {
                        result.first = v0.p;
                    } else {
                        result.second = v0.p;
                    }
                    numPointsFound++;
                    foundPoint = true;
                }
                if ((cutPolyPt.X <= v1.p.x + 1 && cutPolyPt.X >= v1.p.x - 1) &&
                    (cutPolyPt.Y <= v1.p.y + 1 && cutPolyPt.Y >= v1.p.y - 1)) {
                    if (numPointsFound == 0) {
                        result.first = v1.p;
                    } else {
                        result.second = v1.p;
                    }
                    numPointsFound++;
                    foundPoint = true;
                }
                if ((cutPolyPt.X <= v2.p.x + 1 && cutPolyPt.X >= v2.p.x - 1) &&
                    (cutPolyPt.Y <= v2.p.y + 1 && cutPolyPt.Y >= v2.p.y - 1)) {
                    if (numPointsFound == 0) {
                        result.first = v2.p;
                    } else {
                        result.second = v2.p;
                    }
                    numPointsFound++;
                    foundPoint = true;
                }
                // If the split point was not a vertex, we check to see which edge of the face it's
                // on in order to retrieve the point's z-value
                if (findZValueOf2DPointon3DLine(v0.p, v1.p, cutPolyPt, pt)) {
                    if (numPointsFound == 0) {
                        result.first = pt;
                        numPointsFound++;
                    } else {
                        if (!(Point3Equals(result.first, pt, 1))) {
                            result.second = pt;
                            numPointsFound++;
                        }
                    }
                }
                if (findZValueOf2DPointon3DLine(v1.p, v2.p, cutPolyPt, pt)) {
                    if (numPointsFound == 0) {
                        result.first = pt;
                        numPointsFound++;
                    } else {
                        if (!(Point3Equals(result.first, pt, 1))) {
                            result.second = pt;
                            numPointsFound++;
                        }
                    }
                }
                if (findZValueOf2DPointon3DLine(v2.p, v0.p, cutPolyPt, pt)) {
                    if (numPointsFound == 0) {
                        result.first = pt;
                        numPointsFound++;
                    } else {
                        if (!(Point3Equals(result.first, pt, 1))) {
                            result.second = pt;
                            numPointsFound++;
                        }
                    }
                }
            }
            
            if (numPointsFound < 2) {
                result.second = result.first;
            }
            
            resultVect.push_back(result);
            numPairsFound++;
            
            log("split points: <%d, %d, %d>, <%d, %d, %d>\n", result.first.x, result.first.y, result.first.z, result.second.x, result.second.y, result.second.z);
        }
        
        // log("numPairsFound = %d, resultVectSize = %d\n", numPairsFound, resultVect.size());
        return numPairsFound;
    }
    
    MeshSequence VolumeDecomposer::separateMesh(Mesh mesh, std::vector<int> seedVertices){
        
        std::vector<Mesh> childrenMeshes;
        std::vector<bool> markedFaces;
        
        if( seedVertices.empty()){
            MeshSequence meshSeq = {mesh, childrenMeshes};
            return meshSeq;
        }
        
        
        
        //create new meshes for all of the sub-meshes (overhangs) using the seed vertices
        for(int i = 0; i < seedVertices.size(); i++){
            
            //find one face on the sob-mesh which we can use to start the BFS queue
            int anAdjacentFaceIndex = -1;
            if(mesh.vertices[seedVertices[i]].connected_faces[0] != -1){
                anAdjacentFaceIndex = mesh.vertices[seedVertices[i]].connected_faces[0];
            }
            else if(mesh.vertices[seedVertices[i]].connected_faces.size() > 1 && mesh.vertices[seedVertices[i]].connected_faces[1] != -1){
                anAdjacentFaceIndex = mesh.vertices[seedVertices[i]].connected_faces[1];
            }else if(mesh.vertices[seedVertices[i]].connected_faces.size() > 2 && mesh.vertices[seedVertices[i]].connected_faces[2] != -1){
                anAdjacentFaceIndex = mesh.vertices[seedVertices[i]].connected_faces[2];
            }else{
                log("[ERROR] a floating face was found");
            }
            
            if( anAdjacentFaceIndex >= markedFaces.size() || !markedFaces[anAdjacentFaceIndex] ){ //if any of the faces have been marked, this mesh has already been created so we can skip it
                
                std::queue<int> faceQueue;
                Mesh child = new Mesh( FffProcessor::getInstance());
                
                faceQueue.push(anAdjacentFaceIndex);
                
                //BFS on faces
                while( !faceQueue.empty()){
                    int faceIndex = faceQueue.front();
                    
                    if(faceIndex != -1){
                        
                        if(faceIndex >= markedFaces.size()){
                            markedFaces.resize(faceIndex+1);
                        }
                        
                        markedFaces.at(faceIndex) = true;
                        
                        
                        Point3 p0 = mesh.vertices[mesh.faces[faceIndex].vertex_index[0]].p;
                        Point3 p1 = mesh.vertices[mesh.faces[faceIndex].vertex_index[1]].p;
                        Point3 p2 = mesh.vertices[mesh.faces[faceIndex].vertex_index[2]].p;
                        
                        child.addFace(p0, p1, p2);
                        
                        for( int adjacentFace : mesh.faces[faceIndex].connected_face_index){
                            if( adjacentFace >= markedFaces.size() || !markedFaces.at(adjacentFace) ){
                                faceQueue.push(adjacentFace);
                            }
                        }
                    }
                    
                    faceQueue.pop();
                }
                childrenMeshes.push_back(child);
            }
        }
        
        //Find the base mesh, which is the mesh the children will be build upon
        int seedIndex = 0;
        
        //loop until an face that has not been already added to a mesh is found
        while(!(seedIndex >= markedFaces.size()) && markedFaces[seedIndex]){ //find a face which has not been marked
            seedIndex++;
        }
        
        if( seedIndex >= mesh.faces.size()){ //There are no more faces to form the parent mesh
            log("[ERROR] No Parent mesh found when seperating meshes!");
            MeshSequence meshSeq = {mesh, childrenMeshes};
            return meshSeq;
        }
        
        std::queue<int> faceQueue;
        Mesh parent = new Mesh( FffProcessor::getInstance());
        
        faceQueue.push(seedIndex);
        
        while( !faceQueue.empty()){
            int faceIndex = faceQueue.front();
            
            if(faceIndex != -1){
                
                if(faceIndex >= markedFaces.size()){
                    markedFaces.resize(faceIndex+1);
                }
                markedFaces.at(faceIndex) = true;
                
                Point3 p0 = mesh.vertices[mesh.faces[faceIndex].vertex_index[0]].p;
                Point3 p1 = mesh.vertices[mesh.faces[faceIndex].vertex_index[1]].p;
                Point3 p2 = mesh.vertices[mesh.faces[faceIndex].vertex_index[2]].p;
                
                parent.addFace(p0, p1, p2);
                
                for( int adjacentFace : mesh.faces[faceIndex].connected_face_index){
                    if( adjacentFace >= markedFaces.size() || !markedFaces[adjacentFace] ){
                        faceQueue.push(adjacentFace);
                    }
                }
            }
            faceQueue.pop();
        }
        
        //Error checking to ensure that all faces we processed and added to an new mesh
        if(markedFaces.size() == mesh.faces.size()){
            for(int i = 0; i < markedFaces.size(); i++){
                if(markedFaces.at(i) == false){
                    log("[ERROR] There was a face in the original mesh that was not added to the seperated meshes");
                }
            }
        }else{
            log("[ERROR] There was a face in the original mesh that was not added to the seperated meshes");
        }
        MeshSequence meshSeq = {parent, childrenMeshes};
        return meshSeq;
    }
    
    bool VolumeDecomposer::findZValueOf2DPointon3DLine(const Point3& P3_0, const Point3& P3_1, const Point& startPoint, Point3& resultPoint) {
        const Point flat_p0 = Point(P3_0.x, P3_0.y);
        const Point flat_p1 = Point(P3_1.x, P3_1.y);
        
        log("[INFO] (findZValueOf2DPointon3DLine) P3_0 = <%d, %d, %d>, P3_1 = <%d, %d, %d>\n", P3_0.x, P3_0.y, P3_0.z, P3_1.x, P3_1.y, P3_1.z);
        
        // Checks to see if the given point is actually on the 3D line
        Point actualPt = closestPointOnLine(flat_p0, flat_p1, startPoint);
        float dist = sqrt((actualPt.X - startPoint.X)*(actualPt.X - startPoint.X) + (actualPt.Y - startPoint.Y)*(actualPt.Y - startPoint.Y));
        if (dist <= 4) {
            // Get the z-value of the x/y combo
            const Point3 P3_vertexDiff = Point3(P3_1.x, P3_1.y, 0) - Point3(P3_0.x, P3_0.y, 0);
            const Point3 P3_intersectDiff = Point3(actualPt.X, actualPt.Y, 0) - Point3(P3_0.x, P3_0.y, 0);
            double t = (double)P3_intersectDiff.vSize() / (double)P3_vertexDiff.vSize();
            
            int intersection_z = (P3_1.z - P3_0.z) * t + P3_0.z;
            
            resultPoint = Point3(actualPt.X, actualPt.Y, intersection_z);
            return true;
        }
        
        return false;
    }
    
    bool VolumeDecomposer::faceIsOverhangIntersect(const Mesh& mesh, int faceID, const PolygonRef& poly) {
        const MeshFace& face = mesh.faces[faceID];
        const MeshVertex& v0 = mesh.vertices[face.vertex_index[0]];
        const MeshVertex& v1 = mesh.vertices[face.vertex_index[1]];
        const MeshVertex& v2 = mesh.vertices[face.vertex_index[2]];
        
        const Point p0 = Point(v0.p.x, v0.p.y);
        const Point p1 = Point(v1.p.x, v1.p.y);
        const Point p2 = Point(v2.p.x, v2.p.y);
        
        bool p0_inside = poly.inside(p0, true);
        bool p1_inside = poly.inside(p1, true);
        bool p2_inside = poly.inside(p2, true);
        
        if (!((p0_inside && p1_inside && p2_inside) || (!p0_inside && !p1_inside && !p2_inside))) {
            return true;
        }
        
        return false;
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
    
    std::string VolumeDecomposer::faceToString(Mesh & mesh, int id) {
        const MeshFace& face = mesh.faces[id];
        const MeshVertex& v0 = mesh.vertices[face.vertex_index[0]];
        const MeshVertex& v1 = mesh.vertices[face.vertex_index[1]];
        const MeshVertex& v2 = mesh.vertices[face.vertex_index[2]];
        std::string faceString = "v0: [";
        faceString += std::to_string(v0.p.x)  + ", " + std::to_string(v0.p.y) + ", " + std::to_string(v0.p.z) + "], v1: [" + std::to_string(v1.p.x) + ", " + std::to_string(v1.p.y) + ", " + std::to_string(v1.p.z) + "], v2: [" + std::to_string(v2.p.x) + ", " + std::to_string(v2.p.y) + ", " + std::to_string(v2.p.z) + "]";
        return faceString;
    }
    
    std::string VolumeDecomposer::polygonRefToString(const PolygonRef& pref) {
        std::string ret = "Polygon[";
        for (unsigned int i = 0; i < pref.size(); ++i) {
            const Point p = pref[i];
            
            ret += "(" + std::to_string(p.X) + ", " + std::to_string(p.Y) + ")";
            
            if (i < pref.size() - 1) ret += ",";
        }
        ret += "]";
        return ret;
    }
    
    FPoint3 VolumeDecomposer::faceNormal(const Mesh& mesh, const MeshFace& face) {
        Point3 v0 = mesh.vertices[face.vertex_index[1]].p - mesh.vertices[face.vertex_index[0]].p;
        Point3 v1 = mesh.vertices[face.vertex_index[2]].p - mesh.vertices[face.vertex_index[1]].p;
        
        FPoint3 norm = FPoint3::cross(v0, v1);
        
        return norm;
    }
    
    Point3 VolumeDecomposer::truncatedFaceNormal(const Mesh& mesh, const MeshFace& face) {
        FPoint3 norm = faceNormal(mesh, face);
        
        Point3 truncNorm = Point3((int)floor(norm.x), (int)floor(norm.y), (int)floor(norm.z));
        
        return truncNorm;
    }
