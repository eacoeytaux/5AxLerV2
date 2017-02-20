#include "VolumeDecomposer.hpp"
#include "../utils/polygon.h"
#include "../utils/intpoint.h"
#include "comms/SerialComms.hpp"

namespace cura {

VolumeDecomposer::VolumeDecomposer(Mesh& mesh, Slicer* slicer) {
	// SerialComms sc = SerialComms("/dev/ttyACM0");
	std::vector<SlicerLayer> & layers = slicer->layers;

	// Initialize all of our comparison vars
	SlicerLayer & comparisonSlice = layers[0];
	Polygons & comparisonPolys = comparisonSlice.polygons;

	unsigned int numLayers = layers.size();
	log("Progress: [", 0);

	for (unsigned int layer_idx = 1; layer_idx < layers.size(); ++layer_idx) {
		SlicerLayer & slice = layers[layer_idx];
		Polygons & polys = slice.polygons;
		Polygons & openPolys = slice.openPolylines;
		std::vector<std::vector<int>> polyFaces = slice.polyFaces;

		if ((int)(100.0 * ((double)layer_idx / (double)(numLayers - 1))) % 5 == 0) {
			log("=");
		}

		// Main loop
		for (unsigned int polyfaces_idx = 0; polyfaces_idx < polyFaces.size(); ++polyfaces_idx) {
			std::vector<int> faces = polyFaces[polyfaces_idx];

			for (unsigned int comparisonPolys_idx = 0; comparisonPolys_idx < comparisonPolys.size(); ++comparisonPolys_idx) {
				for (unsigned int face_idx = 0; face_idx < faces.size(); ++face_idx) {
					int faceID = faces[face_idx];
					std::string faceString = VolumeDecomposer::faceToString(mesh, faceID);

					bool intersectingOverhang = faceIsOverhangIntersect(mesh, faceID, comparisonPolys[comparisonPolys_idx]);
					if (intersectingOverhang) {
						std::vector<std::pair<Point3, Point3>> splitPoints;
						int numSplitPairs = findSplitPoints(mesh, faceID, comparisonPolys[comparisonPolys_idx], splitPoints);
						
						for (unsigned int splitPointPair_idx = 0; splitPointPair_idx < slitPoints.size(); ++splitPointPair_idx) {
							splitFace(mesh, faceID, numSplitPoints, splitPoints[splitPointPair_idx]);
						}
					}
				}
			}
		}

		comparisonPolys = polys;
	}
}

void VolumeDecomposer::splitFace(Mesh& mesh, int faceID, int numSplitPoints, std::pair<Point3, Point3>& splitPoints) {
	if (numSplitPoints == 0) return;

	// Grab the actual MeshFace
	const MeshFace& face = mesh.faces[faceID];

	// If there's only one splitpoint, or the splitpoints are the same, draw a line from that point
	// up and see if an intersect with another of the face's sides is found, then you've found your split
	// 
	if (numSplitPoints == 1 || splitPoints.first == splitPoints.second) {

	} else {

	}
}

bool VolumeDecomposer::isOn(Point a, Point b, Point c, int tolerance) {
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

bool VolumeDecomposer::collinear(Point a, Point b, Point c, int tolerance) {
	// Return true iff a, b, and c all lie on the same line.
	int left = abs((b.X - a.X) * (c.Y - a.Y));
	int right = abs((c.X - a.X) * (b.Y - a.Y));
	log("[INFO] (collinear) left: %d, right: %d, tolerance: %d\n", left, right, tolerance);
	bool areCollin = (right >= left - tolerance) && (right <= left + tolerance);
	return areCollin;
}

bool VolumeDecomposer::within(double p, double q, double r) {
    // Return true iff q is between p and r (inclusive).
    return (p <= q && q <= r) || (r <= q && q <= p);
}

Point VolumeDecomposer::closestPointOnLine(const Point start, const Point end, const Point pt) {
	Point lineVec = end - start;
	Point pntVec = pt - start;

	int lineLen = vSize(lineVec);
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

unsigned int VolumeDecomposer::findSplitPoints(Mesh& mesh, int faceID, PolygonRef intersectingPolyRef, std::vector<std::pair<Point3, Point3>> resultVect) {
	// The three points of the vertices of the face
	Point3 p0, p1, p2;

	// Gather face and vertex references
	const MeshFace& face = mesh.faces[faceID];
	const MeshVertex& v0 = mesh.vertices[face.vertex_index[0]];
	const MeshVertex& v1 = mesh.vertices[face.vertex_index[1]];
	const MeshVertex& v2 = mesh.vertices[face.vertex_index[2]];

	log("[INFO] Vertices: <%d, %d, %d>, <%d, %d, %d>, <%d, %d, %d>\n", v0.p.x, v0.p.y, v0.p.z, v1.p.x, v1.p.y, v1.p.z, v2.p.x, v2.p.y, v2.p.z);

	// Build out the face polygon and retrieve area
	p0 = v0.p;
	p1 = v1.p;
	p2 = v2.p;
	Polygons facePoly = buildFacePolygon(p0, p1, p2);
	Polygons intersectingPoly = Polygons();
	intersectingPoly.add(intersectingPolyRef);
	double facePolyArea = facePoly[0].area();

	log("[INFO] facePolyArea = %f\n", facePolyArea);
	log("[INFO] face:\n%s\n", polygonRefToString(facePoly[0]).c_str());

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

	log("[INFO] pre-offset intersectingPoly:\n%s\n", polygonRefToString(intersectingPolyRef).c_str());

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
		log("[INFO] diff size = %d\n", diff.size());
		log("[INFO] pre-simplification:\n%s\n", polygonRefToString(cutPoly).c_str());

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

		log("[INFO] not simplified:\n%s\n", polygonRefToString(cutPoly).c_str());

		unsigned int numPointsFound = 0;
		Point3 pt;
		std::pair<Point3, Point3> result = std::make_pair(Point3(0, 0, 0), Point3(0, 0, 0));
		// Iterates through each point of the poly produced by subtracting the comparison poly
		// from the face poly and checks to see if that point is a split point
		for (unsigned int i = 0; i < cutPoly.size() && numPointsFound < 2; ++i) {
			Point cutPolyPt = cutPoly[i];

			log("[INFO] Point being checked: <%d, %d>\n", cutPolyPt.X, cutPolyPt.Y);

			// The point is a split point only if it's inside (i.e. on the edge) of the comparison
			// poly
			if (!intersectingPoly.inside(cutPolyPt, true)) continue;

			log("[INFO] Point is inside\n");

			// Check if the split point is any of the face's vertices, in which case
			// we use the vertex value which is better than using the potentially
			// slightly off split point value
			if ((cutPolyPt.X <= v0.p.x + 1 && cutPolyPt.X >= v0.p.x - 1) &&
				(cutPolyPt.Y <= v0.p.y + 1 && cutPolyPt.Y >= v0.p.y - 1)) {
				if (numPointsFound == 0) {
					result.first = v0.p;
				} else {
					result.second = v0.p;
				}
				numPointsFound++;
			}
			else if ((cutPolyPt.X <= v1.p.x + 1 && cutPolyPt.X >= v1.p.x - 1) &&
				(cutPolyPt.Y <= v1.p.y + 1 && cutPolyPt.Y >= v1.p.y - 1)) {
				if (numPointsFound == 0) {
					result.first = v1.p;
				} else {
					result.second = v1.p;
				}
				numPointsFound++;
			}
			else if ((cutPolyPt.X <= v2.p.x + 1 && cutPolyPt.X >= v2.p.x - 1) &&
				(cutPolyPt.Y <= v2.p.y + 1 && cutPolyPt.Y >= v2.p.y - 1)) {
				if (numPointsFound == 0) {
					result.first = v2.p;
				} else {
					result.second = v2.p;
				}
				numPointsFound++;
			}
			// If the split point was not a vertex, we check to see which edge of the face it's
			// on in order to retrieve the point's z-value
			else if (findZValueOf2DPointon3DLine(v0.p, v1.p, cutPolyPt, pt)) {
				if (numPointsFound == 0) {
					result.first = pt;
				} else {
					result.second = pt;
				}
				numPointsFound++;
			}
			else if (findZValueOf2DPointon3DLine(v1.p, v2.p, cutPolyPt, pt)) {
				if (numPointsFound == 0) {
					result.first = pt;
				} else {
					result.second = pt;
				}
				numPointsFound++;
			}
			else if (findZValueOf2DPointon3DLine(v2.p, v0.p, cutPolyPt, pt)) {
				if (numPointsFound == 0) {
					result.first = pt;
				} else {
					result.second = pt;
				}
				numPointsFound++;
			}
		}

		if (numPointsFound < 2) {
			log("[ERROR] Only %d intersection points were found\n", numPointsFound);
		}

		resultVect.push_back(result);
		numPairsFound++;
	}

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
		int anAdjacentFaceIndex = mesh.vertices[seedVertices[i]].connected_faces[0];
		
		if( anAdjacentFaceIndex >= markedFaces.size() || !markedFaces[anAdjacentFaceIndex] ){ //if any of the faces have been marked, this mesh has already been created so we can skip it
		
			std::queue<int> faceQueue;
			Mesh child = new Mesh( FffProcessor::getInstance());
			
			faceQueue.push(mesh.vertices[seedVertices[i]].connected_faces[0]);
		
			while( !faceQueue.empty()){
				int faceIndex = faceQueue.front();
				
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
				
				faceQueue.pop();
			}
			childrenMeshes.push_back(mesh);
		}
	}
	
	//Find the base mesh, which is the mesh the children will be build upon
	int seedIndex = 0;
	
	//loop until an face that has not been already added to a mesh is found
	while(!(seedIndex >= markedFaces.size()) && markedFaces[seedIndex]){ //find a face which has not been marked
			seedIndex++;
	}
	
	if( seedIndex >= mesh.faces.size()){ //There are no more faces to form the parent mesh
		log("No Parent mesh found when seperating meshes!");
		MeshSequence meshSeq = {mesh, childrenMeshes};
		return meshSeq;
	}
	
	std::queue<int> faceQueue;
	Mesh parent = new Mesh( FffProcessor::getInstance());
	
	faceQueue.push(seedIndex);
	
	while( !faceQueue.empty()){
		int faceIndex = faceQueue.front();
		
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
		faceQueue.pop();
	}
	
	//Error checking to ensure that all faces we processed and added to an new mesh
	if(markedFaces.size() == mesh.faces.size()){
		for(int i = 0; i < markedFaces.size(); i++){
			if(markedFaces.at(i) == false){
				log("There was a face in the original mesh that was not added to the seperated meshes");
			}
		}
	}else{
		log("There was a face in the original mesh that was not added to the seperated meshes");
	}
	
	MeshSequence meshSeq = {mesh, childrenMeshes};
	return meshSeq;
}


bool VolumeDecomposer::findZValueOf2DPointon3DLine(const Point3& P3_0, const Point3& P3_1, const Point& startPoint, Point3& resultPoint) {
	const Point flat_p0 = Point(P3_0.x, P3_0.y);
	const Point flat_p1 = Point(P3_1.x, P3_1.y);

	// Checks to see if the given point is actually on the 3D line
	if (isOn(flat_p0, flat_p1, startPoint, 300)) {
		// Now it's possible for the point to be slightly off the line, so we get it as close to the actual line as possible
		Point actualPt = closestPointOnLine(flat_p0, flat_p1, startPoint);

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

}
