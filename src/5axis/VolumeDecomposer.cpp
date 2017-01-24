#include "VolumeDecomposer.hpp"
#include "../utils/polygon.h"
#include "../utils/intpoint.h"

namespace cura {

VolumeDecomposer::VolumeDecomposer(Mesh& mesh, Slicer* slicer) {
	std::vector<SlicerLayer> & layers = slicer->layers;

	// Initialize all of our comparison vars
	SlicerLayer & comparisonSlice = layers[0];
	Polygons & comparisonPolys = comparisonSlice.polygons;

	for (unsigned int layer_idx = 1; layer_idx < layers.size(); ++layer_idx) {
		SlicerLayer & slice = layers[layer_idx];
		Polygons & polys = slice.polygons;
		Polygons & openPolys = slice.openPolylines;
		std::vector<std::vector<int>> polyFaces = slice.polyFaces;

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
			std::vector<int> faces = polyFaces[polyfaces_idx];
			log("[CUSTOM] Polygon ID: %d\n", polyfaces_idx);
			for (unsigned int face_idx = 0; face_idx < faces.size(); ++face_idx) {
				int faceID = faces[face_idx];
				std::string faceString = VolumeDecomposer::faceToString(mesh, faceID);
				std::pair<bool, int> intersectOverhangResult = faceIsOverhangIntersect(mesh, faceID, comparisonPolys);
				bool intersectingOverhang = intersectOverhangResult.first;
				int polygonID = intersectOverhangResult.second;

				log("\t%s (intersects: %s)\n", faceString.c_str(), intersectingOverhang ? "true" : "false");

				if (intersectingOverhang) {
					// FPoint3 intersection = findPolyFaceIntersection(mesh, faceID, comparisonPolys[polygonID], sliceHeight);
					std::pair<Point3, Point3> splitPoints;
					int numSplitPoints = findSplitPoints(mesh, faceID, comparisonPolys[polygonID], splitPoints);
					splitFace(mesh, faceID, numSplitPoints, splitPoints);
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

unsigned int VolumeDecomposer::findSplitPoints(Mesh& mesh, int faceID, PolygonRef intersectingPolyRef, std::pair<Point3, Point3>& result) {
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
	if (facePolyArea == 0) {
		// If the face is a simple polyline and that polyline is on the egde of the intersecting
		// polygon, then the line may not be correctly cut, so we expand the polygon by 1
		// and hope the small inaccuracy in resulting coordinates doesn't matter.
		// TODO: Fix this
		intersectingPoly = intersectingPoly.offset(1);
		ClipperLib::PolyTree diffTree = facePoly.lineSegmentDifference(intersectingPoly);
		PolygonRef diffRef = PolygonRef(diffTree.GetFirst()->Contour);
		// log("Number of nodes in tree: %d\n", diffTree.Total());

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
		Point cutPolyPt = cutPoly[i];

		if (intersectingPoly.inside(cutPolyPt, true)) {
			splitPoints[numSplitPoints] = &cutPolyPt;
			++numSplitPoints;
		}

		// for (unsigned int j = 0; j < facePolyRef.size(); ++j) {
			// Point facePt = facePolyRef[j];

			// log("cutPolyPt: <%d, %d>, facePt: <%d, %d> (%d, %d)\n", cutPolyPt.X, cutPolyPt.Y, facePt.X, facePt.Y, cutPolyPt.X <= facePt.X + 1, cutPolyPt.X >= facePt.X - 1);
			
			// This is to deal with the max +-1 change in coordinate from offset the intersecting polygon by 1 earlier
			// if (cutPolyPt.X <= facePt.X + 1 && cutPolyPt.X >= facePt.X - 1 &&
			// 	cutPolyPt.Y <= facePt.Y + 1 && cutPolyPt.Y >= facePt.Y - 1) {
			// 	break;
			// }

			// if (j == (facePolyRef.size() - 1)) {
			// 	splitPoints[numSplitPoints] = &cutPolyPt;
			// 	++numSplitPoints;
			// }
		// }
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


			// If there are 2 common vertices, that means this is a face parallel to the +z-axis
			// and the split is along one of the sides of the face
			if (numCommonVertices == 2) {
				result = std::make_pair(commonVertices[0]->p, commonVertices[1]->p);
			}
			// If there is one common vertex, this is either a non-parallel face
			// that barely touches the polygon with one vertex, or a parallel face
			// that may coincide from the vertex to somewhere on an edge in the +z direction
			else if (numCommonVertices == 1) {
				result = std::make_pair(commonVertices[0]->p, Point3(0, 0, 0));
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
					if (tracePointStraightUp(otherVertices[0]->p, otherVertices[1]->p, result.first, secondPoint)) {
						result.second = secondPoint;
					}
				}
			}
			// If there are no common vertices, this is a parallel face where the intersecting points
			// are in the middle of the triangle somewhere
			else {
				Point p0_flat = Point(otherVertices[0]->p.x, otherVertices[0]->p.y);
				Point p1_flat = Point(otherVertices[1]->p.x, otherVertices[1]->p.y);
				Point p2_flat = Point(otherVertices[2]->p.x, otherVertices[2]->p.y);

				if (isOn(p0_flat, p1_flat, splitPoint)) {
					if (tracePointStraightUp(otherVertices[0]->p, otherVertices[1]->p, ))
				}
				else if (isOn(p0_flat, p2_flat, splitPoint)) {

				} else {

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

bool VolumeDecomposer::tracePointStraightUp(const Point3& P3_0, const Point3& P3_1, const Point3& startPoint, Point3& resultPoint) {
	bool isHigherThanBoth = startPoint.z > P3_0.z && startPoint.z > P3_1.z;
	bool isLowerThanBoth = startPoint.z < P3_0.z && startPoint.z < P3_1.z;

	// If the middle point is between two points (in the z-direction), or the middle point
	// is below both other points (in the z-direction), then there is another point above it to
	// split the face by
	if ((!isLowerThanBoth && !isHigherThanBoth) || isLowerThanBoth) {
		const Point3 P3_vertexDiff = Point3(P3_1.x, P3_1.y, 0) - Point3(P3_0.x, P3_0.y, 0);
		const Point3 P3_intersectDiff = Point3(startPoint.x, startPoint.y, 0) - Point3(P3_0.x, P3_0.y, 0);
		int t = P3_vertexDiff.vSize() / P3_intersectDiff.vSize();

		int intersection_z = (1 - t) * P3_0.z + t * P3_1.z;

		resultPoint = Point3(startPoint.x, startPoint.y, intersection_z);
		return true;
	}

	return false;
}

std::pair<bool, int> VolumeDecomposer::faceIsOverhangIntersect(Mesh& mesh, int faceID, Polygons& comparisonPolys) {
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
			return std::make_pair(true, poly_idx);
		}
	}

	return std::make_pair(false, -1);
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
	std::string ret = "Polygon points:\n";
	for (unsigned int i = 0; i < pref.size(); ++i) {
		const Point p = pref[i];

		ret += "\t#" + std::to_string(i) + ": <" + std::to_string(p.X) + ", " + std::to_string(p.Y) + ">\n";
	}
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