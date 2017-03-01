/**
 * VolumeDecomposer.cpp takes in layers of an object and creates individual sub-volumes
 * which each have no overhangs
 *
 * Created by Alexandre Pauwels on 11/4/2016
 */

#ifndef VOLUME_DECOMPOSER
#define VOLUME_DECOMPOSER

#include "../mesh.h"
#include "../slicer.h"
#include "../utils/floatpoint.h"
#include "../utils/intpoint.h"
#include "../FffProcessor.h"
#include "SeqGraph.hpp"


//#ifdef RUN_TESTS
//#define CATCH_CONFIG_MAIN
//#include "../libs/Catch/catch.hpp"
//#endif

namespace cura {
    typedef struct meshSequence{
        Mesh parent;
        std::vector<Mesh> children;
    } MeshSequence;
    
    class VolumeDecomposer {
    public:
		SeqGraph sequenceGraph;
        VolumeDecomposer(Mesh& mesh);
        
    private:
		void decompose(Mesh& mesh, bool first);
		
        /**
         * Outputs a string with all the vertices of the specified face
         *
         * @param mesh The mesh the face is in
         * @param id The ID of the face in that mesh
         *
         * @return A string representation of the face
         */
        static std::string faceToString(Mesh & mesh, int id);
        
        /**
         * Outputs a string with all the points in the PolygonRef. This string
         * has appropriate newlines for clarity, but does not have a newline at the end.
         *
         * @param pref The PolygonRef to print out
         *
         * @return A string representation of the polygon
         */
        static std::string polygonRefToString(const PolygonRef& pref);
        
        /**
         * Takes in the three points of a face and returns a Polygons object representing the face.
         * The face has to be flatenned to do this, so only the <x, y> portion of each point is kept.
         * If the resulting polygon is an open polyline (its normal is perpendicular to the +z-axis),
         * then the polyline is cleaned up to only have two points, eliminating the unnecessary
         * intermediary point.
         *
         * @param p0 The first 3D point of the face
         * @param p1 The second 3D point of the face
         * @param p2 The third 3D point of the face
         *
         * @return A Polygons object that represents the flattened face
         */
        Polygons buildFacePolygon(Point3 p0, Point3 p1, Point3 p2);
        
        /**
         * Given a face in a mesh and a polygon that intersects the face, splits the face along the
         * edge of the polygon. This function works by flattening out the face so that all of its
         * vertices have the same z-value, and then intersecting that 2D shape with the 2D polygon.
         * The returned value is the pair of <x, y, z> points which are the endpoints of the line
         * which cuts the face in half.
         */
        unsigned int findSplitPoints(Mesh& mesh, int faceID, PolygonRef intersectingPoly, std::vector<std::pair<Point3, Point3>>& resultVect);
        
        /**
         * Given two 3D points (P3_0, P3_1) and a third 3D point (startPoint) that is located between
         * the first two points when looking only at x/y values, this function fills the final parameter
         * (resultPoint) with a Point3 value that contains the point intersecting the line.
         *
         * @param P3_0 First Point3 of the 3D line
         * @param P3_1 Second Point3 of the 3D line
         * @param startPoint The point in the middle of the 3D line that is our starting off point somewhere
         *					 in the middle of P3_0 and P3_1 (when only looking at top-down and flattening the points)
         * @param resultPoint The resulting point, if any, will be stored here
         *
         * @return True if a point was found, false otherwise
         */
        bool findZValueOf2DPointon3DLine(const Point3& P3_0, const Point3& P3_1, const Point& startPoint, Point3& resultPoint);
        
        /**
         * Given a face in a mesh and 1 or 2 points on the sides of the face, splits the face along those points, while
         * maintaining mesh integrity by creating sub-triangles where necessary.
         * If there is:
         *	- 1 split point: Finds the point on the side of the face and splits the triangle along the +z-axis
         If the point is one of the vertices of the triangle, does not do anything
         *	- 2 split points: Finds the points on the side of the face, and splits the triangle along those points
	 					  If both points are the same, defaults down to a single split point
         */
        int splitFaces(Mesh& mesh, int faceID, PolygonRef intersectingPoly, std::pair<Point3, Point3> splitPoints);
        
        /**
         * This function takes a mesh, which has been modified to have disjoint parts, and a list of seed vertices
         * to identify where splits were made in the mesh.  These vertices are used to start crawling the mesh, via
         * adjacent face lists, and build new meshes.
         *
         * @param mesh The mesh that is being divided into multiple meshes.  It has been modified to have disjoint parts via splitFace()
         * @param seedVertices a list of vertex indices, which we identified to be in parts of the mesh other than the base/parent.  At least one index is guaranteed to be in each of the disjoint sub-meshes that have been created.
         */
        MeshSequence separateMesh(Mesh mesh, std::vector<int> seedVertices);
        
        /**
         * Discovers if the face is an overhang intersection for any of the polygons
         * in comparisonPoly. An overhang intersection is a face which is both inside and outside
         * of a polygon, it is at the border of the beginning of an overhang.
         *
         * @param mesh The mesh the face that's being checked is in
         * @param faceID The ID of the face in the mesh
         * @param comparisonPoly A Polygon object storing the polygons to check the face against
         *
         * @return A tuple where the first element is a number that indicates how many polygons it is an
         *		   an overhang of. If this number is 0, this face is not an overhang intersect. The second
         *		   part of the tuple is a vector which contains the IDs of the polygon the face
         *		   is an overhang on in the slice.
         */
        bool faceIsOverhangIntersect(const Mesh& mesh, int faceID, const PolygonRef& comparisonPoly);
        
        /**
         * findPolyFaceIntersection is fairly straightforward:
         * 1. Iterate through each line segment in the polygon
         * 2. Take the mesh face and the two points of the line and pass them through the following equations:
         *		d = (n_x * x_0) + (n_y * y_0) + (n_z * z_0)
         *		t = (d - x_0 - y_0 - z_0) / (deltaX + deltaY + deltaZ)
         *
         *		d is part of the scalar equation of the plane
         *		<n_x, n_y, n_z> are the face's normal
         *		<x_0, y_0, z_0> in the top equation is the <x, y, z> of one of the face's vertices
         *		<x_0, y_0, z_0> in the bottom equation is the <x, y, z> of one of the endpoints of the line
         *		deltaX = x_1 - x_0, where x_1 is the other endpoint of the line's x-coordinate (etc for deltaY, deltaZ)
         * 3. Now we check t. If 0 <= t <= 1, then the line from the comparison poly intersects the face
         * 4. In that case, we plug t into the following equation to get the final intersection's <x, y, z>:
         *		r(t) = <x_0 + deltaX*t, y_0 + deltaY*t, z_0 + deltaZ*t>
         *		<x_0, y_0, z_0> is the first endpoint of the line
         *		deltaX etc are defined as above
         * 5. Otherwise, the line does not intersect and we move on to the next line
         * 6. If no intersections are found, an exception is thrown
         */
        FPoint3 findPolyFaceIntersection(Mesh& mesh, int faceID, PolygonRef intersectingPoly, int sliceHeight);
        
        /**
         * Computes the normal of the given face. Returns an FPoint3, maintains
         * float precision.
         *
         * @param mesh The mesh the face is in
         * @param face The face to compute the normal of
         *
         * @return An FPoint3 object which corresponds to the normal vector
         */
        static FPoint3 faceNormal(const Mesh& mesh, const MeshFace& face);
        
        /**
         * Computes the normal of the given face but truncates floats into ints
         *
         * @param mesh The mesh the face is in
         * @param face The face to compute the normal of
         *
         * @return A Point3 object which corresponds to the normal vector
         */
        static Point3 truncatedFaceNormal(const Mesh& mesh, const MeshFace& face);
        
        // isOn finds out if c is on the same line as the one formed by points
        // a and b, collinear and within help with this
        bool isOn(const Point a, const Point b, const Point c, unsigned int tolerance = 0);
        bool collinear(const Point a, const Point b, const Point c, unsigned int tolerance = 0);
        bool within(double p, double q, double r);
        
        Point closestPointOnLine(const Point start, const Point end, const Point pt);
        bool p3EQ(Point3 fp1, Point3 fp2, int tolerance);
    };
	
}

#endif
