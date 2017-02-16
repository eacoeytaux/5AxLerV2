//
//  CollisionDetection.hpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 1/25/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#ifndef CollisionDetection_hpp
#define CollisionDetection_hpp

#include <stdio.h>
#include <math.h>


#include "SeqGraph.hpp"
#include "../mesh.h"
#include "../Slicer.h"
#include "../settings/settings.h"
#include "../MeshGroup.h"
#include "../SliceDataStorage.h"
#include "../FffProcessor.h"
#include "OBB3D.hpp"

namespace cura {
	class CollisionDetection {
		
	public:
		SeqGraph sequenceGraph;
		MeshGroup *meshgroup;
		
		/**
		 * Finds all collisions in a sequence of sub-meshes
		 *
		 * @param sequence The sequence of nodes with populated geometric childre, but initially no collision children
		 */
		void detectPrintCollisions(SeqGraph sequence);
		
		//CURA CODE
		int64_t interpolate(int64_t x, int64_t x0, int64_t x1, int64_t y0, int64_t y1) const
		{
			int64_t dx_01 = x1 - x0;
			int64_t num = (y1 - y0) * (x - x0);
			num += num > 0 ? dx_01/2 : -dx_01/2; // add in offset to round result
			int64_t y = y0 + num / dx_01;
			return y;
		}
		
		SlicerSegment project2D(Point3& p0, Point3& p1, Point3& p2, int32_t z) const
		{
			SlicerSegment seg;
			
			seg.start.X = interpolate(z, p0.z, p1.z, p0.x, p1.x);
			seg.start.Y = interpolate(z, p0.z, p1.z, p0.y, p1.y);
			seg.end  .X = interpolate(z, p0.z, p2.z, p0.x, p2.x);
			seg.end  .Y = interpolate(z, p0.z, p2.z, p0.y, p2.y);
			
			return seg;
		}
		//END CURA CODE
		
	private:
		
		void specialSlice(Mesh* mesh, int initial, int thickness, int slice_layer_count, bool keep_none_closed, bool extensive_stitching);
		
		/**
		 * Finds all aabb collision on on a node, the recursively calls itself on all children
		 *
		 * @param node current node of sequence graph
		 */
		//void recursiveAabb(SequenceNode &);
		
		/**
		 * Finds all collision possibilites using axis-aligned bounded boxes
		 *
		 * @param printingNode The index of the node which is being printed
		 *
		 * @return A vector of potential collisions
		 */
		std::vector<int> aabbCollisionDetection(int printingNode);
		
		/**
		 * Performs collision detection using the slice method
		 *
		 * @param printingNode The node which is being printed
		 
		 */
		void sliceCollisionDetection(int printingNode, std::vector<int> possibleCollisions);
		
		/**
		 * helper functions
		 *
		 */
		
		static std::vector<std::vector<int>> extractRotation(std::vector<std::vector<int>>);
		
		static int faceBoxOverlap(AABB3D, std::vector<Point3>);
		
		static std::vector<float> projection(std::vector<Point3>, Point3);
		static std::vector<float> projection(std::vector<Point3>, FPoint3);
		
		static double dot(FPoint3, Point3);
		static int dot(Point3 a, Point3 b);
		
	
	};
}


#endif /* CollisionDetection_hpp */

