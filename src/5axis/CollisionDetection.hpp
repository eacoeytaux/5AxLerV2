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

#include "SequenceNode.hpp"
#include "../mesh.h"

namespace cura {
	
	class CollisionDetection {
	public:
		/**
		 * Finds all collisions in a sequence of sub-meshes
		 *
		 * @param sequence parent node of sequence graph
		 */
		void detectPrintCollisions(SequenceNode &);
		
		bool checkMeshAndAABB(Mesh, AABB3D);
	private:
		/**
		 * Finds all aabb collision on on a node, the recursively calls itself on all children
		 *
		 * @param node current node of sequence graph
		 */
		void recursiveAabb( SequenceNode &);
		
		/**
		 * Finds all collision possibilites using axis-aligned bounded boxes
		 *
		 * @param printingMesh Mesh of node which is being printed in collision situation
		 * @param collidingMesh Mesh of node which may be colliding with the nozzle
		 *
		 * @return bool True if there is a collision detected between the two Mesh's, False if not
		 */
		void aabbCollisionDetection(Mesh, Mesh);
		
		/**
		 * Deterministically detects any collsions that may occur during the print
		 *
		 * @param printingMesh Mesh of node which is being printed in collision situation
		 * @param collidingMesh Mesh of node which may be colliding with the nozzle
		 *
		 * @return bool True if there is a collision detected between the two Mesh's, False if not
		 */
		void preciseCollisionDetection(Mesh, Mesh);
		
		int faceBoxOverlap( AABB3D, std::vector<FPoint3>);
		
		/**
		 *  
		 *
		 */
		std::vector<float> projection(std::vector<FPoint3>, FPoint3);
	};
}


#endif /* CollisionDetection_hpp */

