//
//  SeqGraph.hpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 2/8/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#ifndef SeqGraph_hpp
#define SeqGraph_hpp

#include <stdio.h>
#include "../mesh.h"


namespace cura {
	
	class SeqNode {
	public:
		/**
		 *The unique ID which is the index in the graph's collection of nodes
		 */
		int Index;
		
		/**
		 *The mesh of the sub-volume represented by this node.
		 */
		Mesh & mesh;
		
		
		/**
		 *The transformation necessary to print sub-volume in positive z-axis at origin
		 */
		FMatrix3x3 transformation;
		
		/**
		 *Get the mesh of this node
		 *The mesh is the mesh of the sub-volume.  It is always in it's original orientation and position.
		 *
		 *@return the mesh
		 */
		Mesh getMesh(){ return mesh; }
		
		/**
		 *Rotates and transforms a mesh so that it's build direction is in the positive Z axis.
		 *
		 *@return the rotated Mesh.
		 */
		Mesh rotateToOrigin();
	};
	
	class SeqGraph {
	public:
		/**
		 *Vector of all nodes in the graph.
		 */
		std::vector<SeqNode> graphNodes;
		
		/**
		 *vector of each node's geometric children indices
		 */
		std::vector<std::vector<int>> geometricChildren;
		
		/**
		 *vector of each node's collision children indices
		 */
		std::vector<std::vector<int>> collisionChildren;
		
		/**
		 *Adds a Sequence Node as a geometric child
		 *
		 *@param parent The index of the parent node in the graph. Node represents the base upon which the child will be built.
		 *@param child The index of the child node in the graph. Node represents the overhanging sub-volume.
		 */
		void addGeometricChild(int parent, int child);
		
		/**
		 *Adds a Sequence Node as a collision child
		 *
		 *@param parent The index of the parent node in the graph.  Node represents the sub-vol which is would not be able to be printed because nozzle will collide with child.
		 *@param child The index of the child node in the graph. Node represents the sub-vol which will interfere with nozzle when printing the parent.
		 */
		void addCollisionChild(int parent, int child);
	private:
		
	};
}

#endif /* SeqGraph_hpp */
