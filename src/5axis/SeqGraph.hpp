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
#include "TransformationMatrix3D.hpp"


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
		TransformationMatrix3D transformationToOrigin;
		
		/**
		 *The transformation which moves the mesh back to it's original orientation
		 */
		TransformationMatrix3D transformationToOriginal;
		
		/**
		 *Get the mesh of this node
		 *The mesh is the mesh of the sub-volume.  It is always in it's original orientation and position.
		 *
		 *@return the mesh
		 */
		Mesh getMesh(){ return mesh; }
		
		/**
		 *Get the transformation of this node
		 *The transformation is the transformation matrix which, when applied to the node, orients it so the build direction is along positive Z axis and base it at origin.
		 *
		 *@return the mesh
		 */
		TransformationMatrix3D getTransformation(){ return transformationToOrigin; }
		
		/**
		 *Rotates and transforms a mesh so that it's build direction is in the positive Z axis.
		 *
		 *@return the rotated Mesh.
		 */
		Mesh rotateToOrigin();
		
		/**
		 *Rotates and transforms a mesh so that it's in its original position
		 *
		 *@return the rotated Mesh.
		 */
		Mesh rotateToOriginal();
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
		 *returns the node of the graph with the specified index
		 *
		 * @param index the index of the node
		 *
		 * @return the node with specified index
		 */
		SeqNode getNode(int index){ return graphNodes[index]; }
		
		/**
		 *returns the geometric children of the node with given index
		 *
		 * @param index The index of the node
		 *
		 * @return the array of the node's geometric children
		 */
		std::vector<int> getGeometricChildren(int index){ return geometricChildren[index]; }
		
		/**
		 *returns the mesh of the node with the specified index
		 *
		 * @param index The index of the node
		 *
		 * @return the mesh of the node
		 */
		Mesh getMesh(int index){ return graphNodes[index].getMesh(); }
		
		/**
		 *returns the number of nodes in the graph
		 */
		long int size(){ return graphNodes.size(); }
		
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
