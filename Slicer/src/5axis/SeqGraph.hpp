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
		Mesh mesh;
		
		/**
		 *The theta rotation to orient the sub-volume with build direction aligned with positive Z
		 */
		float theta;
		
		/**
		 *The phi rotation to orient the sub-volume with build direction aligned with positive Z
		 */
		float phi;
		
		SeqNode(Mesh mesh_c) : mesh(mesh_c){
			theta = 0;
			phi = 0;
		};
		
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
		void orientMesh();
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
		std::vector<std::vector<long int>> geometricChildren;
		
		/**
		 *vector of each node's collision children indices
		 */
		std::vector<std::vector<long int>> collisionChildren;
		
		/**
		 * returns the node of the graph with the specified index
		 *
		 * @param index the index of the node
		 *
		 * @return the node with specified index
		 */
		SeqNode & getNode(long int index){ return graphNodes[index]; }
		
		/**
		 *returns the geometric children of the node with given index
		 *
		 * @param index The index of the node
		 *
		 * @return the array of the node's geometric children
		 */
		std::vector<long int> getGeometricChildren(long int index){ return geometricChildren[index]; }
		
		/**
		 * returns the mesh of the node with the specified index
		 *
		 * @param index The index of the node
		 *
		 * @return the mesh of the node
		 */
		Mesh getMesh(long int index){ return graphNodes[index].getMesh(); }
		
		/**
		 * returns the number of nodes in the graph
		 */
		long int size(){ return graphNodes.size(); }
		
		/**
		 * adds a new node to the graph
		 *
		 * @param node the node reference to be added to the grap
		 */
		void addNode(SeqNode & node);
		
		/**
		 *Adds a Sequence Node as a geometric child
		 *
		 *@param parent The index of the parent node in the graph. Node represents the base upon which the child will be built.
		 *@param child The index of the child node in the graph. Node represents the overhanging sub-volume.
		 */
		void addGeometricChild(long int parent, long int child);
		
		/**
		 * Adds a Sequence Node as a collision child
		 *
		 * @param parent The index of the parent node in the graph.  Node represents the sub-vol which is would not be able to be printed because nozzle will collide with child.
		 * @param child The index of the child node in the graph. Node represents the sub-vol which will interfere with nozzle when printing the parent.
		 */
		void addCollisionChild(long int parent, long int child);
	private:
		
	};
}

#endif /* SeqGraph_hpp */
