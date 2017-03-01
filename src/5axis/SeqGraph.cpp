//
//  SeqGraph.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 2/8/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#include "SeqGraph.hpp"

namespace cura{
	
	Mesh SeqNode::rotateToOrigin(){
		//TODO
		return mesh;
	}
	
	void SeqGraph::addNode(SeqNode & node){
		graphNodes.push_back(node);
		return;
	}
	
	void SeqGraph::addGeometricChild(long int parent, long int child){
		geometricChildren.resize(parent+1);
		
		
		geometricChildren[parent].push_back(child);
		return;
	}
	
	void SeqGraph::addCollisionChild(long int parent, long int child){
		collisionChildren[parent].push_back(child);
		return;
	}
	
}
