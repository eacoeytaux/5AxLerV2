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
	
	void SeqGraph::addGeometricChild(int parent, int child){
		geometricChildren[parent].push_back(child);
	}
	
	void SeqGraph::addCollisionChild(int parent, int child){
		collisionChildren[parent].push_back(child);
	}
	
}
