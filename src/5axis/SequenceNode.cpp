//
//  SequenceNode.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 1/25/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#include "SequenceNode.hpp"

namespace cura {
	
	void SequenceNode::addGeometricChild(SequenceNode& child){
		geometricChildren.push_back(&child);
		size++;
		return;
	}
	
	
	void SequenceNode::addCollisionChild(SequenceNode& child){
		collisionChildren.push_back(&child);
		size++;
		return;
	}
	
	int SequenceNode::getId(){
		return id;
	}
}
