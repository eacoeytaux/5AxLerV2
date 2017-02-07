//
//  SequenceNode.hpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 1/25/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#ifndef SequenceNode_hpp
#define SequenceNode_hpp

#include <stdio.h>

#include "../mesh.h"

namespace cura {
	
	class SequenceNode {
	public:
		Mesh mesh;
		FMatrix3x3 transformation;
		std::vector<SequenceNode*> geometricChildren;
		std::vector<SequenceNode*> collisionChildren;
		int size;
		int id;
		
		/**
		 *Adds a Sequence Node as a direct child
		 *
		 *@param child reference to node which is a geometric child
		 */
		void addGeometricChild(SequenceNode& child);
		
		/**
		 *Adds a Sequence Node as a direct child
		 *
		 *@param child reference to node which is a geometric child
		 */
		void addCollisionChild(SequenceNode& child);
		
		int getId();
	private:
		
	};
}


#endif /* SequenceNode_hpp */
