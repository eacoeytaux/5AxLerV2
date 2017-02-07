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

#endif /* SequenceNode_hpp */

#ifndef MeshToSTL_hpp
#define MeshToSTL_hpp

namespace cura {
	
	class SequenceNode {
	public:
		Mesh mesh;
		std::vector<SequenceNode*> geometricChildren;
		std::vector<SequenceNode*> collisionChildren;
		int size;
		
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
	private:
		
	};
}


#endif /* MeshToSTL_hpp */
