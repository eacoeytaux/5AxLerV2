//
//  OBB3D.hpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 2/14/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#ifndef OBB3D_hpp
#define OBB3D_hpp

#include <stdio.h>
#include "intpoint.h"
#include "AABB3D.h"
#include "floatpoint.h"
#include "TransformationMatrix3D.hpp"

namespace cura{
	class OBB3D{
	public:
		/**
		 * the bounding box which encloses the sub-volume mesh after being rotated to the origin
		 */
		AABB3D aabb;
		
		/**
		 * The transformation of the aabb to make it an oriented bounding box
		 */
		TransformationMatrix3D transformation;
		
		/**
		 * initializes a OBB with only the transformation
		 */
		OBB3D(TransformationMatrix3D matrix);
	};
}


#endif /* OBB3D_hpp */
