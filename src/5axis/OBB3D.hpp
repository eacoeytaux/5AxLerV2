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
#include "../utils/intpoint.h"
#include "../utils/AABB3D.h"
#include "../utils/floatpoint.h"
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
		
		/**
		 *Tests to see whether the OBB intersects with an AABB3D
		 *
		 * @param aabb the AABB3D which the OBB is being tested against for intersections
		 *
		 * @return boolean indicating whether or not the two bounding boxes collide
		 */
		bool hit(AABB3D aabb);
	};
}


#endif /* OBB3D_hpp */
