//
//  TransformationMatrix3D.hpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 2/14/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#ifndef TransformationMatrix3D_hpp
#define TransformationMatrix3D_hpp

#include <stdio.h>
#include "../utils/intpoint.h"


//this is very similar to cura's FMatrix3x3


namespace cura{
	class TransformationMatrix3D
	{
	public:
		/**
		 * The transformation matrix. 
		 * [0][0], [1][0], [2][0], [0][1], [1][1], [2][1], [0][2], [1][2], [2][2] make up the rotation matrix.
		 * [3][0], [3][1], [3][2] make up the translation vector
		 */
		double matrix[4][4];
		
		TransformationMatrix3D()
		{
			matrix[0][0] = 1.0;
			matrix[1][0] = 0.0;
			matrix[2][0] = 0.0;
			matrix[3][0] = 0.0;
			matrix[0][1] = 0.0;
			matrix[1][1] = 1.0;
			matrix[2][1] = 0.0;
			matrix[3][1] = 0.0;
			matrix[0][2] = 0.0;
			matrix[1][2] = 0.0;
			matrix[2][2] = 1.0;
			matrix[2][3] = 0.0;
			matrix[3][0] = 0.0;
			matrix[3][1] = 0.0;
			matrix[3][2] = 0.0;
			matrix[3][3] = 0.0;
		}
		
		/**
		 * Applies the transformation to a 3-dimensional point
		 *
		 * @param p the 3d point to which the transformation is applied
		 */
		Point3 apply(Point3& p) const
		{
			//adding 0.5 is to correct floating point truncation so eg. 1.99999 will round to 2
			int x = static_cast<int>(p.x * matrix[0][0] + p.y * matrix[1][0] + p.z * matrix[2][0] + 1 * matrix[3][0]) + 0.5;
			int y = static_cast<int>(p.x * matrix[0][1] + p.y * matrix[1][1] + p.z * matrix[2][1] + 1 * matrix[3][1]) + 0.5;
			int z = static_cast<int>(p.x * matrix[0][2] + p.y * matrix[1][2] + p.z * matrix[2][2] + 1 * matrix[3][2]) + 0.5;
			
			return Point3(x, y, z);
		}
		
		TransformationMatrix3D getInverse(){
			TransformationMatrix3D & inverse = *new TransformationMatrix3D();
			inverse.matrix[0][0] = matrix[0][0];
			inverse.matrix[1][0] = matrix[0][1];
			inverse.matrix[2][0] = matrix[0][2];
			inverse.matrix[0][1] = matrix[1][0];
			inverse.matrix[1][1] = matrix[1][1];
			inverse.matrix[2][1] = matrix[1][2];
			inverse.matrix[0][2] = matrix[2][0];
			inverse.matrix[1][2] = matrix[2][1];
			inverse.matrix[2][2] = matrix[2][2];
			inverse.matrix[3][0] = -matrix[3][0];
			inverse.matrix[3][1] = -matrix[3][0];
			inverse.matrix[3][2] = -matrix[3][0];
			return inverse;
			
		}
	};
}

#endif /* TransformationMatrix3D_hpp */
