//
//  OBB3D.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 2/14/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#include "OBB3D.hpp"

namespace cura{
	OBB3D::OBB3D( TransformationMatrix3D matrix ){
		transformation = matrix;
	}
}
