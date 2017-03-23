//
//  MeshToGCode.hpp
//  5Axler_cura
//
//  Created by Alexandre Pauwels on 1/24/17.
//  Copyright © 2017 Alexandre Pauwels. All rights reserved.
//

#ifndef MeshToSTL_hpp
#define MeshToSTL_hpp

#include <string>
#include <stdio.h>
#include <stdio.h>
#include <iostream>
#include <fstream>


#include "../mesh.h"

namespace cura {
	
	class MeshToGCode {
	public:
		/**
		 * Outputs and STL file of a Mesh
		 *
		 * @param mesh The mesh
		 * @param stlFilePath output file path
		 */
		static void getGCodeFromMesh(const Mesh &);
	};
}


#endif /* MeshToSTL_hpp */
