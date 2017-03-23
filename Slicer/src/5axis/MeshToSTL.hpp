//
//  MeshToSTL.hpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 1/24/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
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
	
	class MeshToSTL {
	public:
		/**
		 * Outputs and STL file of a Mesh
		 *
		 * @param mesh The mesh
		 * @param stlFilePath output file path
		 */
		static void constructSTLfromMesh(const Mesh &, std::string);
	private:
		/**
		 * Takes a pointer to a file handler and opens the STL
		 *
		 * @param file Point to the ofstream object
		 *
		 * @return true if success, false otherwise
		 */
		static bool getFileHandlerOut(std::ofstream &, std::string);
	};
}


#endif /* MeshToSTL_hpp */
