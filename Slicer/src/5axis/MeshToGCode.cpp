//
//  MeshToSTL.cpp
//  5Axler_cura
//
//  Created by Alexandre Pauwels on 1/24/17.
//  Copyright © 2017 Alexandre Pauwels. All rights reserved.
//

#include <unistd.h>
#include "MeshToSTL.hpp"

namespace cura {
	
void MeshToSTL::getGCodefromMesh(const Mesh & mesh) {
	pid_t pid = fork();

	if (pid == 0) {
		execl("../../build/CuraEngine", "slice -v -j ./5axistests/fdmprinter.def.json -o \"test.gcode\" -e0 -l \"./5axistests/simple.stl\"");
		exit(0);
	}
	else if (pid > 0) {

	} else {
		log("[ERROR] Could not fork process to slice sub-mesh")
	}
}

}
