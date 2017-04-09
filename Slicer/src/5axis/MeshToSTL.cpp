//
//  MeshToSTL.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 1/24/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#include "MeshToSTL.hpp"

namespace cura {
	
	void MeshToSTL::constructSTLfromMesh(const Mesh & mesh, std::string stlFilePath) {
		std::ofstream file;
		
		unsigned int twoByte = 0x0000;						//filler
		unsigned int size = mesh.faces.size();				//number of faces in mesh
		
		//writeLog(INFO, "converting mesh to STL file %s...", stlFilePath.c_str());
		
		if (getFileHandlerOut(file, stlFilePath)) {             // Check that we opened successfully
			for(int i = 0; i < 40; i++){						//write 80 byte header. header is unused
				file.write(reinterpret_cast<const char *>(&twoByte), 2);
			}
			
			file.write(reinterpret_cast<char*>(&size),4);
			
			std::vector<MeshFace> p_faces = mesh.faces;
			for (unsigned int i = 0; i < size; ++i) {		// Loop through all faces in the mesh
				Point3 vertex0 = mesh.vertices[p_faces[i].vertex_index[0]].p;
				Point3 vertex1 = mesh.vertices[p_faces[i].vertex_index[1]].p;
				Point3 vertex2 = mesh.vertices[p_faces[i].vertex_index[2]].p;
				
				//calculate normal of face
				Point3 BA = vertex1-vertex0;
				Point3 BC = vertex2-vertex0;
				FPoint3 normal = FPoint3::cross(BA, BC);
			
				float normalX =  normal.x;
				float normalY =  normal.y;
				float normalZ =  normal.z;
				float vertex0X = (float)vertex0.x/1000;
				float vertex0Y = (float)vertex0.y/1000;
				float vertex0Z = (float)vertex0.z/1000;
				float vertex1X = (float)vertex1.x/1000;
				float vertex1Y = (float)vertex1.y/1000;
				float vertex1Z = (float)vertex1.z/1000;
				float vertex2X = (float)vertex2.x/1000;
				float vertex2Y = (float)vertex2.y/1000;
				float vertex2Z = (float)vertex2.z/1000;
				
				file.write((char *)&normalX, 4);
				file.write((char *)&normalY, 4);
				file.write((char *)&normalZ, 4);
				file.write((char *)&vertex0X, 4);
				file.write((char *)&vertex0Y, 4);
				file.write((char *)&vertex0Z, 4);
				file.write((char *)&vertex1X, 4);
				file.write((char *)&vertex1Y, 4);
				file.write((char *)&vertex1Z, 4);
				file.write((char *)&vertex2X, 4);
				file.write((char *)&vertex2Y, 4);
				file.write((char *)&vertex2Z, 4);
				file.write((char *)&twoByte, 2);
			}
			
			file.close();
			
			return;
		} else {
			//writeLog(ERROR, "unable to open file %s [errno: %d]", stlFilePath.c_str(), strerror(errno));
			return;
		}
	}
	
	bool MeshToSTL::getFileHandlerOut(std::ofstream & file, std::string filePath) {
		file.open(filePath.c_str(), std::ios::out | std::ios::binary);
		
		return file.is_open();
	}
}
