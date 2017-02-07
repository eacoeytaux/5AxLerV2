//
//  CollisionDetection.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 1/25/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#include "CollisionDetection.hpp"
namespace cura{
	void CollisionDetection::detectPrintCollisions(SequenceNode & sequence){
		
		SequenceNode *currentPtr = &sequence;
		
		recursiveAabb( sequence );
	}
	
	
	bool CollisionDetection::checkMeshAndAABB(Mesh mesh, AABB3D aabb){
		for( int i = 0; i < mesh.vertices.size(); i++){
			for (int j = 0; j < 3; i++){
				//Point3 point = mesh.vertices[i][j];
				//if(true);
			}
		}
		
		return true;
	}
	
	void CollisionDetection::recursiveAabb( SequenceNode & node){
		//aabbCollisionDetection(node.mesh);
		
		for(int i = 0; i < node.geometricChildren.size(); i++){
			
			recursiveAabb(*node.geometricChildren[i]);
		}
	}
	
	/********************************************************/
	
	/* AABB-triangle overlap test code                      */
	
	/* by Tomas Akenine-Möller                              */
	
	/* Function: int triBoxOverlap(float boxcenter[3],      */
	
	/*          float boxhalfsize[3],float triverts[3][3]); */
	
	/* History:                                             */
	
	/*   2001-03-05: released the code in its first version */
	
	/*   2001-06-18: changed the order of the tests, faster */
	
	/*                                                      */
	
	/* Acknowledgement: Many thanks to Pierre Terdiman for  */
	
	/* suggestions and discussions on how to optimize code. */
	
	/* Thanks to David Hunt for finding a ">="-bug!         */
	
	/********************************************************/
/*
	int CollisionDetection::planeBoxOverlap(FPoint3 normal, FPoint3 vert, FPoint3 maxbox){
		FPoint3 vmin, vmax;
		
		if(normal.x > 0.0f){
			vmin.x =- maxbox.x - vert.x;
			vmax.x= maxbox.x - vert.x;
		}else{
			vmin.x= maxbox.x - vert.x;
			vmax.x=-maxbox.x - vert.x;
		}
		if(normal.y > 0.0f){
			vmin.y =- maxbox.y - vert.y;
			vmax.y= maxbox.y - vert.y;
		}else{
			vmin.x= maxbox.y - vert.y;
			vmax.x=-maxbox.y - vert.y;
		}
		if(normal.x > 0.0f){
			vmin.x =- maxbox.z - vert.z;
			vmax.x= maxbox.z - vert.z;
		}else{
			vmin.x= maxbox.z - vert.z;
			vmax.x=-maxbox.z - vert.z;
		}
		
		float dotMin = normal.x * vmin.x + normal.y * vmin.y + normal.z * vmin.z;
		float dotMax = normal.x * vmax.x + normal.y * vmax.y + normal.z * vmax.z;
		
		if(dotMin > 0.0f) return 0;	// -NJMP-
  		if(dotMax >= 0.0f) return 1;	// -NJMP-
		return 0;
	}
	*/
	
	int CollisionDetection::faceBoxOverlap( AABB3D box, std::vector<FPoint3> face){
		float faceMin, faceMax;
		float boxMin, boxMax;
		
		FPoint3 boxNormals[3];
		boxNormals[0] = FPoint3(0, 0, 1);
		boxNormals[1] = FPoint3(0, 1, 0);
		boxNormals[2] = FPoint3(1, 0, 0);
		
		//first 3 tests: normals of faces of aabb box
		std::vector<float> triangleMinAndMax = projection(face, boxNormals[0]);
		if( triangleMinAndMax[1] < box.min.x || triangleMinAndMax[0] > box.max.x)
			return false;
		triangleMinAndMax = projection(face, boxNormals[1]);
		if( triangleMinAndMax[1] < box.min.y || triangleMinAndMax[0] > box.max.y)
			return false;
		triangleMinAndMax = projection(face, boxNormals[2]);
		if( triangleMinAndMax[1] < box.min.z || triangleMinAndMax[0] > box.max.z)
			return false;
		
		//test normal of the triangle
		float triangleOffset = 
		
		
		
	}
	
	std::vector<float> CollisionDetection::projection(std::vector<FPoint3> points, FPoint3 axis){
		std::vector<float> minAndMax = {std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};
		
		for(int i = 0; i < 3; i++){
			float val = axis.x * points[i].x + axis.y * points[i].y + axis.y * points[i].y;
			
			if( val < minAndMax[0] )//val < min
				minAndMax[0] = val;
			if (val > minAndMax[1]) //val < max
				minAndMax[1] = val;
		}
		
		return minAndMax;
	}
}
