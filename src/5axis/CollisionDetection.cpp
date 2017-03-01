//
//  CollisionDetection.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 1/25/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#include "CollisionDetection.hpp"

using namespace ClipperLib;

namespace cura{
	void CollisionDetection::detectPrintCollisions(SeqGraph sequence){
		sequenceGraph = sequence; //store the entire graph (root) to be used in collision detections
		meshgroup = new MeshGroup(FffProcessor::getInstance());
		
		//recursiveAabb( sequence );
		
	}
	
	/*
	void CollisionDetection::recursiveAabb( SequenceNode & node){
		std::vector<SequenceNode> possibleCollisions;
		
		//Perform collision detection on one sub-volume against all others, starting from the root of the graph
		aabbCollisionDetection(node, sequenceGraph, possibleCollisions);
		
		for(int i = 0; i < possibleCollisions.size(); i++){
			sliceCollisionDetection(possibleCollisions[i], possibleCollisions);
		}
		
		
		
		//Perform collision detection on all geometric children of this sub-volume
		for(int i = 0; i < node.geometricChildren.size(); i++){
			recursiveAabb(*node.geometricChildren[i]);
		}
	}
	 */
	
	std::vector<int> CollisionDetection::aabbCollisionDetection(int printingIndex){
		std::vector<int> possibleCollisions;
		SeqNode & printingNode = sequenceGraph.getNode(printingIndex);
		
		AABB3D * unorientedBB = new AABB3D();
		OBB3D * OBB = new OBB3D(printingNode.getTransformation().getInverse());
		
		//loop through all points in printing node, transforming the mesh.  simultaneously compute an AABB to be used in the OBB
		for( int i = 0; i < printingNode.getMesh().vertices.size(); i++){
			MeshVertex vertex = printingNode.getMesh().vertices[i];
			vertex.p = printingNode.getTransformation().apply(vertex.p);
			unorientedBB->include(vertex.p);
		}
		
		OBB->aabb = *unorientedBB;
		OBB->aabb.max.z = std::numeric_limits<int32_t>::max();
		//TODO: find max Z direction and extend printingNode AABB to this z value
		
		for(int i = 0; i < sequenceGraph.size(); i++){
			AABB3D collidingNodeAABB = sequenceGraph.getNode(i).getMesh().getAABB();
			
			if( i != printingIndex && OBB->hit(collidingNodeAABB) ){
				possibleCollisions.push_back(i);
			}
		}
		
		return possibleCollisions;
		
	}
	
	void CollisionDetection::sliceCollisionDetection(int printingNode, std::vector<int> possibleCollisions){
		
		std::vector<int> verifiedCollisions;
		
		//the Mesh of the sub-volume that is being printed
		Mesh printingMesh = sequenceGraph.getMesh(printingNode);
		
		//setting variables needed to slice a mesh
		//TODO: Test that all these settings are correct for the mesh
		SliceDataStorage storage(meshgroup);
		int initial_layer_thickness = printingMesh.getSettingInMicrons("layer_height_0");
		int layer_thickness = printingMesh.getSettingInMicrons("layer_height");
		int initial_slice_z = initial_layer_thickness - layer_thickness / 2;
		int slice_layer_count = (storage.model_max.z - initial_slice_z) / layer_thickness + 1;
		int extended_slice_layer_count = slice_layer_count;  // this needs to be set to how many slices are needed for the sliced printing node
		
		
		//the transformation which moves the sub-volume to a orientation/position where it can be printed in the positive Z axis.
		//this must be applied to all meshes which are dealt with
		TransformationMatrix3D transformationMatrix = sequenceGraph.getNode(printingNode).getTransformation();
		
		//apply the transformation to the mesh being printed
		for(MeshVertex &vertex : printingMesh.vertices){
			vertex = transformationMatrix.apply(vertex.p);
		}
		
		//slice the mesh that is being printed regularly first
		Slicer printingSlicer = Slicer(&printingMesh, initial_layer_thickness, layer_thickness, extended_slice_layer_count, printingMesh.getSettingBoolean("meshfix_keep_open_polygons"), printingMesh.getSettingBoolean("meshfix_extensive_stitching"));

		//first slice is not modified, but every slice after is expanded to include the slice below
		for(int i = 1; i < extended_slice_layer_count; i++){
			SlicerLayer & topLayer = printingSlicer.layers[i];
			SlicerLayer & bottomLayer = printingSlicer.layers[i-1];
			
			if(printingSlicer.layers[i-1].polygons.size() == 0){
				log("[ERROR] COLLISION DETECTION - the special slicing has come across a previously processed layer which was empty.");
				return;
			}else if(printingSlicer.layers[i].polygons.size() == 0){ //this layer is above the actual height of the object, so will be the same as the layer below it.
				for( int m = 0; m < printingSlicer.layers[i-1].polygons.size(); m++){
					printingSlicer.layers[i].polygons.add(printingSlicer.layers[i-1].polygons[m]);
				}
			}else{
				double initialLayerArea = 0;
				double finalLayerArea = 0;
				for( Path path : topLayer.polygons){
					initialLayerArea += ClipperLib::Area(path);
				}
				
				topLayer.polygons = topLayer.polygons.unionPolygons(bottomLayer.polygons);
				
				for( Path path : topLayer.polygons){
					finalLayerArea += ClipperLib::Area(path);
				}
				
				if(initialLayerArea > finalLayerArea) //the next layer should always either stay the same or grow in area
					log("[ERROR] COLLISION DETECTION - A special slice layer is smaller than the one before it");
			}
		}
		
		
		//loop through each node (sub-volume) that the nozzle may collide with when printing the current mesh
		for( int collisionIndex : possibleCollisions){
			Mesh collisionMesh = sequenceGraph.getMesh(collisionIndex);
			
			//apply the transformation
			for(MeshVertex &vertex : collisionMesh.vertices){
				vertex = transformationMatrix.apply(vertex.p);
			}
			

			Slicer collisionSlicer = Slicer(&collisionMesh, initial_layer_thickness, layer_thickness, slice_layer_count, collisionMesh.getSettingBoolean("meshfix_keep_open_polygons"), collisionMesh.getSettingBoolean("meshfix_extensive_stitching"));
			
			//jump to the layer of the printing slicer where the z value is the same as the first populated slice of the collision slicer
			int startIndex = 0;
			while(printingSlicer.layers[startIndex].z != collisionSlicer.layers[0].z && startIndex <= printingSlicer.layers.size()) //ASSUMES THAT THE SLICER STARTS AT THE BASE OF AN OBJECT may be wrong
				startIndex++;
			
			if(startIndex >= printingSlicer.layers.size())
				log("[ERROR] COLLISION DETECTION - Could not find a matching z value to start testing for precise collisions");
			
			for(int n = startIndex; n < printingSlicer.layers.size(); n++){
				Polygons intersection = printingSlicer.layers[n].polygons.intersection(collisionSlicer.layers[n - startIndex].polygons);
				
				//if the intersection contains any polygons, there was a collision detected and we can move onto the next possible collision
				if(intersection.size() > 0){
					verifiedCollisions.push_back(collisionIndex);
					break;
				}
				
			}
		}
	}
		
	static std::vector<std::vector<int>> extractRotation(std::vector<std::vector<int>> fullTransformation){
		std::vector<std::vector<int>> rotationMatrix;
		std::vector<int> row;
		
		//create a default matrix
		rotationMatrix.push_back(row);
		rotationMatrix.push_back(row);
		rotationMatrix.push_back(row);
		rotationMatrix.push_back(row);
		rotationMatrix[0].push_back(1);
		rotationMatrix[0].push_back(0);
		rotationMatrix[0].push_back(0);
		rotationMatrix[0].push_back(0);
		rotationMatrix[1].push_back(0);
		rotationMatrix[1].push_back(1);
		rotationMatrix[1].push_back(0);
		rotationMatrix[1].push_back(0);
		rotationMatrix[2].push_back(0);
		rotationMatrix[2].push_back(0);
		rotationMatrix[2].push_back(1);
		rotationMatrix[2].push_back(0);
		rotationMatrix[3].push_back(0);
		rotationMatrix[3].push_back(0);
		rotationMatrix[3].push_back(0);
		rotationMatrix[3].push_back(1);
		
		//TODO: Find the correct transformation matrix
		
		return fullTransformation;
	}
	

	
	
	
	//http://stackoverflow.com/questions/17458562/efficient-aabb-triangle-intersection-in-c-sharp
	
	int CollisionDetection::faceBoxOverlap( AABB3D box, std::vector<Point3> face){
		float faceMin, faceMax;
		float boxMin, boxMax;
		
		//calculate normal of face
		Point3 BA = face[1]-face[0];
		Point3 BC = face[2]-face[0];
		FPoint3 faceNormal = FPoint3::cross(BA, BC);
		
		Point3 boxNormals[3];
		boxNormals[0] = Point3(0, 0, 1);
		boxNormals[1] = Point3(0, 1, 0);
		boxNormals[2] = Point3(1, 0, 0);
		
		
		//first 3 tests: normals of faces of aabb box
		std::vector<float> triangleMinAndMax = projection(face, boxNormals[2]);
		printf(":::::::::::: %f .. %d", triangleMinAndMax[1], box.min.x);
		if( triangleMinAndMax[1] < box.min.x || triangleMinAndMax[0] > box.max.x)
			return false;
		triangleMinAndMax = projection(face, boxNormals[1]);
		if( triangleMinAndMax[1] < box.min.y || triangleMinAndMax[0] > box.max.y)
			return false;
		triangleMinAndMax = projection(face, boxNormals[0]);
		if( triangleMinAndMax[1] < box.min.z || triangleMinAndMax[0] > box.max.z)
			return false;
		
		printf("---------------------------------Passed box normal test");
		
		std::vector<Point3> boxVertices;
		
		boxVertices.push_back(Point3(box.min.x, box.min.y, box.min.z));
		boxVertices.push_back(Point3(box.max.x, box.min.y, box.min.z));
		boxVertices.push_back(Point3(box.max.x, box.max.y, box.min.z));
		boxVertices.push_back(Point3(box.max.x, box.max.y, box.max.z));
		boxVertices.push_back(Point3(box.min.x, box.max.y, box.min.z));
		boxVertices.push_back(Point3(box.min.x, box.max.y, box.max.z));
		boxVertices.push_back(Point3(box.min.x, box.min.y, box.max.z));
		boxVertices.push_back(Point3(box.max.x, box.min.y, box.max.z));
		
		//test normal of the triangle
		double faceOffset = dot(faceNormal, face[0]);
		std::vector<float> boxMinAndMax = projection(boxVertices, faceNormal);
		if( boxMinAndMax[1] < faceOffset || boxMinAndMax[0] > faceOffset)
			return false;
		
		printf("---------------------------------Passed face normal test\n");
		
		//test nine edge cross-products
		std::vector<Point3> faceEdges = {face[0]-face[1],face[1]-face[2],face[2]-face[0]};
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				printf("               %i ----- %i \n", i,j);
				FPoint3 axis = FPoint3::cross(faceEdges[i],boxNormals[j]);
				
				std::vector<float> triangleMinAndMax = projection(face, axis);
				std::vector<float> boxMinAndMax = projection(boxVertices, axis);
				
				if(boxMinAndMax[1] <= triangleMinAndMax[0] || boxMinAndMax[0] >= triangleMinAndMax[1]){
					return false;
				}
			}
		}
		printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
		printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
		printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
		printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
		printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
		printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
		return true;
	}
	
	std::vector<float> CollisionDetection::projection(std::vector<Point3> points, Point3 axis){
		std::vector<float> minAndMax = {std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};
		
		for(Point3 point: points){
			float val = (axis.x * point.x) + (axis.y * point.y) + (axis.z * point.z);
			printf("-------------VAL: %f\n", val);
			
			if( val < minAndMax[0] )//val < min
				minAndMax[0] = val;
			if (val > minAndMax[1]) //val < max
				minAndMax[1] = val;
		}
		
		//printf("Face: {%d, %d, %d}, {%d, %d, %d}, {%d, %d, %d}\n", points[0].x, points[0].y, points[0].z, points[1].x, points[1].y, points[1].z, points[2].x, points[2].y, points[2].z);
		//printf("Normal: {%d, %d, %d}\n", axis.x, axis.y, axis.z);
		//printf("%f  ..  %f\n", minAndMax[0], minAndMax[1]);
		
		return minAndMax;
	}
	
	std::vector<float> CollisionDetection::projection(std::vector<Point3> points, FPoint3 axis){
		std::vector<float> minAndMax = {std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};
		
		for(Point3 point: points){
			float val = (axis.x * point.x) + (axis.y * point.y) + (axis.z * point.z);
			
			if (val < minAndMax[0])//val < min
				minAndMax[0] = val;
			if (val > minAndMax[1]) //val < max
				minAndMax[1] = val;
		}
		
		return minAndMax;
	}
	
	double CollisionDetection::dot(FPoint3 a, Point3 b){
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}
	
	int CollisionDetection::dot(Point3 a, Point3 b){
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}
}
