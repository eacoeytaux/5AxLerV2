//
//  CollisionDetection.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 1/25/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#include "CollisionDetection.hpp"
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
		AABB3D printingNodeAABB = sequenceGraph.getNode(printingIndex).getMesh().getAABB();
		
		//TODO: rotate everything so printingNode build direction is +Z
		
		//TODO: find max Z direction and extend printingNode AABB to this z value
		
		for(int i = 0; i < sequenceGraph.size(); i++){
			AABB3D collidingNodeAABB = sequenceGraph.getNode(i).getMesh().getAABB();
			
			if( i != printingIndex && printingNodeAABB.hit(collidingNodeAABB) ){
				possibleCollisions.push_back(i);
			}
		}
		/*
		for(int i = 0; i < collidingNode.geometricChildren.size(); i++){
			aabbCollisionDetection(printingNode, *collidingNode.geometricChildren[i], possibleCollisions);
		}
		 */
		
		return possibleCollisions;
		
	}
	
	void CollisionDetection::sliceCollisionDetection(int printingNode, std::vector<int> possibleCollisions){
		
		Mesh currentMesh = sequenceGraph.getMesh(printingNode);
		
		//TODO: Get this matrix from the current nodes build direction
		FMatrix3x3 transformationMatrix = FMatrix3x3();
		transformationMatrix.m[0][0] = cos(3.14159265);
		transformationMatrix.m[1][0] = -sin(3.14159265);
		transformationMatrix.m[2][0] = 0;
		transformationMatrix.m[0][0] = sin(3.14159265);
		transformationMatrix.m[1][0] = cos(3.14159265);
		transformationMatrix.m[2][0] = 0;
		transformationMatrix.m[0][0] = 0;
		transformationMatrix.m[1][0] = 0;
		transformationMatrix.m[2][0] = 1;
		
		//TODO: Slice the current mesh collision detection style
		
		
		//loop through each node (sub-volume) that the nozzle may collide with when printing the current mesh
		for( int collisionIndex : possibleCollisions){
			Mesh collisionMesh = sequenceGraph.getMesh(collisionIndex);
			
			//apply the transformation
			//TODO: Apply this transformation to reference, not copy, then account for this transformation so that it doesnt have to be undone but instead worked over
			for(MeshVertex &vertex : collisionMesh.vertices){
				vertex = transformationMatrix.apply(vertex.p);
			}
			
			SliceDataStorage storage(meshgroup);
			
			int initial_layer_thickness = collisionMesh.getSettingInMicrons("layer_height_0");
			int layer_thickness = collisionMesh.getSettingInMicrons("layer_height");
			int initial_slice_z = initial_layer_thickness - layer_thickness / 2;
			int slice_layer_count = (storage.model_max.z - initial_slice_z) / layer_thickness + 1;

			Slicer slicer = Slicer(&collisionMesh, initial_layer_thickness, layer_thickness, slice_layer_count, collisionMesh.getSettingBoolean("meshfix_keep_open_polygons"), collisionMesh.getSettingBoolean("meshfix_extensive_stitching"));
			
			
			//TODO: Test that all these settings are correct for the mesh
		}
		
	}
	
	//based off of cura's slicer (copied, than modified)
	void CollisionDetection::specialSlice(Mesh* mesh, int initial, int thickness, int slice_layer_count, bool keep_none_closed, bool extensive_stitching){
		std::vector<SlicerLayer> layers;
		//const Mesh* slicedMesh = nullptr; //!< The sliced mesh
		
		//CURA CODE
		assert(slice_layer_count > 0);
		
		TimeKeeper slice_timer;
		
		layers.resize(slice_layer_count);
		//END CURA CODE
		
		//need to go one layer at a time
		for(int32_t layer_nr = 0; layer_nr < slice_layer_count; layer_nr++)
		{
			layers[layer_nr].z = initial + thickness * layer_nr;
			
			for(unsigned int mesh_idx = 0; mesh_idx < mesh->faces.size(); mesh_idx++)
			{
				
			}
		}
		
		
		//CURA CODE
		for(unsigned int mesh_idx = 0; mesh_idx < mesh->faces.size(); mesh_idx++)
		{
			const MeshFace& face = mesh->faces[mesh_idx];
			const MeshVertex& v0 = mesh->vertices[face.vertex_index[0]];
			const MeshVertex& v1 = mesh->vertices[face.vertex_index[1]];
			const MeshVertex& v2 = mesh->vertices[face.vertex_index[2]];
			Point3 p0 = v0.p;
			Point3 p1 = v1.p;
			Point3 p2 = v2.p;
			int32_t minZ = p0.z;
			int32_t maxZ = p0.z;
			if (p1.z < minZ) minZ = p1.z;
			if (p2.z < minZ) minZ = p2.z;
			if (p1.z > maxZ) maxZ = p1.z;
			if (p2.z > maxZ) maxZ = p2.z;
			int32_t layer_max = (maxZ - initial) / thickness;
			for(int32_t layer_nr = (minZ - initial) / thickness; layer_nr <= layer_max; layer_nr++)
			{
				int32_t z = layer_nr * thickness + initial;
				if (z < minZ) continue;
				if (layer_nr < 0) continue;
				
				SlicerSegment s;
				s.endVertex = nullptr;
				int end_edge_idx = -1;
				if (p0.z < z && p1.z >= z && p2.z >= z)
				{
					s = project2D(p0, p2, p1, z);
					end_edge_idx = 0;
					if (p1.z == z)
					{
						s.endVertex = &v1;
					}
				}
				else if (p0.z > z && p1.z < z && p2.z < z)
				{
					s = project2D(p0, p1, p2, z);
					end_edge_idx = 2;
					
				}
				
				else if (p1.z < z && p0.z >= z && p2.z >= z)
				{
					s = project2D(p1, p0, p2, z);
					end_edge_idx = 1;
					if (p2.z == z)
					{
						s.endVertex = &v2;
					}
				}
				else if (p1.z > z && p0.z < z && p2.z < z)
				{
					s = project2D(p1, p2, p0, z);
					end_edge_idx = 0;
					
				}
				
				else if (p2.z < z && p1.z >= z && p0.z >= z)
				{
					s = project2D(p2, p1, p0, z);
					end_edge_idx = 2;
					if (p0.z == z)
					{
						s.endVertex = &v0;
					}
				}
				else if (p2.z > z && p1.z < z && p0.z < z)
				{
					s = project2D(p2, p0, p1, z);
					end_edge_idx = 1;
				}
				else
				{
					//Not all cases create a segment, because a point of a face could create just a dot, and two touching faces
					//  on the slice would create two segments
					continue;
				}
				layers[layer_nr].face_idx_to_segment_idx.insert(std::make_pair(mesh_idx, layers[layer_nr].segments.size()));
				s.faceIndex = mesh_idx;
				s.endOtherFaceIdx = face.connected_face_index[end_edge_idx];
				s.addedToPolygon = false;
				layers[layer_nr].segments.push_back(s);
			}
		}
		log("slice of mesh took %.3f seconds\n",slice_timer.restart());
		for(unsigned int layer_nr=0; layer_nr<layers.size(); layer_nr++)
		{
			layers[layer_nr].makePolygons(mesh, keep_none_closed, extensive_stitching);
		}
		mesh->expandXY(mesh->getSettingInMicrons("xy_offset"));
		log("slice make polygons took %.3f seconds\n",slice_timer.restart());
	}
	//END CURA CODE
	
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
