//
//  SeqGraph.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 2/8/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#include "SeqGraph.hpp"

namespace cura{
	
	void SeqNode::orientMesh(){
		//create transformation matrices for the two rotations
		TransformationMatrix3D rotateAroundY = TransformationMatrix3D();
		TransformationMatrix3D rotateAroundX = TransformationMatrix3D();
		rotateAroundY.rotateAroundYAxis(theta);
		rotateAroundX.rotateAroundXAxis(phi);
		
		//first apply rotation around Y axis
		for(MeshVertex & vertex : mesh.vertices){
			vertex.p= rotateAroundY.apply(vertex.p);
		}
		
		//apply rotation around X axis
		for(MeshVertex & vertex : mesh.vertices){
			vertex.p=rotateAroundX.apply(vertex.p);
		}
		return;
	}
	
	void SeqGraph::addNode(SeqNode & node){
		graphNodes.push_back(node);
		return;
	}
	
	void SeqGraph::addGeometricChild(long int parent, long int child){
		geometricChildren.resize(parent+1);
		
		
		geometricChildren[parent].push_back(child);
		return;
	}
	
	void SeqGraph::addCollisionChild(long int parent, long int child){
		collisionChildren[parent].push_back(child);
		return;
	}
	
	void SeqGraph::outputGraphJSON(){
		std::ofstream outputFile;
		outputFile.open ("sequence_data.json");
		outputFile << "{\n	\"subvolumes\":[\n";
		for(int i = 0; i < graphNodes.size(); i++){
			outputFile << "		{\"filename\": output_decomp_";
			outputFile << i;
			outputFile << ", \"sequence\":";
			outputFile << graphNodes[i].sequence;
			outputFile << ", \"theta\":";
			outputFile << graphNodes[i].theta;
			outputFile << ", \"phi\":";
			outputFile << graphNodes[i].phi;
			outputFile << "}";
			if( i != graphNodes.size()-1){
				outputFile << "'";
			}
			outputFile << "\n";
		}
		outputFile << "}";
		outputFile.close();
		}
}
