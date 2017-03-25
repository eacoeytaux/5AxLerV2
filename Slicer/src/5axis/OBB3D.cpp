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
		aabb = *new AABB3D();
	}
	
	bool OBB3D::hit(AABB3D other){
		//Using the Separating Axis Theorem, must test 15 different axes
		//https://www.geometrictools.com/Documentation/DynamicCollisionDetection.pdf
		
		Point3 & otherX = *new Point3(1,0,0);
		Point3 & otherY = *new Point3(0,1,0);
		Point3 & otherZ = *new Point3(0,0,1);
		int otherHalfX = (other.max.x - other.min.x)/2;
		int otherHalfY = (other.max.y - other.min.z)/2;
		int otherHalfZ = (other.max.z - other.min.z)/2;
		Point3 & otherCenter = *new Point3(other.min.x + otherHalfX, other.min.y + otherHalfY, other.min.z + otherHalfZ);
		
		Point3 & thisX = *new Point3(1,0,0);
		Point3 & thisY = *new Point3(0,1,0);
		Point3 & thisZ = *new Point3(0,0,1);
		int thisHalfX = (aabb.max.x - aabb.min.x)/2;
		int thisHalfY = (aabb.max.y - aabb.min.z)/2;
		int thisHalfZ = (aabb.max.z - aabb.min.z)/2;
		Point3 & thisCenter = *new Point3(aabb.min.x + thisHalfX, aabb.min.y + thisHalfY, aabb.min.z + thisHalfZ);
		TransformationMatrix3D inverse = transformation.getInverse();
		inverse.apply(thisCenter);
		inverse.apply(thisX);
		inverse.apply(thisY);
		inverse.apply(thisZ);
		
		Point3 distance = *new Point3(otherCenter.x - thisCenter.x, otherCenter.y - thisCenter.y, otherCenter.z - thisCenter.z);
		
		//test 1
		long long int r0 = thisHalfX;
		long long int r1 = otherHalfX * (std::abs(thisX.dot(otherX))) + otherHalfY * (std::abs(thisX.dot(otherY))) + otherHalfY * (std::abs(thisX.dot(otherZ)));
		long long int r = std::abs( thisX.dot(distance) );
		
		if( r > r0 + r1 ){
			return false;
		}
		//test 2
		r0 = thisHalfY;
		r1 = otherHalfX * (std::abs(thisY.dot(otherX))) + otherHalfY * (std::abs(thisY.dot(otherY))) + otherHalfY * (std::abs(thisY.dot(otherZ)));
		r = std::abs( thisY.dot(distance));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 3
		r0 = thisHalfZ;
		r1 = otherHalfX * (std::abs(thisZ.dot(otherX))) + otherHalfY * (std::abs(thisZ.dot(otherY))) + otherHalfY * (std::abs(thisZ.dot(otherZ)));
		r = std::abs( thisZ.dot(distance));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 4
		r0 = thisHalfX * (std::abs(thisX.dot(otherX))) + thisHalfY * (std::abs(thisY.dot(otherX))) + thisHalfY * (std::abs(thisZ.dot(otherX)));
		r1 = otherHalfX;
		r = std::abs( otherX.dot(distance));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 5
		r0 = thisHalfX * (std::abs(thisX.dot(otherY))) + thisHalfY * (std::abs(thisY.dot(otherY))) + thisHalfY * (std::abs(thisZ.dot(otherY)));
		r1 = otherHalfY;
		r = std::abs( otherY.dot(distance));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 6
		r0 = thisHalfX * (std::abs(thisX.dot(otherZ))) + thisHalfY * (std::abs(thisY.dot(otherZ))) + thisHalfY * (std::abs(thisZ.dot(otherZ)));
		r1 = otherHalfZ;
		r = std::abs( otherZ.dot(distance));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 7
		r0 = thisHalfY * (std::abs(thisZ.dot(otherX))) + thisHalfZ * (std::abs(thisY.dot(otherX)));
		r1 = otherHalfY * (std::abs(thisX.dot(otherZ))) + otherHalfZ * (std::abs(thisX.dot(otherY)));
		r = std::abs((thisY.dot(otherX)) * (thisZ.dot(distance)) - (thisZ.dot(otherX)) * (thisY.dot(distance)));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 8
		r0 = thisHalfY * (std::abs(thisZ.dot(otherY))) + thisHalfZ * (std::abs(thisY.dot(otherY)));
		r1 = otherHalfX * (std::abs(thisX.dot(otherZ))) + otherHalfZ * (std::abs(thisX.dot(otherX)));
		r = std::abs((thisY.dot(otherY)) * (thisZ.dot(distance)) - (thisZ.dot(otherY)) * (thisY.dot(distance)));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 9
		r0 = thisHalfY * (std::abs(thisZ.dot(otherZ))) + thisHalfZ * (std::abs(thisY.dot(otherZ)));
		r1 = otherHalfX * (std::abs(thisX.dot(otherY))) + otherHalfY * (std::abs(thisX.dot(otherX)));
		r = std::abs((thisY.dot(otherZ)) * (thisZ.dot(distance)) - (thisZ.dot(otherZ)) * (thisY.dot(distance)));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 10
		r0 = thisHalfX * (std::abs(thisZ.dot(otherX))) + thisHalfZ * (std::abs(thisX.dot(otherX)));
		r1 = otherHalfY * (std::abs(thisY.dot(otherZ))) + otherHalfZ * (std::abs(thisY.dot(otherY)));
		r = std::abs((thisZ.dot(otherX)) * (thisX.dot(distance)) - (thisX.dot(otherX)) * (thisZ.dot(distance)));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 11
		r0 = thisHalfX * (std::abs(thisZ.dot(otherY))) + thisHalfZ * (std::abs(thisX.dot(otherY)));
		r1 = otherHalfX * (std::abs(thisY.dot(otherZ))) + otherHalfZ * (std::abs(thisY.dot(otherX)));
		r = std::abs((thisZ.dot(otherY)) * (thisX.dot(distance)) - (thisX.dot(otherY)) * (thisZ.dot(distance)));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 12
		r0 = thisHalfX * (std::abs(thisZ.dot(otherZ))) + thisHalfZ * (std::abs(thisX.dot(otherZ)));
		r1 = otherHalfX * (std::abs(thisY.dot(otherY))) + otherHalfY * (std::abs(thisY.dot(otherX)));
		r = std::abs((thisZ.dot(otherZ)) * (thisX.dot(distance)) - (thisX.dot(otherZ)) * (thisZ.dot(distance)));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 13
		r0 = thisHalfX * (std::abs(thisY.dot(otherX))) + thisHalfY * (std::abs(thisX.dot(otherX)));
		r1 = otherHalfY * (std::abs(thisZ.dot(otherZ))) + otherHalfZ * (std::abs(thisZ.dot(otherY)));
		r = std::abs((thisX.dot(otherX)) * (thisY.dot(distance)) - (thisY.dot(otherX)) * (thisX.dot(distance)));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 14
		r0 = thisHalfX * (std::abs(thisY.dot(otherY))) + thisHalfY * (std::abs(thisX.dot(otherY)));
		r1 = otherHalfX * (std::abs(thisZ.dot(otherZ))) + otherHalfZ * (std::abs(thisZ.dot(otherX)));
		r = std::abs((thisX.dot(otherY)) * (thisY.dot(distance)) - (thisY.dot(otherY)) * (thisX.dot(distance)));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		//test 15
		r0 = thisHalfX * (std::abs(thisY.dot(otherZ))) + thisHalfY * (std::abs(thisX.dot(otherZ)));
		r1 = otherHalfX * (std::abs(thisZ.dot(otherY))) + otherHalfY * (std::abs(thisZ.dot(otherX)));
		r = std::abs((thisX.dot(otherZ)) * (thisY.dot(distance)) - (thisY.dot(otherZ)) * (thisX.dot(distance)));
		
		if( r > r0 + r1 ){
			return false;
		}
		
		return true;
	}
}
