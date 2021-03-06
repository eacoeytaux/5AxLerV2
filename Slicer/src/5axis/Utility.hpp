//
//  Utility.hpp
//  5AxLerV2
//
//  Created by Ethan Coeytaux on 2/7/17.
//  Copyright © 2017 mapmqp. All rights reserved.
//

#ifndef Utility_hpp
#define Utility_hpp

#include <cmath>
#include "../mesh.h"
#include "TransformationMatrix3D.hpp"

#define THETA_MAX 0.785398163397448309616 //in radians (should be between 0-pi)

#define A_AXIS_RANGE_DEGREES 90.0
#define A_AXIS_PRECISION_DEGREES 0.1
#define A_AXIS_DISCRETE_POINTS (int)(A_AXIS_RANGE_DEGREES / A_AXIS_PRECISION_DEGREES)

#define B_AXIS_PRECISION_DEGREES 0.1
#define B_AXIS_RANGE_DEGREES 360.0
#define B_AXIS_DISCRETE_POINTS (int)(B_AXIS_RANGE_DEGREES / B_AXIS_PRECISION_DEGREES)

// Defines multiple dimensional array types for representing matrices
typedef float (*Matrix3x1)[1];
typedef float (*Matrix3x8)[8];

namespace cura {
    /**
     * Computes the normal of the given face. Returns an FPoint3, maintains
     * float precision.
     *
     * @param mesh The mesh the face is in
     * @param face The face to compute the normal of
     *
     * @return An FPoint3 object which corresponds to the normal vector
     */
    static FPoint3 faceNormal(const Mesh& mesh, const MeshFace& face) {
        Point3 v0 = mesh.vertices[face.vertex_index[1]].p - mesh.vertices[face.vertex_index[0]].p;
        Point3 v1 = mesh.vertices[face.vertex_index[2]].p - mesh.vertices[face.vertex_index[1]].p;
        
        FPoint3 norm = FPoint3::cross(v0, v1);
        
        return norm;
    }
    
    /**
     * Computes the normal of the given face but truncates floats into ints
     *
     * @param mesh The mesh the face is in
     * @param face The face to compute the normal of
     *
     * @return A Point3 object which corresponds to the normal vector
     */
    static Point3 truncatedFaceNormal(const Mesh& mesh, const MeshFace& face) {
        FPoint3 norm = faceNormal(mesh, face);
        
        Point3 truncNorm = Point3((int)floor(norm.x), (int)floor(norm.y), (int)floor(norm.z));
        
        return truncNorm;
    }
    
    static bool Point3Equals(const Point3& p0, const Point3& p1, int tolerance = 0) {
        return ((fabs(p0.x - p1.x) <= tolerance) && (fabs(p0.y - p1.y) <= tolerance) && (fabs(p0.z - p1.z) <= tolerance));
    }

    static bool IntEquals(const long long int i1, const long long int i2, const int tolerance = 0) {
        return abs(i1 - i2) <= tolerance;
    }
    
    static double thetaFromCartesian(FPoint3 v) {
        return atan2(v.y, v.x); //http://keisan.casio.com/exec/system/1359533867
    }
    
    static double thetaFromCartesian(Point3 v) {
        return thetaFromCartesian(FPoint3(v.x, v.y, v.z));
    }
    
    static double phiFromCartesian(FPoint3 v) {
        return atan2(sqrt((v.x * v.x) + (v.y * v.y)), v.z); //http://keisan.casio.com/exec/system/1359533867
    }
    
    static double phiFromCartesian(Point3 v) {
        return phiFromCartesian(FPoint3(v.x, v.y, v.z));
    }
    
    static FPoint3 FPoint3FromSpherical(double theta, double phi, double r = 1) {
        return FPoint3(r * sin(phi) * cos(theta), r * sin(phi) * sin(theta), r * cos(phi)); //http://keisan.casio.com/exec/system/1359534351
    }
    
    static double radiansToDegrees(double radians) {
        return radians * 180.0 / M_PI;
    }
    
    static double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
	
	static FPoint3 rotateAroundXAxis(FPoint3 vector, float angle){
		return FPoint3( vector.x, vector.y*cosf(angle) - vector.z*sinf(angle), vector.y*sinf(angle) + vector.z*cosf(angle));
	}
	
	static FPoint3 rotateAroundYAxis(FPoint3 vector, float angle){
		return FPoint3(vector.z*sinf(angle) - vector.x*cosf(angle) , vector.y, vector.z*cosf(angle) - vector.x*sinf(angle));
	}
	
	static FPoint3 rotateAroundZAxis(FPoint3 vector, float angle){
		return FPoint3( vector.x*cosf(angle) - vector.y*sinf(angle),  vector.x*sinf(angle) + vector.y*cosf(angle),  vector.z);
	}
	
	/*
	static FPoint3 rotateAroundArbitraryLine(FPoint3 pOnLine, FPoint3 lineDirection, FPoint3 rotatingPoint, float angle){
		//pOnLine = <a,b,c>
		//lineDirection = <u,v,w>
		float u = lineDirection.x;
		float v = lineDirection.y;
		float w = lineDirection.z;
		float u2 = lineDirection.x;
		float v2 = lineDirection.y;
		float w2 = lineDirection.z;
		float l2 = u2 + v2 + w2;
		
		//the transformation matrix of the rotation
		TransformationMatrix3D transformation = TransformationMatrix3D();
		transformation.matrix[0][0] = u2+(v2+w2)*std::cosf(angle);
		transformation.matrix[1][0] = u*v*(1-(std::cosf(angle))) - w * (;
		transformation.matrix[2][0] = - std::sinf(angle);
		transformation.matrix[3][0] = 0.0;
		transformation.matrix[0][1] = 0.0;
		transformation.matrix[1][1] = 1.0;
		transformation.matrix[2][1] = 0.0;
		transformation.matrix[3][1] = 0.0;
		transformation.matrix[0][2] = std::sinf(angle);
		transformation.matrix[1][2] = 0.0;
		transformation.matrix[2][2] = std::cosf(angle);
		transformation.matrix[3][2] = 0.0;
		transformation.matrix[0][3] = 0.0;
		transformation.matrix[1][3] = 0.0;
		transformation.matrix[2][3] = 0.0;
		transformation.matrix[3][3] = 1.0;

	
	}
	 */
}

#endif /* Utility_hpp */
