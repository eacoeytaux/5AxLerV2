//
//  5AxLerTests.hpp
//  5AxLerV2
//
//  Created by Ethan Coeytaux on 2/27/17.
//  Copyright © 2017 mapmqp. All rights reserved.
//

#ifndef _AxLerTests_hpp
#define _AxLerTests_hpp

#include "../../libs/cppunit/TestCase.h"
#include "../../libs/cppunit/TestSuite.h"
#include "../../libs/cppunit/TestCaller.h"
#include "../../libs/cppunit/TestRunner.h"

class BuildMapTestCase : public CppUnit::TestCase {
public:
    // constructor - Note 3
    StudentTestCase(std::string name) : TestCase(name) {}
    
    // method to test the constructor
    void testConstructor();
    
    // method to test the assigning and retrieval of grades
    void testAssignAndRetrieveGrades();
    
    // method to create a suite of tests
    static Test *suite ();
};

#endif /* _AxLerTests_hpp */
