//
//  TestVolumeDecomposer.hpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 2/16/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#ifndef TestVolumeDecomposer_hpp
#define TestVolumeDecomposer_hpp

#include <stdio.h>
#include "../cppunit/TestFixture.h"
#include "../cppunit/extensions/HelperMacros.h"

#include "../src/settings/settings.h"
#include "../src/MeshGroup.h"

namespace cura{
	
class TestVolumeDecomposer : public CppUnit::TestFixture{
public:
	/*!
	 * Set up test suite.  Creates an instance of a VolumeDecomposer.
	 */
	void setUp();
	
	/*!
	 * Destroys the VolumeDecomposer
	 */
	void tearDown();
	
	//The actual test cases.
	void validateSeperatedMeshes();
};

}
#endif /* TestVolumeDecomposer_hpp */
