//
//  TestVolumeDecomposer.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 2/16/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//

#include "TestVolumeDecomposer.hpp"


using namespace std;

namespace cura{
	CPPUNIT_TEST_SUITE_REGISTRATION(TestVolumeDecomposer);
	
	void TestVolumeDecomposer::setUp(){
		string test = "machine_extruder_count";
		SettingsBase settings;
		settings.setSetting(test, "1");
		MeshGroup meshgroup(&settings);
	
		Mesh mesh = settings ? Mesh(settings) : Mesh(meshgroup); //If we have object_parent_settings, use them as parent settings. Otherwise, just use meshgroup.
	}

}
