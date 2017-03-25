//
//  main.cpp
//  5Axler_cura
//
//  Created by Hugh Whelan on 2/19/17.
//  Copyright © 2017 Hugh Whelan. All rights reserved.
//


#include "../cppunit/extensions/TestFactoryRegistry.h"
#include "../cppunit/ui/text/TestRunner.h"
#include "../cppunit"

int main(int argc,char** argv)
{

	CppUnit::TextUi::TestRunner runner;
	

	
	//Set the output type to be JUnit-style XML.
	std::ofstream output("output.xml");
	CppUnit::XmlOutputter* outputter = new CppUnit::XmlOutputter(&runner.result(), output);
	runner.setOutputter(outputter);
	
	CppUnit::TestFactoryRegistry& registry = CppUnit::TestFactoryRegistry::getRegistry();
	runner.addTest(registry.makeTest()); //Makes a test suite from all test cases that are registered with CPPUNIT_TEST_SUITE_REGISTRATION().
	bool success = runner.run("", false); //Run the tests!
	return success ? 0 : 1;
}
