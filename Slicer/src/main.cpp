/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <vector>
#include <sys/time.h>
#include <iterator>
#include <signal.h>
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
#include <execinfo.h>
#include <sys/resource.h>
#endif
#include <stddef.h>
#include <vector>

#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "utils/string.h"

#include "FffProcessor.h"
#include "settings/SettingRegistry.h"

#include "settings/SettingsToGV.h"

// CUSTOM CODE
#include "5axis/MeshToSTL.hpp"
#include "5axis/path_smoothing/PathSmoother.hpp"
#include "5axis/TransformationMatrix3D.hpp"
#include "5axis/OBB3D.hpp"
#include "5axis/VolumeDecomposer.hpp"
#include "5axis/BuildMap.hpp"
#include "5axis/Utility.hpp"
// END CUSTOM CODE

namespace cura
{
    
void print_usage()
{
    logAlways("\n");
    logAlways("usage:\n");
    logAlways("CuraEngine help\n");
    logAlways("\tShow this help message\n");
    logAlways("\n");
    logAlways("CuraEngine connect <host>[:<port>] [-j <settings.def.json>]\n");
    logAlways("  --connect <host>[:<port>]\n\tConnect to <host> via a command socket, \n\tinstead of passing information via the command line\n");
    logAlways("  -j<settings.def.json>\n\tLoad settings.json file to register all settings and their defaults\n");
    logAlways("  -v\n\tIncrease the verbose level (show log messages).\n");
    logAlways("\n");
    logAlways("CuraEngine slice [-v] [-p] [-j <settings.json>] [-s <settingkey>=<value>] [-g] [-e<extruder_nr>] [-o <output.gcode>] [-l <model.stl>] [--next]\n");
    logAlways("  -v\n\tIncrease the verbose level (show log messages).\n");
    logAlways("  -p\n\tLog progress information.\n");
    logAlways("  -j\n\tLoad settings.def.json file to register all settings and their defaults.\n");
    logAlways("  -s <setting>=<value>\n\tSet a setting to a value for the last supplied object, \n\textruder train, or general settings.\n");
    logAlways("  -l <model_file>\n\tLoad an STL model. \n");
    logAlways("  -g\n\tSwitch setting focus to the current mesh group only.\n\tUsed for one-at-a-time printing.\n");
    logAlways("  -e<extruder_nr>\n\tSwitch setting focus to the extruder train with the given number.\n");
    logAlways("  --next\n\tGenerate gcode for the previously supplied mesh group and append that to \n\tthe gcode of further models for one-at-a-time printing.\n");
    logAlways("  -o <output_file>\n\tSpecify a file to which to write the generated gcode.\n");
    logAlways("\n");
    logAlways("The settings are appended to the last supplied object:\n");
    logAlways("CuraEngine slice [general settings] \n\t-g [current group settings] \n\t-e0 [extruder train 0 settings] \n\t-l obj_inheriting_from_last_extruder_train.stl [object settings] \n\t--next [next group settings]\n\t... etc.\n");
    logAlways("\n");
    logAlways("In order to load machine definitions from custom locations, you need to create the environment variable CURA_ENGINE_SEARCH_PATH, which should contain all search paths delimited by a (semi-)colon.\n");
    logAlways("\n");
}

//Signal handler for a "floating point exception", which can also be integer division by zero errors.
void signal_FPE(int n)
{
    (void)n;
    cura::logError("Arithmetic exception.\n");
    exit(1);
}

void print_call(int argc, char **argv)
{
    cura::logError("Command called:\n");
    for (int idx= 0; idx < argc; idx++)
        cura::logError("%s ", argv[idx]);
    cura::logError("\n");
}

void connect(int argc, char **argv)
{
    std::string ip;
    int port = 49674;

    // parse ip port
    std::string ip_port(argv[2]);
    if (ip_port.find(':') != std::string::npos)
    {
        ip = ip_port.substr(0, ip_port.find(':'));
        port = std::stoi(ip_port.substr(ip_port.find(':') + 1).data());
    }

    for(int argn = 3; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            for(str++; *str; str++)
            {
                switch(*str)
                {
                case 'v':
                    cura::increaseVerboseLevel();
                    break;
                case 'j':
                    argn++;
                    if (SettingRegistry::getInstance()->loadJSONsettings(argv[argn], FffProcessor::getInstance()))
                    {
                        cura::logError("Failed to load json file: %s\n", argv[argn]);
                        std::exit(1);
                    }
                    break;
                default:
                    cura::logError("Unknown option: %c\n", *str);
                    print_call(argc, argv);
                    print_usage();
                    break;
                }
            }
        }
    }

    CommandSocket::instantiate();
    CommandSocket::getInstance()->connect(ip, port);
}


void slice(int argc, char **argv)
{   
    FffProcessor::getInstance()->time_keeper.restart();
    
    FMatrix3x3 transformation; // the transformation applied to a model when loaded
                        
    MeshGroup* meshgroup = new MeshGroup(FffProcessor::getInstance());
	
    int extruder_train_nr = 0;

    SettingsBase* last_extruder_train = nullptr;
    // extruder defaults cannot be loaded yet cause no json has been parsed
    SettingsBase* last_settings_object = FffProcessor::getInstance();
    for(int argn = 2; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            if (str[1] == '-')
            {
                if (stringcasecompare(str, "--next") == 0)
                {
                    try {
                        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
                        // Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
                        log("Loaded from disk in %5.3fs\n", FffProcessor::getInstance()->time_keeper.restart());
                        
                        for (int extruder_nr = 0; extruder_nr < FffProcessor::getInstance()->getSettingAsCount("machine_extruder_count"); extruder_nr++)
                        { // initialize remaining extruder trains and load the defaults
                            meshgroup->createExtruderTrain(extruder_nr); // create new extruder train objects or use already existing ones
                        }

                        meshgroup->finalize();

                        //start slicing
                        FffProcessor::getInstance()->processMeshGroup(meshgroup);
                        
                        // initialize loading of new meshes
                        FffProcessor::getInstance()->time_keeper.restart();
                        delete meshgroup;
                        meshgroup = new MeshGroup(FffProcessor::getInstance());
                        last_extruder_train = meshgroup->createExtruderTrain(0); 
                        last_settings_object = meshgroup;
                        
                    }catch(...){
                        cura::logError("Unknown exception\n");
                        exit(1);
                    }
                }else{
                    cura::logError("Unknown option: %s\n", str);
                }
            }else{
                for(str++; *str; str++)
                {
                    switch(*str)
                    {
                    case 'v':
                        cura::increaseVerboseLevel();
                        break;
                    case 'p':
                        cura::enableProgressLogging();
                        break;
                    case 'j':
                        argn++;
                        if (SettingRegistry::getInstance()->loadJSONsettings(argv[argn], last_settings_object))
                        {
                            cura::logError("Failed to load json file: %s\n", argv[argn]);
                            std::exit(1);
                        }
                        break;
                    case 'e':
                        str++;
                        extruder_train_nr = int(*str - '0'); // TODO: parse int instead (now "-e10"="-e:" , "-e11"="-e;" , "-e12"="-e<" .. etc) 
                        last_settings_object = meshgroup->createExtruderTrain(extruder_train_nr);
                        last_extruder_train = last_settings_object;
                        break;
                    case 'l':
                        argn++;
                        
                        log("Loading %s from disk...\n", argv[argn]);

                        transformation = last_settings_object->getSettingAsPointMatrix("mesh_rotation_matrix"); // the transformation applied to a model when loaded

                        if (!last_extruder_train)
                        {
                            last_extruder_train = meshgroup->createExtruderTrain(0); // assume a json has already been provided on the command line
                        }
                        if (!loadMeshIntoMeshGroup(meshgroup, argv[argn], transformation, last_extruder_train))
                        {
                            logError("Failed to load model: %s\n", argv[argn]);
                            std::exit(1);
                        }
                        else 
                        {
                            last_settings_object = &(meshgroup->meshes.back()); // pointer is valid until a new object is added, so this is OK
                        }
                        break;
                    case 'o':
                        argn++;
                        if (!FffProcessor::getInstance()->setTargetFile(argv[argn]))
                        {
                            cura::logError("Failed to open %s for output.\n", argv[argn]);
                            exit(1);
                        }
                        break;
                    case 'g':
                        last_settings_object = meshgroup;
                    case 's':
                        {
                            //Parse the given setting and store it.
                            argn++;
                            char* valuePtr = strchr(argv[argn], '=');
                            if (valuePtr)
                            {
                                *valuePtr++ = '\0';

                                last_settings_object->setSetting(argv[argn], valuePtr);
                            }
                        }
                        break;
                    default:
                        cura::logError("Unknown option: %c\n", *str);
                        print_call(argc, argv);
                        print_usage();
                        exit(1);
                        break;
                    }
                }
            }
        }
        else
        {
            
            cura::logError("Unknown option: %s\n", argv[argn]);
            print_call(argc, argv);
            print_usage();
            exit(1);
        }
    }

    int extruder_count = FffProcessor::getInstance()->getSettingAsCount("machine_extruder_count");
    for (extruder_train_nr = 0; extruder_train_nr < extruder_count; extruder_train_nr++)
    { // initialize remaining extruder trains and load the defaults
        meshgroup->createExtruderTrain(extruder_train_nr); // create new extruder train objects or use already existing ones
    }
    
    
#ifndef DEBUG
    try {
#endif
        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
        // Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
        meshgroup->finalize();
        log("Loaded from disk in %5.3fs\n", FffProcessor::getInstance()->time_keeper.restart());
		
		
        //start slicing
        FffProcessor::getInstance()->processMeshGroup(meshgroup);

#ifndef DEBUG
    }catch(...){
        cura::logError("Unknown exception\n");
        exit(1);
    }
#endif
    //Finalize the processor, this adds the end.gcode. And reports statistics.
    FffProcessor::getInstance()->finalize();
	
	
	
	
	
	//MeshToSTL::constructSTLfromMesh(meshgroup->meshes[0], "output.stl");
	
	/*
	printf("\nALL OF THE FACES OF THE MESH:------------------------------");
	for ( int k = 0; k < meshgroup->meshes[0].faces.size(); k++){
		MeshFace currentFace = meshgroup->meshes[0].faces[k];
		MeshVertex v0 = meshgroup->meshes[0].vertices[currentFace.vertex_index[0]];
		MeshVertex v1 = meshgroup->meshes[0].vertices[currentFace.vertex_index[2]];
		MeshVertex v2 = meshgroup->meshes[0].vertices[currentFace.vertex_index[2]];

		printf("-FACE ID: %i\n	V0: %i\n",  k, currentFace.vertex_index[0]);
		printf("		connected to: ");
		for(int j = 0; j < v0.connected_faces.size(); j++){
			printf("%i, ", v0.connected_faces[j]);
		}
		printf("\n	V1: %i\n", currentFace.vertex_index[1]);
		printf("		connected to: ");
		for(int j = 0; j < v1.connected_faces.size(); j++){
			printf("%i, ", v1.connected_faces[j]);
		}
		printf("\n	V1: %i\n", currentFace.vertex_index[2]);
		printf("		connected to: ");
		for(int j = 0; j < v2.connected_faces.size(); j++){
			printf("%i, ", v2.connected_faces[j]);
		}
		printf("\n");
	}
	
	printf("\nALL OF THE VERTICES OF THE MESH:------------------------------\n");
	for ( int k = 0; k < meshgroup->meshes[0].vertices.size(); k++){
		MeshVertex currentVertex = meshgroup->meshes[0].vertices[k];
		
		printf("-VERTEX ID: %i", k);
		printf("		Point: [%i, %i, %i]", currentVertex.p.z, currentVertex.p.x, currentVertex.p.y);
		printf("		connected to: ");
		for(int j = 0; j < currentVertex.connected_faces.size(); j++){
			printf("%i, ", currentVertex.connected_faces[j]);
		}
		printf("\n");
	}
	*/
	
	TransformationMatrix3D transMatrix = *new TransformationMatrix3D();
	transMatrix.matrix[0][0] = 1;
	transMatrix.matrix[1][0] = 0;
	transMatrix.matrix[2][0] = 0;
	transMatrix.matrix[3][0] = 0;
	transMatrix.matrix[0][1] = 0;
	transMatrix.matrix[1][1] = -cos(3.14159265/2);
	transMatrix.matrix[2][1] = sin(3.14159265/2);
	transMatrix.matrix[3][1] = 0;
	transMatrix.matrix[0][2] = 0;
	transMatrix.matrix[1][2] = -sin(3.14159265/2);
	transMatrix.matrix[2][2] = -cos(3.14159265/2);
	transMatrix.matrix[3][2] = 0;
	transMatrix.matrix[0][3] = 0;
	transMatrix.matrix[1][3] = 0;
	transMatrix.matrix[2][3] = 0;
	transMatrix.matrix[3][3] = 1;
	
	OBB3D obb = *new OBB3D(transMatrix);
	obb.aabb.include(*new Point3(0,0,0));
	obb.aabb.include(*new Point3(15,15,15));
	
	AABB3D aabb = *new AABB3D();
	aabb.include(*new Point3(10,10,10));
	aabb.include(*new Point3(30,30,30));
	
	if(obb.hit(aabb)){
		printf("\n\n\n ---- THEY HIT ----\n\n");
	}else{
		printf("\n\n\n ---- NO HIT ----\n\n");
	}

    //delete meshgroup;
	return;
}

// CUSTOM CODE
void decompose(int argc, char **argv){
	
	FffProcessor::getInstance()->time_keeper.restart();
	
	FMatrix3x3 transformation; // the transformation applied to a model when loaded
	
	MeshGroup* meshgroup = new MeshGroup(FffProcessor::getInstance());
	
	int extruder_train_nr = 0;
	
	SettingsBase* last_extruder_train = nullptr;
	// extruder defaults cannot be loaded yet cause no json has been parsed
	SettingsBase* last_settings_object = FffProcessor::getInstance();
	for(int argn = 2; argn < argc; argn++)
	{
		char* str = argv[argn];
		if (str[0] == '-')
		{
			if (str[1] == '-')
			{
				if (stringcasecompare(str, "--next") == 0)
				{
					try {
						//Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
						// Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
						log("Loaded from disk in %5.3fs\n", FffProcessor::getInstance()->time_keeper.restart());
						
						for (int extruder_nr = 0; extruder_nr < FffProcessor::getInstance()->getSettingAsCount("machine_extruder_count"); extruder_nr++)
						{ // initialize remaining extruder trains and load the defaults
							meshgroup->createExtruderTrain(extruder_nr); // create new extruder train objects or use already existing ones
						}
						
						meshgroup->finalize();
						
						//start slicing
						FffProcessor::getInstance()->processMeshGroup(meshgroup);
						
						// initialize loading of new meshes
						FffProcessor::getInstance()->time_keeper.restart();
						delete meshgroup;
						meshgroup = new MeshGroup(FffProcessor::getInstance());
						last_extruder_train = meshgroup->createExtruderTrain(0);
						last_settings_object = meshgroup;
						
					}catch(...){
						cura::logError("Unknown exception\n");
						exit(1);
					}
				}else{
					cura::logError("Unknown option: %s\n", str);
				}
			}else{
				for(str++; *str; str++)
				{
					switch(*str)
					{
						case 'v':
							cura::increaseVerboseLevel();
							break;
						case 'p':
							cura::enableProgressLogging();
							break;
						case 'j':
							argn++;
							if (SettingRegistry::getInstance()->loadJSONsettings(argv[argn], last_settings_object))
							{
								cura::logError("Failed to load json file: %s\n", argv[argn]);
								std::exit(1);
							}
							break;
						case 'e':
							str++;
							extruder_train_nr = int(*str - '0'); // TODO: parse int instead (now "-e10"="-e:" , "-e11"="-e;" , "-e12"="-e<" .. etc)
							last_settings_object = meshgroup->createExtruderTrain(extruder_train_nr);
							last_extruder_train = last_settings_object;
							break;
						case 'l':
							argn++;
							
							log("Loading %s from disk...\n", argv[argn]);
							
							transformation = last_settings_object->getSettingAsPointMatrix("mesh_rotation_matrix"); // the transformation applied to a model when loaded
							
							if (!last_extruder_train)
							{
								last_extruder_train = meshgroup->createExtruderTrain(0); // assume a json has already been provided on the command line
							}
							if (!loadMeshIntoMeshGroup(meshgroup, argv[argn], transformation, last_extruder_train))
							{
								logError("Failed to load model: %s\n", argv[argn]);
								std::exit(1);
							}
							else
							{
								last_settings_object = &(meshgroup->meshes.back()); // pointer is valid until a new object is added, so this is OK
							}
							break;
						case 'o':
							argn++;
							if (!FffProcessor::getInstance()->setTargetFile(argv[argn]))
							{
								cura::logError("Failed to open %s for output.\n", argv[argn]);
								exit(1);
							}
							break;
						case 'g':
							last_settings_object = meshgroup;
						case 's':
						{
							//Parse the given setting and store it.
							argn++;
							char* valuePtr = strchr(argv[argn], '=');
							if (valuePtr)
							{
								*valuePtr++ = '\0';
								
								last_settings_object->setSetting(argv[argn], valuePtr);
							}
						}
							break;
						default:
							cura::logError("Unknown option: %c\n", *str);
							print_call(argc, argv);
							print_usage();
							exit(1);
							break;
					}
				}
			}
		}
		else
		{
			
			cura::logError("Unknown option: %s\n", argv[argn]);
			print_call(argc, argv);
			print_usage();
			exit(1);
		}
	}
	
	int extruder_count = FffProcessor::getInstance()->getSettingAsCount("machine_extruder_count");
	for (extruder_train_nr = 0; extruder_train_nr < extruder_count; extruder_train_nr++)
	{ // initialize remaining extruder trains and load the defaults
		meshgroup->createExtruderTrain(extruder_train_nr); // create new extruder train objects or use already existing ones
	}
	
	
#ifndef DEBUG
	try {
#endif
		//Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
		// Only ClipperLib currently throws exceptions. And only in case that it makes an internal error.
		meshgroup->finalize();
		log("Loaded from disk in %5.3fs\n", FffProcessor::getInstance()->time_keeper.restart());
		

#ifndef DEBUG
	}catch(...){
		cura::logError("Unknown exception\n");
		exit(1);
	}
#endif
	
	VolumeDecomposer* vd = new VolumeDecomposer(meshgroup->meshes[0]);

	// vd->decompose(meshgroup->meshes[0], true);
	
	std::ofstream viewerfile;
	viewerfile.open ("output/viewer_input.txt");
	
	int fileIndex = 0;
	bool baseNode = true;
	
	printf(" \n\n\n IT HAS ::: %i faces", vd->sequenceGraph.graphNodes[0].mesh.faces.size());
	
	for(SeqNode & node : vd->sequenceGraph.graphNodes){
		FPoint3 bestVector;
		if(!baseNode){
			//find the build direction of the mesh
			Mesh temp = node.getMesh();
			BuildMap buildmap(temp);
			FPoint3 bestVector = buildmap.findBestVector();
			log("BEST VECTOR: [%f, %f, %f]\n", bestVector.x, bestVector.y, bestVector.z);
			node.theta = phiFromCartesian(bestVector); //theta
			node.phi = thetaFromCartesian(bestVector); //phi
			
			std::string filename = "output/buildmap_" + std::to_string(fileIndex)+ ".m";
			//BuildMapToMATLAB::parse(filename, buildmap, BuildMapToMATLAB::PLANE, 25);
			
			//orient the mesh based on this build direction
			node.orientMesh();
		}else{
			baseNode = false;
		}
		
		//output the final STL
		std::string filename = "output/subvolumes/output_decomp_" + std::to_string(fileIndex)+ ".STL";
		MeshToSTL::constructSTLfromMesh(node.getMesh(), filename);
		// MeshToGCode::getGCodeFromMesh(node.getMesh());
		fileIndex++;
		
		viewerfile << "../../Slicer/" << filename << "\n" << bestVector.x << " " << bestVector.y << " " << bestVector.z << "\n";
	}
	
	vd->sequenceGraph.outputGraphJSON();
	
	viewerfile.close();
}
// END CUSTOM CODE
	
}//namespace cura

using namespace cura;

int main(int argc, char **argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    //Lower the process priority on linux and mac. On windows this is done on process creation from the GUI.
    setpriority(PRIO_PROCESS, 0, 10);
#endif

#ifndef DEBUG
    //Register the exception handling for arithmic exceptions, this prevents the "something went wrong" dialog on windows to pop up on a division by zero.
    signal(SIGFPE, signal_FPE);
#endif

    Progress::init();
	
    std::cerr << std::boolalpha;
    logAlways("\n");
    logAlways("Cura_SteamEngine version %s\n", VERSION);
    logAlways("Copyright (C) 2014 David Braam\n");
    logAlways("\n");
    logAlways("This program is free software: you can redistribute it and/or modify\n");
    logAlways("it under the terms of the GNU Affero General Public License as published by\n");
    logAlways("the Free Software Foundation, either version 3 of the License, or\n");
    logAlways("(at your option) any later version.\n");
    logAlways("\n");
    logAlways("This program is distributed in the hope that it will be useful,\n");
    logAlways("but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
    logAlways("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n");
    logAlways("GNU Affero General Public License for more details.\n");
    logAlways("\n");
    logAlways("You should have received a copy of the GNU Affero General Public License\n");
    logAlways("along with this program.  If not, see <http://www.gnu.org/licenses/>.\n");


    if (argc < 2)
    {
        print_usage();
        exit(1);
    }
	
    if (stringcasecompare(argv[1], "connect") == 0)
    {
        connect(argc, argv);
    } 
    else if (stringcasecompare(argv[1], "slice") == 0)
    {
        slice(argc, argv);
	}else if (stringcasecompare(argv[1], "decompose") == 0)
	{
		decompose(argc, argv);
	}
    else if (stringcasecompare(argv[1], "help") == 0)
    {
        print_usage();
        exit(0);
    }
    else if (stringcasecompare(argv[1], "analyse") == 0)
    { // CuraEngine analyse [json] [output.gv] [engine_settings] -[p|i|e|w]
        // p = show parent-child relations
        // i = show inheritance function
        // e = show error functions
        // w = show warning functions
        // dot refl_ff.gv -Tpng > rafl_ff_dotted.png
        // see meta/HOWTO.txt
        
        bool parent_child_viz = false;
        bool inherit_viz = false;
        bool warning_viz = false;
        bool error_viz = false;
        bool global_only_viz = false;
        if (argc >= 6)
        {
            char* str = argv[5];
            if (str[0] == '-')
            {
                for(str++; *str; str++)
                {
                    switch(*str)
                    {
                    case 'p':
                        parent_child_viz = true;
                        break;
                    case 'i':
                        inherit_viz = true;
                        break;
                    case 'e':
                        error_viz = true;
                        break;
                    case 'w':
                        warning_viz = true;
                        break;
                    case 'g':
                        global_only_viz = true;
                        break;
                    default:
                        cura::logError("Unknown option: %c\n", *str);
                        print_call(argc, argv);
                        print_usage();
                        break;
                    }
                }
            }
        }
        else
        {
            cura::log("\n");
            cura::log("usage:\n");
            cura::log("CuraEngine analyse <fdmPrinter.def.json> <output.gv> <engine_settings_list> -[p|i|e|w]\n");
            cura::log("\tGenerate a grpah to visualize the setting inheritance structure.\n");
            cura::log("\t<fdmPrinter.def.json>\n\tThe base seting definitions file.\n");
            cura::log("\t<output.gv>\n\tThe output file.\n");
            cura::log("\t<engine_settings_list>\n\tA text file with all setting keys used in the engine, separated by newlines.\n");
            cura::log("\t-[p|i|e|w]\n\tOptions for what to include in the visualization\n");
            cura::log("\t\tp\tVisualize the parent-child relationship.\n");
            cura::log("\t\ti\tVisualize inheritance function relationships.\n");
            cura::log("\t\te\tVisualize (max/min) error function relationships.\n");
            cura::log("\t\tw\tVisualize (max/min) warning function relationships.\n");
            cura::log("\n");
    
        }
        
        SettingsToGv gv_out(argv[3], argv[4], parent_child_viz, inherit_viz, error_viz, warning_viz, global_only_viz);
        if (gv_out.generate(std::string(argv[2])))
        {
            cura::logError("Failed to analyse json file: %s\n", argv[2]);
        }
        exit(0);
    }
    // CUSTOM CODE
    else if (stringcasecompare(argv[1], "velocityprofile") == 0) {
        char* gcodeFile = argv[2];

        PathSmoother ps = PathSmoother(gcodeFile);
    }
    // END CUSTOM CODE
    else
    {
        cura::logError("Unknown command: %s\n", argv[1]);
        print_call(argc, argv);
        print_usage();
        exit(1);
    }
	 /*
	
	AABB3D box;
	box.include(Point3(0,0,0));
	box.include(Point3(5,5,5));
	
	std::vector<Point3> face;
	face.push_back(Point3(-5,0,-1));
	face.push_back(Point3(1, 1,6));
	face.push_back(Point3(1, 0, 0));
	
	//CollisionDetection::faceBoxOverlap(box, face);
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
	
	
	printf("\n New rotation matrix: \n| %i  %i  %i   %i |\n| %i  %i  %i   %i |\n| %i  %i  %i   %i |\n| %i  %i  %i   %i |\n", rotationMatrix[0][0], rotationMatrix[0][1], rotationMatrix[0][2], rotationMatrix[0][3], rotationMatrix[1][0], rotationMatrix[1][1], rotationMatrix[1][2], rotationMatrix[1][3], rotationMatrix[2][0], rotationMatrix[2][1], rotationMatrix[2][2], rotationMatrix[2][3], rotationMatrix[3][0], rotationMatrix[3][1], rotationMatrix[3][2], rotationMatrix[3][3]);

	*/
    return 0;
}
