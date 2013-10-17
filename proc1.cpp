/* This file is reading paces saved from teleoperation, processes them and resaves into multiple files:
 * 1. file with time and global coordinates
 * 2. for each object it produces one additional file with time, local coordinates and z orientation (2 objects -> 2 files)
 * 3. file with time and contact points
 * 
 */


#include <rw/rw.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <boost/foreach.hpp>
#include <rw/kinematics/Frame.hpp>
#include <string>
#include <rw/kinematics/FKTable.hpp>
#include <vector>
#include <rw/kinematics/FKRange.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>

#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/Motion.hpp>
#include <rwlibs/task/Action.hpp>
#include <rwlibs/task/Target.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>
#include <rwlibs/task/loader/XMLTaskSaver.hpp>
//#include <DynamicWorkCell.hpp>
#include <iostream>
#include "InOut.hpp" 

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rwlibs::proximitystrategies;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::loaders;
using namespace rwlibs::pathplanners;
using namespace rw::common;
using namespace rwlibs::task;

typedef std::vector<Transform3D<> > TransformPath;

  int main(int argc, char** argv) {

    Log::infoLog() << "The using namespace enables us to call Log directly!\n";
    
    // ammount of manipulated objects
    int N_o = 2;
    
    //file definitions
    string file_name, file_glob, file_loc1, file_loc2, file_cnct;
    file_name = "../../path_0";
    file_glob = "../../glob";
    file_loc1 = "../../loc1";
    file_loc2 = "../../loc2";
    file_cnct = "../../cnct"; //connection points
    
    //Reader, Recorder, paces definitions
    Reader Reader1(file_name);
    Recorder_coord Recorder_glob; //Recorder_loc1, Recorder_loc2;
    Recorder_z Recorder_loc1, Recorder_loc2;
    Recorder_cnct Recorder_cnct;
    
    Recorder_glob.save_coord(file_glob);
    //Recorder_loc1.save_coord(file_loc1);
    //Recorder_loc2.save_coord(file_loc2);
    Recorder_loc1.save_z(file_loc1);
    Recorder_loc2.save_z(file_loc2);
    Recorder_cnct.save_cnct(file_cnct);
    
    Pace * Pace; //pace read from the file 
    
    // Read, process, write loop
    int i =0;
    
    while ( (Pace = Reader1.readNext()) != NULL ) {
	//RW_WARN("");
	//Pace_coord::Ptr  Pace_glob;
	Pace_coord::Ptr  Pace_glob = Pace->Proc_glob(); // getting only global coord from original pace + coord of end-effector
	Pace_cnct::Ptr  Pace_cnct = Pace->Proc_cnct(); // getting only connection points
	Pace_z::Ptr  Pace_loc1 = Pace->Proc_loc_z(0); // getting only local coord for object1
	Pace_z::Ptr  Pace_loc2 = Pace->Proc_loc_z(1); // getting only local coord for object2
	//Pace_coord::Ptr  Pace_loc1 = Pace->Proc_loc(0); // getting only local coord for object1
	//Pace_coord::Ptr  Pace_loc2 = Pace->Proc_loc(1); // getting only local coord for object2
	
	Recorder_glob.addPace_coord(Pace_glob);
	Recorder_loc1.addPace_z(Pace_loc1);
	Recorder_loc2.addPace_z(Pace_loc2);
	//Recorder_loc1.addPace_coord(Pace_loc1);
	//Recorder_loc2.addPace_coord(Pace_loc2);
	Recorder_cnct.addPace_cnct(Pace_cnct);
	
	i = i+1;
    }
    
    RW_WARN("");
    Recorder_glob.save_coord(file_glob);
    Recorder_loc1.save_z(file_loc1);
    Recorder_loc2.save_z(file_loc2);
    //Recorder_loc1.save_coord(file_loc1);
    //Recorder_loc2.save_coord(file_loc2);
    Recorder_cnct.save_cnct(file_cnct);
    
    Log::infoLog() << "Ammount of iterations\n" << i;
    
    Reader1.closeFile();
    RW_WARN("");
    
}