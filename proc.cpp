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
    
    // Playing with reader class
    
    //Reader::Reader(path_0);
    string file_name, file_name1, file_name2, file_name3;
    vector<Pace *> fail1;
    //RW_WARN("");
    file_name = "../../path_0";
    file_name1 = "../../path_1";
    //RW_WARN("");
    //Reader Reader1(file_name);
    Reader Reader1(file_name);
    //RW_WARN("");
    Pace * Pace1 = Reader1.readNext();
    //RW_WARN("");
    std::cout << Pace1 << std::endl;
   // RW_WARN("");
    double time1 = Pace1->Time();
    //RW_WARN("");
    //std::cout << time1 << std::endl;
    //RW_WARN("");
    Pace1->printPace();
    RW_WARN("");
    Pace_new::Ptr  Pace2 = Pace1->Proc();
    RW_WARN("");
    //Pace2->printPace_new();
    RW_WARN("");
    
    // Trying to process the data from pace to the format i want
    
    file_name2 = "../../path_2";
    Recorder_new Recorder1;
    //RW_WARN("");
    Recorder1.save_new(file_name2);
    //RW_WARN("");
    Recorder1.addPace_new(Pace2);
    //RW_WARN("");
    Recorder1.save_new(file_name2);
    //RW_WARN("");
    
    // PUT THE WHOLE READ_PROCESS_WRITE INTO ONE CYCLE
    vector<Pace *> paces;
    Pace * Pace3;
    int i =0;
	//RW_WARN("");
    //openFile(filename);
	
    while ( (Pace3 = Reader1.readNext()) != NULL ) {
	//paces.push_back(pace);
	//RW_WARN("");
	 Pace_new::Ptr  Pace4;
	if (i < 100) {
	  Pace4 = Pace3->Proc_1();
	}
	if (i >= 100) {
	   Pace4 = Pace3->Proc_0();
	}  
	
	// Pace_new::Ptr  Pace4 = Pace3->Proc_0(); //Pace3->Proc();
	 //Pace4->printPace_new();
	// RW_WARN("");
	 Recorder1.addPace_new(Pace4);
	 //RW_WARN("");
	 //Recorder1.save_new(file_name2);
	 //RW_WARN("");
	 i = i+1;
	 //Log::infoLog() << "Ammount of iterations\n" << i;
    }
	RW_WARN("");
	Recorder1.save_new(file_name2);
	Log::infoLog() << "Ammount of iterations\n" << i;
    Reader1.closeFile();
    RW_WARN("");
    /*
    // Loading dynamic workcell
    WorkCell::Ptr wc = WorkCellFactory::load("/home/nadezda/workspace/DataProc/cup_pg70_table.dwc.xml");
    //Printing out the name
    Log::infoLog() << "Name of workcell: " << wc->getName() << std::endl;
    //getDevices
    std::vector <rw::common::Ptr<Device> >  devices_frames = wc->getDevices();
    //Add for loop with printing inside alike wc print
    for(int i = 0; i< devices_frames.size(); i++) {
       Log::infoLog() << "Device name: " << devices_frames[i]->getName() << std::endl;
    }
    // get the default state
    State state = wc->getDefaultState();
    // Extract the first device of the workcell
    Device::Ptr device1 = devices_frames[0];
    
 */
}