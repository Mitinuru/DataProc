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
#include <iostream>
//using namespace rw::models;
//#include <WorkCellLoader.hpp>

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

 /*
    //==========================================================================================
    // MANUAL
    //==========================================================================================


//Printing out all devices in the WorkCell
void printDeviceNames(WorkCell::Ptr workcell)
{
  std::cout << "Workcell " << workcell->getName() << " contains devices:\n";
  //BOOST_FOREACH(Device* device, workcell.getDevices()) {
  BOOST_FOREACH(rw::common::Ptr<Device> device, workcell->getDevices()) {
    std::cout << "- " << device->getName() << "\n";
  }
}

//Printing out the kinematic tree
void printKinematicTree( Frame* frame, const State& state, const Transform3D<>& parentTransform, int level)
{
  const Transform3D<> transform = parentTransform * frame->getTransform(state);
  std::cout
    << std::string(level, ' ')
    << frame->getName()
    << " at "
    << transform.P()
    << "\n";
  BOOST_FOREACH(Frame& child, frame->getChildren(state)) {
    printKinematicTree(&child, state, transform, level + 1);
  }
}

//Printing out Default workcell structure
void printDefaultWorkCellStructure(WorkCell::Ptr workcell)
{
  std::cout << workcell->getName() << "\n";
  printKinematicTree(
  workcell->getWorldFrame(),
  workcell->getDefaultState(),
  Transform3D<>::identity(),
  0);
}

//Effective computing of forward kinematics for a number of frames for a common state. 
//The results of the forward kinematics are stored in the FKTable object so that the transform for a frame is not computed over and over again.
std::vector<Transform3D<> > worldTransforms( const std::vector<Frame*>& frames, const State& state)
{
  FKTable fk(state);
  std::vector<Transform3D<> > result;
  BOOST_FOREACH(Frame* f, frames) {
    result.push_back(fk.get(*f)); // compute the transform with respect to world frame
  }
  return result;
}

//efficiently compute the relative transform for a pair of frames the path in the kinematic tree that connects the frames must be computed by FKRange
Transform3D<> frameToFrameTransform( const Frame& a, const Frame& b, const State& state)
{
  FKRange fk(&a, &b, state);
  return fk.get(state);
}

//repeatedly compute the forward kinematics for the same pair of frames and the same parent-child structure of the tree for a set of states
std::vector<Transform3D<> > frameToFrameTransforms( const Frame& a, const Frame& b, const State& tree_structure, const std::vector<State>& states)
{
  FKRange fk(&a, &b, tree_structure);
  std::vector<Transform3D<> > result;
  BOOST_FOREACH(const State& state, states) {
    result.push_back(fk.get(state));
  }
  return result;
}

//check if frame is dynamically attached frame (DAF)
bool isDaf(const Frame& frame)
{
  return frame.getParent() == NULL;
}

//Attaching a DAF frame to Gripper 
void gripMovableFrame(MovableFrame& item, Frame& gripper, State& state)
{
  FKRange fk(&gripper, &item, state);
  const Transform3D<> transform = fk.get(state);
  item.setTransform(transform, state);
  item.attachTo(&gripper, state);
}

// convert a sequence of configurations for a common state into a sequence of states
std::vector<State> getStatePath(const Device& device, const std::vector<Q>& path, const State& common_state)
{
  State state = common_state;
  std::vector<State> result;
  BOOST_FOREACH(const Q& q, path) {
    device.setQ(q, state);
    result.push_back(state);
  }
  return result;
}

// instantiation and expected output for 3 different metrics
void metricExample()
{
  typedef Vector3D<> V;
  typedef Ptr<Metric<V> > VMetricPtr;

  VMetricPtr m1 = MetricFactory::makeManhattan<V>();
  VMetricPtr m2 = MetricFactory::makeEuclidean<V>();
  VMetricPtr mInf = MetricFactory::makeInfinity<V>();

  const V a(0, 0, 0);
  const V b(1, 1, 1);

  std::cout
    << m1->distance(a, b) << " is " << 3.0 << "\n"
    << m2->distance(a, b) << " is " << sqrt(3.0) << "\n"
    << mInf->distance(a, b) << " is " << 1 << "\n";
}

//Collision detector for the default collision setup of a workcell. Program then calls the collision detector to see if the workcell is in collision in its initial state
void collisionExample(rw::models::WorkCell& workcell)
{
  CollisionDetector detector(&workcell, ProximityStrategyYaobi::make()); //collision detector call for default setup with ProximityStrategyYaobi::make() strategy
    
  const bool collision =  detector.inCollision(workcell.getDefaultState()); //check if in collision
  
  std::cout
    << "Workcell "
    << workcell.getName()
    << " is in collision in its initial state: "
    << collision
    << "\n";
}

//Adding and removing geometries from collision detector
void addGeometryToDetector(CollisionDetector::Ptr cd, Frame* myframe, Geometry::Ptr myGeometry)
{
  //cd->addModel(myframe, mygeometry);
  cd->addGeometry(myframe, myGeometry);
}
void removeGeometryFromDetector(CollisionDetector::Ptr cd, Frame* myframe, Geometry::Ptr myGeometry)
{
  //cd->removeModel(myframe, mygeometry->getId());
  cd->removeGeometry(myframe, myGeometry->getId());
}

//Collision detector and corresponding default planner constraint for the first device of the workcell. 
//The program calls the planner constraint to check if the edge from the lower to upper corner of the configuration space can be traversed:
void constraintExample(WorkCell& workcell)
{
  Device::Ptr device = workcell.getDevices().front();
  
  const PlannerConstraint constraint = PlannerConstraint::make(
    ownedPtr( new CollisionDetector(
      &workcell, ProximityStrategyYaobi::make() ) ),
    device, 
    workcell.getDefaultState());
  
  const Q start = device->getBounds().first; // upper bound of joint space
  const Q end = device->getBounds().second; // lower bound of joint space
  
  std::cout
    << "Start configuration is in collision: "
    << constraint.getQConstraint().inCollision(start)
    << "\n"
    << "End configuration is in collision: "
    << constraint.getQConstraint().inCollision(end)
    << "\n"
    << "Edge from start to end is in collision: "
    << constraint.getQEdgeConstraint().inCollision(start, end)
    << "\n";
}

//construction of a sampler of collision free configurations. 
//The sampler calls a randomized sampler of the configuration space of the device, and filters these configurations by the constraint that the configurations should be collision free.
void samplerExample(WorkCell& workcell)
{
  Device::Ptr device = workcell.getDevices().front(); //first device in the workcell which is probably robot
  
  QConstraint::Ptr constraint = QConstraint::make( //creating constrain in configuration space
    ownedPtr( new CollisionDetector(
      &workcell, ProximityStrategyYaobi::make() ) ),
    //CollisionDetector::make( //creating collision detector
      //&workcell, ProximityStrategyYaobi::make()),
    device,
    workcell.getDefaultState()); //in default state

  QSampler::Ptr anyQ = QSampler::makeUniform(device); //creatte uniform sampling in configuration space; Uniform random sampling for a device. 
  QSampler::Ptr cfreeQ = QSampler::makeConstrained(anyQ, constraint); //	A sampler filtered by a constraint. 

  for (int i = 0; i < 4; i++) {
    const Q q = cfreeQ->sample();
    std::cout
      << "Q(" << i << ") is in collision: "
      << constraint->inCollision(q) << "\n";
  }
}

// path planner for the first device of the workcell, it plans a number of paths to random collision free configurations of the workcell. 
//The full configuration space path mapped to the corresponding sequence of states (rw::kinematics::State) and written to a file that can 
//be loaded into RobWorkStudio using the PlayBack plugin
void plannerExample(WorkCell& workcell)
{
  // The common state for which to plan the paths.
  const State state = workcell.getDefaultState();

  // The first device of the workcell.
  Device::Ptr device = workcell.getDevices().front();

  // The path planning constraint is to avoid collisions.
  const PlannerConstraint constraint = PlannerConstraint::make(
    ProximityStrategyYaobi::make(), &workcell, device, state);

  QConstraint::Ptr constraint_Q =constraint.getQConstraintPtr();
  rw::math::QMetric::Ptr metric = MetricFactory::makeManhattan<Q>();;
  QEdgeConstraintIncremental::Ptr constraint_edge_Q =  QEdgeConstraintIncremental::make(constraint_Q, metric);
  //QEdgeConstraint::Ptr constraint_edge_Q =constraint.getQEdgeConstraintPtr();
    
  // An SBL based point-to-point path planner.
  //QToQPlanner::Ptr planner = SBLPlanner::makeQToQPlanner(
    //SBLSetup::make(constraint, device));
  SBLSetup setup = SBLSetup::make(constraint_Q, constraint_edge_Q, device, -1.0, -1.0);
  QToQPlanner::Ptr planner = SBLPlanner::makeQToQPlanner(setup);

  // A sampler of collision free configurations for the device.
  QSampler::Ptr cfreeQ = QSampler::makeConstrained(
    QSampler::makeUniform(device),
    constraint.getQConstraintPtr());

  // The start configuration for the path.
  Q pos = device->getQ(state);

  // Plan 10 paths to sampled collision free configurations.
  //std::vector<Q> path;
  rw::trajectory::Path< rw::math::Q > path;
  for (int cnt = 0; cnt < 10; cnt++) {
    const Q next = cfreeQ->sample();
    const bool ok = planner->query(pos, next, path);
    if (!ok) {
      std::cout << "Path " << cnt << " not found.\n";
      return;
  } else {
      pos = next;
    }
  }

  // Map the configurations to a sequence of states.
  const std::vector<State> states = Models::getStatePath(*device, path, state);

  // Write the sequence of states to a file.
  PathLoader::storeVelocityTimedStatePath(
  workcell, states, "ex-path-planning.rwplay");
}


// instantiation of some more of the available path planners
QToQPlanner::Ptr getQToQPlanner(Device::Ptr device, const PlannerConstraint constraint, const std::string& type)
{
  if (type == "SBL") {
    QConstraint::Ptr constraint_Q =constraint.getQConstraintPtr();
    rw::math::QMetric::Ptr metric = MetricFactory::makeManhattan<Q>();;
    QEdgeConstraintIncremental::Ptr constraint_edge_Q =  QEdgeConstraintIncremental::make(constraint_Q, metric);
  
    SBLSetup setup = SBLSetup::make(constraint_Q, constraint_edge_Q, device, -1.0, -1.0);
    QToQPlanner::Ptr planner = SBLPlanner::makeQToQPlanner(setup);
    Log::infoLog() << "Name of constrain planner is SBL "  << std::endl;
    return planner;
  } else if (type == "RRT") {
      Log::infoLog() << "Name of constrain planner is RRT "  << std::endl;
      return RRTPlanner::makeQToQPlanner(constraint, device);
      
  } else if (type == "ARW") {
    Log::infoLog() << "Name of constrain planner is ARW "  << std::endl;  
    return ARWPlanner::makeQToQPlanner(constraint, device);

  } else {
    Log::infoLog() << "Not found "  << std::endl;  
    return NULL;
    
  }
}

*/


//inverse kinematics (IK) solver
void inverseKinematics(Device::Ptr device, const State& state, const Transform3D<>& target)
{
  JacobianIKSolver solver(device, state);
  std::vector<Q> solutions = solver.solve(target, state);
  std::cout<<"Looking for solution ..." <<std::endl;
  //std::cout<<"Amount of solutions = " << solutions.size <<std::endl;
  int i = 0;
  BOOST_FOREACH(Q q, solutions) {
    std::cout<<"Solution = "<<q<<std::endl;
    i++;
  }
  std::cout<<"Amount of solutions = " << i <<std::endl;
}

//program below tests the default iterative IK solver for a device. The program selects 10 random base to end transforms for a device using the forward kinematics for the device. 
//Using the default IK sampler, the program then checks that an IK solution is found for all transforms. 
//Only a small number of start configurations are used for each target transform, and therefore the IK sampler might not always find an IK solution. 
//If the IK sampler is constrained by the requirement that the IK solutions must be collision free, then solutions for only a subset of the target transforms are found.
TransformPath getRandomTargets(const Device& device, State state, int targetCnt)
{
  TransformPath result;
  QSampler::Ptr sampler = QSampler::makeUniform(device);
  for (int cnt = 0; cnt < targetCnt; cnt++) {
    device.setQ(sampler->sample(), state);
    result.push_back(device.baseTend(state));
  }
  return result;
}

void printReachableTargets(
  Device& device,
  const State& state,
  const TransformPath& targets,
  QIKSampler& ik)
{
  int i = 0;
  BOOST_FOREACH(const Transform3D<>& target, targets) {
    const Q q = ik.sample(target);
    std::cout << i << " " << (q.empty() ? "False" : "True") << "\n";
    ++i;
  }
}

void invkinExample(Device& device, const State& state, QConstraint& constraint)
{
  QIKSampler::Ptr ik_any = QIKSampler::make(&device, state, NULL, NULL, 25);
  QIKSampler::Ptr ik_cfree = QIKSampler::makeConstrained(ik_any, &constraint, 25);
  
  const TransformPath targets = getRandomTargets(device, state, 10);
  
  std::cout << "IK solutions found for targets:\n";
  printReachableTargets(device, state, targets, *ik_any);
  
  std::cout << "Collision free IK solutions found for targets:\n";
  printReachableTargets(device, state, targets, *ik_cfree);
}


//construct a small task, prints out the task, saves it to file, reloads it and prints it once again.
void printMotion(QMotion::Ptr motion) {
  switch (motion->motionType()) {
  case MotionType::P2P: {
    QP2PMotion::Ptr p2p = motion.cast<QP2PMotion>();
    std::cout<<"Got P2P Motion from "<<p2p->start()<<" to "<<p2p->end()<<std::endl;
    break; }
  case MotionType::Linear: {
    QLinearMotion::Ptr lin = motion.cast<QLinearMotion>();
    std::cout<<"Got Linear Motion from "<<lin->start()<<" to "<<lin->end()<<std::endl;
    break; }
  }
}

void printAction(Action::Ptr action) {
  std::cout<<"Got Action of type = "<<action->getId()<<std::endl;
}

void printTask(QTask::Ptr task) {
  std::vector<Entity::Ptr> entities = task->getEntities();
  for (std::vector<Entity::Ptr>::iterator it = entities.begin(); it != entities.end(); ++it) {
    Entity::Ptr entity = *it;
    switch (entity->entityType()) {
    case EntityType::Motion: {
      QMotion::Ptr motion = entity.cast<QMotion>();
      printMotion(motion);
      break; }
    case EntityType::Action: {
      Action::Ptr action = entity.cast<Action>();
      printAction(action);
      break; }
    case EntityType::Task: {
      QTask::Ptr task = entity.cast<QTask>();
      printTask(task);
      break; }
    }
  }
}


int main(int argc, char** argv) {

  
     
    WorkCell::Ptr wc = WorkCellFactory::load("/home/nadezda/workspace/nadia_rw_project/SimpleWorkcell.xml");
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
    

    /*
    //==========================================================================================
    // MANUAL
    //==========================================================================================
    std::cout << "Printing in function \n" << std::endl;
    printDeviceNames(wc);
    
    std::cout << "Printing out Default workcell structure \n" << std::endl;
    printDefaultWorkCellStructure(wc);
    
    std::cout << "Effective computing forward kinematics \n" << std::endl;
    std::vector<Frame*> frames = wc->getFrames (); 
    std::vector<Transform3D<> > vector_transforms = worldTransforms(frames, state);
    for(int i = 0; i< vector_transforms.size(); i++) {
       Log::infoLog() << frames[i]->getName() << " : " <<vector_transforms[i] << std::endl;
    }

    std::cout << "Effective computing relative transform for a pair of frames \n" << std::endl;
    Frame* a = wc->findFrame("SchunkHand.f1.Joint2");
    Frame* b = wc->findFrame("Scanner25D");
    Transform3D<> aTb = frameToFrameTransform(*a, *b,state);
    Log::infoLog() << "relative transform for a pair of frames  aTb : " <<  aTb << std::endl;
    
    
    std::cout << "Effective computing relative transform for a pair of frames in new configuration \n" << std::endl;
    //Default configuration
    math::Q q_0 = device1->getQ(state);
    //New configuration
    double  val [6] = {0.451, 1.4, 0.976, 0.0, 0.76, 0.0};
    math::Q q_new(6, val);
    device1->setQ(q_new, state);
    Transform3D<> aTb_new = frameToFrameTransform(*a, *b,state);
    Log::infoLog() << "relative transform for a pair of frames  aTb : " <<  aTb_new << std::endl; 

    
    
    std::cout << "Attaching DAF frame of cup to gripper \n" << std::endl;
    MovableFrame* cup = wc->findFrame<MovableFrame>("Cup_item");
    //std::cout<<cup<<std::endl; // check if frame was found
    Frame* gripp = wc->findFrame("handBase");    
    //std::cout<<gripp<<std::endl; // check if frame was found
    Log::infoLog() << "Frame cup_item is attached to : " <<  cup->getParent(state)->getName() << std::endl;     
    gripMovableFrame(*cup, *gripp, state);
    Log::infoLog() << "Now frame cup_item is attached to : " <<  cup->getParent(state)->getName() << std::endl;       
    
    std::cout << "Converting a sequence of configurations for a common state into a sequence of states \n" << std::endl;    
    std::vector<Q> path(5, q_0);
    //Printing out default and new configurations
    Log::infoLog() << "Default configuration: " << q_0 << std::endl;
    Log::infoLog() << "New configuration: " << q_new << std::endl;
    path[0]= q_0;
    Log::infoLog() << "New configuration q0: " << path[0] << std::endl;
    double  val1 [6] = {0.451,  0.0,  0.0, 0.0,  0.0, 0.0};
    math::Q q_1(6, val1);  
    path[1]= q_1;
    Log::infoLog() << "New configuration q1: " << path[1] << std::endl;
    double  val2 [6] = {0.451,  1.4,  0.0, 0.0,  0.0, 0.0};
    math::Q q_2(6, val2); 
    path[2]= q_2;
    Log::infoLog() << "New configuration q2: " << path[2] << std::endl;
    double  val3 [6] = {0.451,  1.4,   0.976, 0.0,  0.0, 0.0};
    math::Q q_3(6, val3);
    //path[1]= q_1;
    //path[2]= q_2;
    path[3]= q_3;
    Log::infoLog() << "New configuration q3: " << path[3] << std::endl;
    path[4]= q_new;
    Log::infoLog() << "New configuration q4: " << path[4] << std::endl;
    State state0 = state;
    std::vector<State> states_path = getStatePath(*device1, path, state);
   
    metricExample();
    
    collisionExample(*wc);
    
    constraintExample(*wc);
   
    samplerExample(*wc);
    
    plannerExample(*wc);
    
    // The path planning constraint is to avoid collisions.
    const PlannerConstraint constraint = PlannerConstraint::make( ProximityStrategyYaobi::make(), wc, device1, state);
    QToQPlanner::Ptr QtoQplanner_mine = getQToQPlanner(device1, constraint, "ARW");
    */
    
    TransformPath targetes1 = getRandomTargets(*device1, state, 1);
    Transform3D<> target = targetes1[0];
    inverseKinematics(device1, state, target);
    
    CollisionDetector::Ptr detector = ownedPtr( new CollisionDetector(
      wc, ProximityStrategyYaobi::make()) );
    QConstraint::Ptr constraint = QConstraint::make(detector, device1, state);
    invkinExample(*device1, state, *constraint);
   
    
     //Construct a Task
    QTask::Ptr task = ownedPtr(new QTask());
    Q q1(1); q1(0) = 1;
    Q q2(1); q2(0) = 2;
    Q q3(1); q3(0) = 3;
    task->addTargetByValue(q1);
    task->addTargetByValue(q2);
    task->addTargetByValue(q3);

    std::vector<QTarget::Ptr>& targets = task->getTargets();
    task->addMotion(ownedPtr(new QP2PMotion(targets[0], targets[1])));
    task->addMotion(ownedPtr(new QLinearMotion(targets[1], targets[2])));
    task->addAction(ownedPtr(new Action(ActionType::On)));

    printTask(task);

    XMLTaskSaver::save(task, "MyTask.xml");

    XMLTaskLoader loader;
    loader.load("MyTask.xml");
    QTask::Ptr task2 = loader.getQTask();

    printTask(task2);
     
    /*
    //====================================================================================
    // EXERSIZE 1 - 3 FROM PROGRAMMING PRIMER
    //====================================================================================
    // get the default state
    State state = wc->getDefaultState();
    // Extract the first device of the workcell
    Device::Ptr device1 = frames[0];
    //Default configuration
    math::Q q_0 = device1->getQ(state);
    //New configuration
    double  val [6] = {0.451, 1.4, 0.976, 0.0, 0.76, 0.0};
    math::Q q_new(6, val);
    //Set device to new configuration
    device1->setQ(q_new, state);
    //Printing out default and new configurations
    Log::infoLog() << "Default configuration: " << q_0 << std::endl;
    Log::infoLog() << "New configuration: " << q_new << std::endl;
    
    // get device base frame
    Frame *base = device1->getBase();
    Log::infoLog() << base->getTransform(state) << std::endl;
    // get device end effector
    Frame *end = device1->getEnd();
    Log::infoLog() << end->getTransform(state) << std::endl;
    // calculate base to end transform
    Transform3D<> bTe = device1->baseTend(state);
    Log::infoLog() << "Base to end effector transform: " << bTe << std::endl;
    
    Transform3D<> wTb = device1->worldTbase(state); 
    Transform3D<> wTe = wTb*bTe;
    Log::infoLog() << "World toend effector transform: " << wTe << std::endl;
    //Log::infoLog() << "Default state of workcell: " << state << std::endl;
    
    /*
    
    /*  
    //==================================================================================== 
    // THE TUTORIAL - PROGRAMING PRIMER EXAMPLE
    //====================================================================================
    Frame* worldFrame = wc->getWorldFrame();
    // find a frame by name, remember NULL is a valid return
    Frame* frame = wc->findFrame("FixedFrameName");
    // find a frame by name, but with a specific frame type
    FixedFrame* fframe = wc->findFrame<FixedFrame>("FixedFrameName");
    MovableFrame* mframe = wc->findFrame<MovableFrame>("MovableFrameName");
    // find a device by name
    Device::Ptr device = wc->findDevice("SerialDeviceName");
    SerialDevice::Ptr sdevice = wc->findDevice<SerialDevice>("SerialDeviceName");



    // calculate the transform from one frame to another
    Transform3D<> fTmf = Kinematics::frameTframe(frame, mframe, state);
    // calculate the transform from world to frame
    Transform3D<> wTmf = Kinematics::worldTframe( mframe, state );
    // we can find the world to frame transform by a little jogling
    Transform3D<> wTf = wTmf * inverse(fTmf);
    // test if frame is a dynamic attachable frame
    if( Kinematics::isDAF( mframe ) ){
        // attach mframe to end of serial device
        Kinematics::gripFrame(mframe, sdevice->getEnd(), state);
    }


    // get device base frame
    Frame *base = sdevice->getBase();
    // get device end effector
    Frame *end = sdevice->getEnd();
    // calculate base to end transform
    Transform3D<> bTe = sdevice->baseTend(state);
    // or just base to any frame
    Transform3D<> bTmf = sdevice->baseTframe(mframe, state);
    // get device name
    std::string sdevicename = sdevice->getName();
    // the degrees of freedom of this device
    int dof = sdevice->getDOF();
    // set the configuration of the device to zero
    sdevice->setQ( Q::zero(dof) , state );

*/
    return 0;
}