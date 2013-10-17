#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

int main(int argc, char** argv) {
    //Transform3D<> trans;
    //trans.P();
  
    RPY<> rpy(0, 0, 90*Deg2Rad); //Constructor call; 90 degree rotation around x-axis
    Rotation3D<> rot = rpy.toRotation3D(); // create Rotation3D matrix 3x3
    EAA<> eaa( rot ); // construct Equivalent Angle-axis representation eaa form rotation3d
    Quaternion<> quat( rot ); // construct quaternion from rotation3d; Extracts a Quaternion from Rotation matrix using setRotation(const Rotation3D<R>& rot) 

    // there are streaming operators for all math types
    Log::infoLog() << rpy << std::endl;
    Log::infoLog() << rot << std::endl;
    Log::infoLog() << eaa << std::endl;
    Log::infoLog() << quat << std::endl;


    // rotate a vector (0,1,0) 90 degrees around x-axis
    Log::infoLog() << rot*Vector3D<>(0,1,0) << std::endl;
    // transform a vector
    Transform3D<> t1( Vector3D<>(0,0,1), rot);
    std::cout << "Transform a vector" << std::endl;
    Log::infoLog() << t1*Vector3D<>(0,1,0) << std::endl;
    // calcualte the inverse rotation
    Log::infoLog() << inverse( rot ) << std::endl;
    // calculate the inverse transform
    Log::infoLog() << inverse( t1 ) << std::endl;
    // get the rotation and translation part of a transform
    Log::infoLog() << t1.R() << t1.P() << std::endl;
    
    //Exercise 1
    
    RPY<> rpy1(90*Deg2Rad, 0, 0); //Constructor call; 90 degree rotation around z-axis
    Rotation3D<> rot1 = rpy1.toRotation3D(); // create Rotation3D matrix 3x3
    Transform3D<> t11( Vector3D<>(1,1,1), rot1);

    RPY<> rpy2(0, 0, 0); //Constructor call; 90 degree rotation around z-axis
    Rotation3D<> rot2 = rpy2.toRotation3D(); // create Rotation3D matrix 3x3
    Transform3D<> t12( Vector3D<>(0,0,1), rot2);

    // there are streaming operators for all math types
    Log::infoLog() << t11 << std::endl;
    Log::infoLog() << t12 << std::endl;
    Log::infoLog() << inverse(t11) << std::endl;
    Log::infoLog() << inverse(t12) << std::endl;

    //Multiplication
    Transform3D<> t2;
    Transform3D<> t3;
    t3.multiply(t11, t12, t2);
    
    // there are streaming operators for all math types
    Log::infoLog() << t2 << std::endl;
    Log::infoLog() << t3 << std::endl;
    
    //Exercise 2
    Vector3D<> v0;
    v0 = inverse( t11 )*inverse( t12 )*Vector3D<>(0.5,1,0);
    std::cout << "Vector (0.5,1,0) in new coordinates" << std::endl;
    Log::infoLog() << v0 << std::endl;
    /*
    RPY<> rpy1(90*Deg2Rad, 0, 0); //Constructor call; 90 degree rotation around z-axis
    Rotation3D<> rot1 = rpy1.toRotation3D(); // create Rotation3D matrix 3x3
    Transform3D<> t11( Vector3D<>(1,1,1), rot1);

    RPY<> rpy2(0, 0, 0); //Constructor call; 90 degree rotation around z-axis
    Rotation3D<> rot2 = rpy2.toRotation3D(); // create Rotation3D matrix 3x3
    Transform3D<> t12( Vector3D<>(0,0,1), rot2);

    // there are streaming operators for all math types
    Log::infoLog() << t11 << std::endl;
    Log::infoLog() << t12 << std::endl;

    //Multiplication
    Transform3D<> t2;
    Transform3D<> t3;
    t3.multiply(t11, t12, t2);
    
    // there are streaming operators for all math types
    Log::infoLog() << t2 << std::endl;
    Log::infoLog() << t3 << std::endl;
    
    
    //Exercise 2
    Vector3D<> v0;
    v0 = inverse( t11 )*inverse( t12 )*Vector3D<>(0.5,1,0);
    std::cout << "Vector (0.5,1,0) in new coordinates" << std::endl;
    Log::infoLog() << v0 << std::endl;
     */
}
