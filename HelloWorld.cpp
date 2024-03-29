 /*
 #include <rw/common/Log.hpp>

 using namespace rw::common;

 int main(int argc, char** argv) {

   Log::infoLog() << "Hey, we are printing to the RobWork log!\n";

   std::cout << "Which should just be standard out for now!" << std::endl;
}
*/
  #include <rw/rw.hpp>

  USE_ROBWORK_NAMESPACE

  using namespace robwork;

  int main(int argc, char** argv) {

    Log::infoLog() << "The using namespace enables us to call Log directly!\n";

    rw::common::Log::infoLog() << "We can still use the native namespace!\n";

    robwork::Log::infoLog() << "but also the general namespace!\n";
}
