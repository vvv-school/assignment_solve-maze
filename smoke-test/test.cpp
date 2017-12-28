/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>

#include <rtf/dll/Plugin.h>
#include <rtf/TestAssert.h>

#include <yarp/rtf/TestCase.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;


/**********************************************************************/
class TestAssignmentSolveMaze : public yarp::rtf::TestCase {
  BufferedPort<Property> portRadar;

 public:
  /******************************************************************/
  TestAssignmentSolveMaze() :
      yarp::rtf::TestCase("TestAssignmentSolveMaze") {
  }

  /******************************************************************/
  virtual ~TestAssignmentSolveMaze() {
  }

  /******************************************************************/
  bool setup(yarp::os::Property& property)override {
    portRadar.open("/" + getName() + "/radar:i");
    RTF_ASSERT_ERROR_IF_FALSE(Network::connect("/assignment_solve-maze-handler/radar:o",
                                               portRadar.getName()),
                              "Unable to connect to maze handler");
    return true;
  }

  /******************************************************************/
  void tearDown()override {
    portRadar.close();
  }

  /******************************************************************/
  void run()override {
    RTF_TEST_REPORT("Checking maze status");

    string state;
    while (true) {
      Property* radar = portRadar.read();
      state = radar->find("state").asString();
      if (state != "running")
        break;
    }

    RTF_TEST_CHECK(state == "reached",
                   Asserter::format("Final state = %s", state.c_str()));
  }
};

PREPARE_PLUGIN(TestAssignmentSolveMaze)
