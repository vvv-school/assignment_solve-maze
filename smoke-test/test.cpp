/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;


/**********************************************************************/
class TestAssignmentSolveMaze : public yarp::robottestingframework::TestCase {
  BufferedPort<Property> portRadar;

 public:
  /******************************************************************/
  TestAssignmentSolveMaze() :
      yarp::robottestingframework::TestCase("TestAssignmentSolveMaze") {
  }

  /******************************************************************/
  virtual ~TestAssignmentSolveMaze() {
  }

  /******************************************************************/
  bool setup(yarp::os::Property& property)override {
    portRadar.open("/" + getName() + "/radar:i");
    if (!Network::connect("/assignment_solve-maze-handler/radar:o",portRadar.getName()))
        ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to maze handler");
    return true;
  }

  /******************************************************************/
  void tearDown()override {
    portRadar.close();
  }

  /******************************************************************/
  void run()override {
    ROBOTTESTINGFRAMEWORK_TEST_REPORT("Checking maze status");

    string state;
    while (true) {
      Property* radar = portRadar.read();
      state = radar->find("state").asString();
      if (state != "running")
        break;
    }

    ROBOTTESTINGFRAMEWORK_TEST_CHECK(state == "solved",
                   Asserter::format("Final state = %s", state.c_str()));
  }
};

ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TestAssignmentSolveMaze)
