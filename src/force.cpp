
#include <string>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <mutex>
#include <unistd.h>
#include <thread>
#include <fstream>
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace literals::chrono_literals;

/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArmR, drvArmL, drvHandR, drvHandL, drvTorso;
    ICartesianControl *iarm;
    int startup_ctxt_arm_right;
    int startup_ctxt_arm_left;
    int startup_ctxt_gaze;
    Vector x_init, o_init;
    RpcServer rpcPort;
    enum class State { periodicMovement, idle, stop, ready};
    State state;
    int emerStop;
    mutex mutexState;
    mutex mutexControl;
    string robot;
    IControlLimits   *ilim;
    IControlMode     *imod;
    IPositionControl *ipos;
    double trajectory_time, period;
    vector<double> xL, data;
    void *IControl;
    double t0 = Time::now();
    bool cartesian;
    ofstream myfile;
    char buffer[ 64 ];
    Port tactPort;
    Bottle tactData;
    /***************************************************/

    bool periodicMov(){
      drvArmL.view(iarm);
      Vector tmp,o, xg;
      tmp.resize(3);
      xg.resize(3);
      o.resize(3);
      iarm->getPose(tmp,o);
      // snprintf( buffer, sizeof(buffer), "xA: %f; ", tmp[2]);
      // myfile << buffer;
      xg[0] = xL[0];
      xg[1] = xL[1];
      double t = Time::now();
      xg[2] = xL[2] + 0.05*sin(2.0 * M_PI * 0.02 * (t-t0));
      iarm->goToPose(xg,o);
      // snprintf( buffer, sizeof(buffer), "xG: %f\n", xg[2]);
      // myfile << buffer;
      return true;
    }

    State getState(){
      mutexState.lock();
      State localState = state;
      mutexState.unlock();
      return localState;
    }

    void setState(const State state_value){
      if (!emerStop){
        mutexState.lock();
        state = state_value;
        mutexState.unlock();
      }else{
        yInfo() << "Emergency stop is active, cannot change the state until the start has pressed";
      }
    }


    void moveShoulders(const string hand){
      int j = 1;
      PolyDriver &drvHand = (hand=="right"?drvHandR:drvHandL);
      double target = 90.0;
      drvHand.view(imod);
      drvHand.view(ipos);
      imod->setControlMode(j,VOCAB_CM_POSITION);
      ipos->positionMove(j,target);
    }

    void stop(){
      mutexControl.lock();

      drvArmL.view(iarm);
      iarm->stopControl();

      drvArmR.view(iarm);
      iarm->stopControl();

      drvHandL.view(ipos);
      ipos->stop();

      drvHandR.view(ipos);
      ipos->stop();

      drvTorso.view(ipos);
      ipos->stop();
      mutexControl.unlock();
    }


    bool reset(){

      if (!openCartesian(robot,"right_arm"))
          return false;

      if(!openCartesian(robot, "left_arm"))
          return false;

      if(!openHand(robot, "left_arm"))
          return false;
      // save startup contexts
      drvArmR.view(iarm);
      iarm->storeContext(&startup_ctxt_arm_right);
      drvArmL.view(iarm);
      iarm->storeContext(&startup_ctxt_arm_left);
      return true;
    }

    bool readTactileData(){
      tactPort.read(tactData);
      cout << "I am here" << endl;
      data.resize(60);
      for (size_t i = 0; i < 60; i++){
        Value item = tactData.get(i);
        data.at(i) = item.asInt32();
      }
      std::stringstream result;
      std::copy(data.begin(), data.end(), std::ostream_iterator<int>(result, " "));
      cout << result.str() << endl;
      // fprintf(stdout, "%s\n", result);
      return true;
    }

    /***************************************************/
    bool openCartesian(const string &robot, const string &arm)
    {
        PolyDriver &drvArm = (arm=="right_arm"?drvArmR:drvArmL);

        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/"+robot+"/cartesianController/"+arm);
        optArm.put("local","/cartesian_client/"+arm);

        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }

        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller for "<<arm;
            return false;
        }
        return true;
    }

    bool openHand(const string &robot, const string &arm){
      PolyDriver &drvHand=(arm=="right_arm"?drvHandR:drvHandL);
      Property optJoint;
      optJoint.put("device","remote_controlboard");
      optJoint.put("remote","/"+robot+"/" +arm);
      optJoint.put("local","/position/"+arm);
      bool ok=false;
      double t0=Time::now();
      while (Time::now()-t0<10.0)
      {
          // this might fail if controller
          // is not connected to solver yet
          if (drvHand.open(optJoint))
          {
              ok=true;
              break;
          }

          Time::delay(1.0);
      }

      if (!ok)
      {
          yError()<<"Unable to open the Hand Controller";
          return false;
      }
      return true;
    }

    bool openTorso(const string &robot){
      Property optJoint;
      optJoint.put("device","remote_controlboard");
      optJoint.put("remote","/"+robot+"/torso");
      optJoint.put("local","/position/torso");
      bool ok=false;
      double t0=Time::now();
      while (Time::now()-t0<10.0)
      {
          // this might fail if controller
          // is not connected to solver yet
          if (drvTorso.open(optJoint))
          {
              ok=true;
              break;
          }

          Time::delay(1.0);
      }

      if (!ok)
      {
          yError()<<"Unable to open the Torso Controller";
          return false;
      }
      return true;
    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        robot=rf.check("robot",Value("icubSim")).asString();
        Bottle* b = rf.findGroup("left").find("pose").asList();
        xL.resize(b->size());
        for (size_t i = 0; i < b->size(); i++){
          Value item_v = b->get(i);
          if (item_v.isNull()){
            cout << "Error: " << i << " is null" << endl;
            return false;
          }
          xL.at(i) = item_v.asFloat32();
        }
        trajectory_time = rf.check("trajectory_time", Value(10.0)).asFloat32();
        period = rf.check("frequency", Value(1.0)).asFloat32();

        if (!openCartesian(robot,"right_arm"))
            return false;

        if(!openCartesian(robot, "left_arm"))
            return false;

        if(!openHand(robot, "right_arm"))
            return false;

        if(!openHand(robot, "left_arm"))
            return false;

        if(!openTorso(robot))
            return false;
        // save startup contexts
        drvArmR.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_right);
        drvArmL.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_left);
        tactPort.open("/tactile");
        Network::connect("/icub/skin/left_hand", "/tactile");
        rpcPort.open("/serviceForce");
        attach(rpcPort);
        state = State::ready;
        emerStop = 0;
        // myfile.open ("data.txt", ios::out);
        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArmR.view(iarm);
        iarm->restoreContext(startup_ctxt_arm_right);

        drvArmR.close();

        rpcPort.close();
        myfile.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();

        if (cmd=="periodic")
        {
            t0 = Time::now();

            setState(State::periodicMovement);
            reply.addString("Done");
        }else if (cmd == "pause"){
          setState(State::idle);
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return period;
    }

    /***************************************************/
    bool updateModule()
    {
      if (!emerStop){
        readTactileData();
        State localState = getState();
        if (localState == State::periodicMovement){
            periodicMov();
            yInfo() << "Up and down again";
        }else if (localState == State::ready){
          yInfo() << "At your service...";
        }else if (localState == State::idle){
          yInfo() << "Pause...";
        }
      }
      return true;
    }
};


/***************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    CtrlModule mod;
    ResourceFinder rf;
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
