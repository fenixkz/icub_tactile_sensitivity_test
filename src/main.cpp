
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <mutex>
#include <unistd.h>
#include <thread>
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
    enum class State { upLeft, closeFingers, goRight, goLeft, touch, periodicMovement, home, idle, stop, ready};
    State state;
    int emerStop;
    mutex mutexState;
    mutex mutexControl;
    string robot;
    IControlLimits   *ilim;
    IControlMode     *imod;
    IPositionControl *ipos;
    vector<double> poseR, poseL, up_left, homeR, homeL, homeT, fingers_target, fingers_joint;
    double trajectory_time, period, ampl, w;
    void *IControl;
    double t0 = Time::now();
    double tS = Time::now();
    bool cartesian;
    /***************************************************/
    void approachBox()
    {

        drvArmR.view(iarm);
        Vector curDof;
        iarm->getDOF(curDof);
        Vector newDof(3);
        newDof[0] = 1;
        newDof[1] = 0;
        newDof[2] = 1;
        iarm->setDOF(newDof,curDof);

        Vector x,o, xd(3), od(4);
        iarm->getPose(x,o);

        xd[0] = poseR[0];
        xd[1] = poseR[1];
        xd[2] = poseR[2];

        Vector o1(4), o2(4);
        Matrix R1, R2, R;
        o1[0] = 0.0; o1[1] = 1.0; o1[2] = 0.0; o1[3] = M_PI;
        o2[0] = 1.0; o2[1] = 0.0; o2[2] = 0.0; o2[3] = M_PI/2;
        R1=yarp::math::axis2dcm(o1);
        R2=yarp::math::axis2dcm(o2);
        R = R1 * R2;
        od = yarp::math::dcm2axis(R);

        iarm->goToPose(xd, od, trajectory_time);

    }

    bool periodicMov(){
      drvArmL.view(iarm);
      Vector o1(4), o2(4), o3(4), xg(4), od(4);
      Matrix R1, R2, R3, R;
      o1[0] = 0.0; o1[1] = 1.0; o1[2] = 0.0; o1[3] = M_PI;
      o2[0] = 1.0; o2[1] = 0.0; o2[2] = 0.0; o2[3] = M_PI/2;
      o3[0] = 0.0; o3[1] = 0.0; o3[2] = 1.0; o3[3] = M_PI/6;
      R1=yarp::math::axis2dcm(o1);
      R2=yarp::math::axis2dcm(o2);
      R3=yarp::math::axis2dcm(o3);
      R = R1 * R2 * R3;
      od = yarp::math::dcm2axis(R);
      // snprintf( buffer, sizeof(buffer), "xA: %f; ", tmp[2]);
      // myfile << buffer;
      xg[1] = poseL[1];
      xg[0] = poseL[0];
      double t = Time::now();
      xg[2] = poseL[2] + ampl*sin(2.0 * M_PI * 1/w * (t-tS));
      iarm->goToPose(xg,od);
      // snprintf( buffer, sizeof(buffer), "xG: %f\n", xg[2]);
      // myfile << buffer;
      return true;
    }

    void moveLeft()
    {
        // select the correct interface
        drvArmL.view(iarm);
        Vector curDof;
        iarm->getDOF(curDof);
        Vector newDof(3);
        newDof[0] = 0;
        newDof[1] = 0;
        newDof[2] = 0;
        iarm->setDOF(newDof,curDof);

        Vector x,o, xd(3), od(4);
        iarm->getPose(x,o);

        xd[0] = poseL[0];
        xd[1] = poseL[1];
        xd[2] = poseL[2];

        Vector o1(4), o2(4), o3(4);
        Matrix R1, R2, R3, R;
        o1[0] = 0.0; o1[1] = 1.0; o1[2] = 0.0; o1[3] = M_PI;
        o2[0] = 1.0; o2[1] = 0.0; o2[2] = 0.0; o2[3] = M_PI/2;
        o3[0] = 0.0; o3[1] = 0.0; o3[2] = 1.0; o3[3] = M_PI/6;
        R1=yarp::math::axis2dcm(o1);
        R2=yarp::math::axis2dcm(o2);
        R3=yarp::math::axis2dcm(o3);
        R = R1 * R2 * R3;
        od = yarp::math::dcm2axis(R);

        iarm->goToPoseSync(xd,od, trajectory_time);
    }

    void moveIndex(){
      IControlLimits   *ilim;
      IControlMode     *imod;
      IPositionControl *ipos;
      IEncoders        *ienc;
      drvHandL.view(ilim);
      drvHandL.view(imod);
      drvHandL.view(ipos);
      drvHandL.view(ienc);
      int j = 11;
      imod->setControlMode(j,VOCAB_CM_POSITION);
      double enc;
      ienc->getEncoder(j,&enc);
      double target = enc + 2;
      ipos->positionMove(j,target);

    }

    void leftHand(){
      drvArmL.view(iarm);
      Vector xd(3), od(4);
      xd[0]=up_left[0];
      xd[1]=up_left[1];
      xd[2]=up_left[2];
      Vector o1(4), o2(4);
      Matrix R1, R2, R;
      o1[0] = 0.0; o1[1] = 1.0; o1[2] = 0.0; o1[3] = M_PI;
      o2[0] = 1.0; o2[1] = 0.0; o2[2] = 0.0; o2[3] = M_PI/2;
      R1=yarp::math::axis2dcm(o1);
      R2=yarp::math::axis2dcm(o2);
      R = R1 * R2;
      od = yarp::math::dcm2axis(R);

      iarm->goToPose(xd,od, trajectory_time);
    }

    bool goHomeTorso(){
      vector<double> targets;
      drvTorso.view(imod);
      drvTorso.view(ipos);
      targets = homeT;
      VectorOf<int> Tjoints {0,1,2};
      VectorOf<int> Tmodes {VOCAB_CM_POSITION, VOCAB_CM_POSITION, VOCAB_CM_POSITION};
      imod->setControlModes(Tjoints.size(), Tjoints.data(), Tmodes.data());
      ipos->positionMove(Tjoints.size(), Tjoints.data(), targets.data());
      return true;
    }
    bool goHomeHands(const string hand){
      vector<double> targets;
      if (hand == "left"){
        drvHandL.view(imod);
        drvHandL.view(ipos);
        targets = homeL;
        VectorOf<int> joints {0,1,2,3,4,5,6};
        VectorOf<int> modes {VOCAB_CM_POSITION, VOCAB_CM_POSITION, VOCAB_CM_POSITION, VOCAB_CM_POSITION,
        VOCAB_CM_POSITION, VOCAB_CM_POSITION, VOCAB_CM_POSITION};
        imod->setControlModes(joints.size(), joints.data(), modes.data());
        ipos->positionMove(joints.size(), joints.data(), targets.data());
      }else{
        drvHandR.view(imod);
        drvHandR.view(ipos);
        VectorOf<int> joints {0,1,2,3,4,5,6};
        VectorOf<int> modes {VOCAB_CM_POSITION, VOCAB_CM_POSITION, VOCAB_CM_POSITION, VOCAB_CM_POSITION,
        VOCAB_CM_POSITION, VOCAB_CM_POSITION, VOCAB_CM_POSITION};
        targets = homeR;
        imod->setControlModes(joints.size(), joints.data(), modes.data());
        ipos->positionMove(joints.size(), joints.data(), targets.data());
      }
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

    void moveFingers(const string &hand)
    {
        // IEncoders        *ienc;
        if (hand=="right")
        {
            drvHandR.view(ilim);
            drvHandR.view(imod);
            drvHandR.view(ipos);
        }
        else
        {
            drvHandL.view(ilim);
            drvHandL.view(imod);
            drvHandL.view(ipos);
        }

        double target;
        VectorOf<int> middle {13,14};
        VectorOf<int> thumb {8,9,10};
        int pinky = 15;
        //midle
        for (size_t i=0; i<middle.size(); i++)
        {
              int j=middle[i];
              // ilim->getLimits(j,&min,&max);
              // yInfo() << "Joint %d, max %f" << j, max;
              // fprintf(stdout, "Joint %d, max %f\n", j, max);
              imod->setControlMode(j,VOCAB_CM_POSITION);
              target = fingers_target[i];
              // set up the speed in [deg/s]
              ipos->setRefSpeed(j,30.0);
              // set up max acceleration in [deg/s^2]
              ipos->setRefAcceleration(j,50.0);
              ipos->positionMove(j,target);
        }
        bool done = false;
        while(!done){
          Time::delay(0.1);
          ipos->checkMotionDone(&done);
        }
        // ilim->getLimits(pinky, &min, &max);
        imod->setControlMode(pinky,VOCAB_CM_POSITION);
        target = fingers_target[2];
        // fprintf(stdout, "Joint %d, max %f\n", pinky, max);
        ipos->setRefSpeed(pinky,30.0);
        ipos->setRefAcceleration(pinky,50.0);
        ipos->positionMove(pinky,target);
        done = false;
        while(!done){
          Time::delay(0.1);
          ipos->checkMotionDone(&done);
        }
        //thumb
        for (size_t i=0; i<thumb.size(); i++)
        {
              int j=thumb[i];
              // ilim->getLimits(j,&min,&max);
              imod->setControlMode(j,VOCAB_CM_POSITION);
              target = fingers_target[i+3];
              // set up the speed in [deg/s]
              ipos->setRefSpeed(j,30.0);
              // set up max acceleration in [deg/s^2]
              ipos->setRefAcceleration(j,50.0);
              // fprintf(stdout, "Joint %d, max %f\n", j, max);
              ipos->positionMove(j,target);
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
        // poseR = rf.findGroup("hand right").find("x").asFloat32();
        Bottle* b = rf.findGroup("right").find("pose").asList();
        poseR.resize(b->size());
        for (size_t i = 0; i < b->size(); i++){
          Value item_v = b->get(i);
          if (item_v.isNull()){
            cout << "Error: " << i << " is null" << endl;
            return false;
          }
          poseR.at(i) = item_v.asFloat32();
        }

        b = rf.findGroup("left").find("pose").asList();
        Bottle* bottle_up = rf.findGroup("left").find("up").asList();
        poseL.resize(b->size());
        up_left.resize(bottle_up->size());
        for (size_t i = 0; i < b->size(); i++){
          Value item_v = b->get(i);
          Value item_u = bottle_up->get(i);
          if (item_v.isNull() || item_u.isNull()){
            cout << "Error: " << i << " is null" << endl;
            return false;
          }
          up_left.at(i) = item_u.asFloat32();
          poseL.at(i) = item_v.asFloat32();
        }



        Bottle* bottle_home_left = rf.findGroup("home").find("arm_left").asList();
        Bottle* bottle_home_right = rf.findGroup("home").find("arm_right").asList();

        homeL.resize(bottle_home_left->size());
        homeR.resize(bottle_home_right->size());
        for (size_t i = 0; i < bottle_home_left->size(); i++){
          Value item_l = bottle_home_left->get(i);
          Value item_r = bottle_home_right->get(i);
          if (item_l.isNull() || item_r.isNull()){
            cout << "Error: " << i << " is null" << endl;
            return false;
          }
          homeL.at(i) = item_l.asFloat32();
          homeR.at(i) = item_r.asFloat32();
        }

        b = rf.findGroup("home").find("torso").asList();
        homeT.resize(b->size());
        for (size_t i = 0; i < b->size(); i++){
          Value item_v = b->get(i);
          if (item_v.isNull()){
            cout << "Error: " << i << " is null" << endl;
          }
          homeT.at(i) = item_v.asFloat32();
        }

        b = rf.findGroup("fingers").find("target").asList();
        fingers_target.resize(b->size());
        for (size_t i = 0; i < b->size(); i++){
          Value item_v = b->get(i);
          if (item_v.isNull()){
            cout << "Error: " << i << " is null" << endl;
          }
          fingers_target.at(i) = item_v.asFloat32();
        }

        trajectory_time = rf.check("trajectory_time", Value(10.0)).asFloat32();
        period = rf.check("frequency", Value(1.0)).asFloat32();
        ampl = rf.check("amplitude", Value(0.05)).asFloat32();
        w = rf.check("omega", Value(3.0)).asFloat32();
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
        drvArmR.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_right);
        drvArmL.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_left);
        rpcPort.open("/service");
        attach(rpcPort);
        state = State::ready;
        emerStop = 0;
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
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();

        if (cmd=="goRight")
        {
            setState(State::goRight);
            reply.addString("Done");
        }
        else if (cmd == "upLeft"){
          setState(State::upLeft);
          reply.addString("Done");
        }
        else if(cmd == "closeFingers"){
          setState(State::closeFingers);
          reply.addString("Done");
        }
        else if(cmd == "goLeft"){
          setState(State::goLeft);
          reply.addString("Done");
        }
        else if(cmd == "touch"){
          setState(State::touch);
          reply.addString("Done");
        }
        else if(cmd == "home"){
          setState(State::home);
          reply.addString("Done");
        }
        else if(cmd == "stop"){

          emerStop = 1;
          yInfo() << "Stop has been pressed";
          stop();
          setState(State::stop);
          reply.addString("Done");
        }
        else if(cmd == "start"){
          emerStop = 0;
          setState(State::ready);
          yInfo() << "Reset has been pressed";
          // reset();
          reply.addString("Done");
        }else if (cmd=="periodic"){
            tS = Time::now();
            setState(State::periodicMovement);
            reply.addString("Done");
        }else if (cmd == "pause"){
            setState(State::ready);
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        return true;
        //x: -0.305561, y: 0.201069, z: -0.073300
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
          State localState = getState();
          if (localState == State::goRight){
            yInfo() << "Approaching the box";
            approachBox();
            t0=Time::now();
            cartesian = true;
            setState(State::idle);
          }else if (localState == State::upLeft){
            yInfo() << "Moving the left hand";
            leftHand();
            cartesian = true;
            t0=Time::now();
            setState(State::idle);
          }else if (localState == State::closeFingers){
            yInfo() << "Moving the fingers of the left hand";
            moveFingers("left");
            cartesian = false;
            t0=Time::now();
            setState(State::idle);
          }else if (localState == State::goLeft){
            yInfo() << "Moving the left hand";
            moveLeft();
            t0=Time::now();
            cartesian = true;
            setState(State::idle);
          }else if (localState == State::touch){
            yInfo() << "Moving the index finger";
            moveIndex();
            t0=Time::now();
            cartesian = false;
            setState(State::idle);
          }else if (localState == State::periodicMovement){
              periodicMov();
              yInfo() << "Up and down again";
          }else if (localState == State::home){
            yInfo() << "Moving to the home position";
            drvArmL.view(iarm);
            iarm->stopControl();

            drvArmR.view(iarm);
            iarm->stopControl();
            moveShoulders("right");
            // std::this_thread::sleep_for(1.5s);;
            yInfo() << "Now";
            std::this_thread::sleep_for(2s);
            yInfo() << "Now + 2s";
            moveShoulders("left");
            std::this_thread::sleep_for(2s);;
            goHomeTorso();
            std::this_thread::sleep_for(2s);;
            goHomeHands("left");
            std::this_thread::sleep_for(2s);;
            goHomeHands("right");
            t0=Time::now();
            cartesian = false;
            setState(State::idle);
          }else if (localState == State::idle){
            bool done = false;
            cartesian?iarm->checkMotionDone(&done):ipos->checkMotionDone(&done);
            if (done || ((Time::now() - t0) > 10.0)){
              setState(State::ready);
              yInfo() << "Motion has been completed";
              setState(State::ready);
            }
          }
          else{
            yInfo() << "Doing nothing";
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
