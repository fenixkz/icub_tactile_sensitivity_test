
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArmR, drvArmL, drvHandR, drvHandL;
    ICartesianControl *iarm;
    int startup_ctxt_arm_right;
    int startup_ctxt_arm_left;
    int startup_ctxt_gaze;
    Vector x_init, o_init;
    RpcServer rpcPort;


    /***************************************************/
    void approachBox()
    {
        // select the correct interface
        drvArmR.view(iarm);
        Vector curDof;
        iarm->getDOF(curDof);
        Vector newDof(3);
        newDof[0] = 1;
        newDof[1] = 0;
        newDof[2] = 1;
        iarm->setDOF(newDof,curDof);

        // enable all dofs but the roll of the torso
        Vector x,o, xd(3), od(4);
        iarm->getPose(x,o);
        // xd[0]=x[0];
        // xd[1]=x[1];
        // xd[2]=x[2];
        xd[0] = x[0] + 0.02;
        xd[1] = 0.201 - 0.08 - 0.1;
        xd[2] = -0.0733 - 0.02;

        Vector o1(4), o2(4);
        Matrix R1, R2, R;
        o1[0] = 0.0; o1[1] = 1.0; o1[2] = 0.0; o1[3] = M_PI;
        o2[0] = 1.0; o2[1] = 0.0; o2[2] = 0.0; o2[3] = M_PI/2;
        R1=yarp::math::axis2dcm(o1);
        R2=yarp::math::axis2dcm(o2);
        R = R1 * R2;
        od = yarp::math::dcm2axis(R);
        // go to the target :)
        // (in streaming)
        iarm->goToPose(xd,od);
        // iarm->waitMotionDone(0.04);
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

        // enable all dofs but the roll of the torso
        Vector x,o, xd(3), od(4);
        iarm->getPose(x,o);
        // xd[0]=x[0];
        // xd[1]=x[1];
        // xd[2]=x[2];
        xd[0] = -0.283348 + 0.08;
        xd[1] = 0.201 - 0.08 - 0.146;
        xd[2] = -0.0733 - 0.02;

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
        // go to the target :)
        // (in streaming)
        iarm->goToPoseSync(xd,od);
        // iarm->waitMotionDone(0.04);
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
      Vector x,o, xd(3), od(4);
      iarm->getPose(x,o);
      xd[0]=x[0];
      xd[1]=x[1];
      xd[2]=x[2] + 0.2;
      Vector o1(4), o2(4);
      Matrix R1, R2, R;
      o1[0] = 0.0; o1[1] = 1.0; o1[2] = 0.0; o1[3] = M_PI;
      o2[0] = 1.0; o2[1] = 0.0; o2[2] = 0.0; o2[3] = M_PI/2;
      R1=yarp::math::axis2dcm(o1);
      R2=yarp::math::axis2dcm(o2);
      R = R1 * R2;
      od = yarp::math::dcm2axis(R);
      // go to the target :)
      // (in streaming)
      iarm->goToPose(xd,od);
      // iarm->waitMotionDone(0.04);
    }
    void reset(){
      drvArmR.view(iarm);
      iarm->goToPoseSync(x_init,o_init);
      iarm->waitMotionDone(0.04);
    }

    void moveFingers(const string &hand)
    {
        // select the correct interface
        IControlLimits   *ilim;
        IControlMode     *imod;
        IPositionControl *ipos;
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

        // enforce [0,1] interval
        // double fingers_closure_sat=std::min(1.0,std::max(0.0,fingers_closure));
        double min, max, target;
        VectorOf<double> targets;
        VectorOf<int> joints {8,9,10,13,14,15};
        // move each finger first:
        // if min_j and max_j are the minimum and maximum bounds of joint j,
        // then we should move to min_j+fingers_closure_sat*(max_j-min_j)
        for (size_t i=0; i<joints.size(); i++)
        {
            int j=joints[i];
            ilim->getLimits(j,&min,&max);
            imod->setControlMode(j,VOCAB_CM_POSITION);
            target = max;
            targets.push_back(target);
            // set up the speed in [deg/s]
            ipos->setRefSpeed(j,30.0);

            // set up max acceleration in [deg/s^2]
            ipos->setRefAcceleration(j,50.0);
            ipos->positionMove(j,target);
            // FILL IN THE CODE
        }
        // double enc;
        // wait until all fingers have attained their set-points
        // for (size_t i=0; i<joints.size(); i++)
        // {
        //   int j=joints[i];
        //   ipos->checkMotionDone(&done);
        // }
        // FILL IN THE CODE
        bool done = false;
        while(!done){
          Time::delay(0.1);
          ipos->checkMotionDone(&done);
        }

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

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icubSim")).asString();

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


        rpcPort.open("/service");
        attach(rpcPort);

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

            yInfo() << "Approaching the box";
            approachBox();
            yInfo() << "Done";
            Vector x,o;
            drvArmR.view(iarm);
            iarm->getPose(x,o);
            fprintf(stdout, "Current x: %f, y: %f, z: %f\n", x[0], x[1], x[2]);
            // yInfo() << "Current x: %f, y: %f, z: %f" << x[0], x[1], x[2];
            reply.addString("Done");
            // we assume the robot is not moving now
        }
        else if (cmd == "reset"){
          yInfo() << "Resetting";
          reset();
          yInfo() << "Done";
          reply.addString("Resetted");
        }
        else if (cmd == "upLeft"){
          yInfo() << "Moving the left hand";
          leftHand();
          yInfo() << "Done";
          reply.addString("Done");
        }
        else if(cmd == "closeFingers"){
          yInfo() << "Moving the fingers of the left hand";
          moveFingers("left");
          yInfo() << "Done";
          reply.addString("Done");
        }
        else if(cmd == "goLeft"){
          yInfo() << "Moving the left hand";
          moveLeft();
          yInfo() << "Done";
          reply.addString("Done");
        }
        else if(cmd == "touch"){
          yInfo() << "Moving the index finger";
          moveIndex();
          yInfo() << "Done";
          reply.addString("Done");
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
        return 1.0;
    }

    /***************************************************/
    bool updateModule()
    {
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