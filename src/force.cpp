
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
#include <vector>
#include <numeric>
#include <chrono>

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
    IPWMControl *ipwm;
    int startup_ctxt_arm_right;
    int startup_ctxt_arm_left;
    int startup_ctxt_gaze;
    Vector x_init, o_init;
    RpcServer rpcPort;
    enum class State { prepare, home_index, reference, idle, stop, ready};
    State state;
    int emerStop;
    mutex mutexState;
    mutex mutexControl;
    string robot;
    IControlLimits   *ilim;
    IControlMode     *imod;
    IPositionControl *ipos;
    IEncoders        *ienc;
    double trajectory_time, period, dt, ang_thr;
    vector<double> xL, data, data_offset, data_filtered;
    double t0 = Time::now();
    bool cartesian;
    ofstream myfile;
    char buffer[ 64 ];
    BufferedPort<yarp::sig::Vector> tactPort, dataPort;
    int ref, scale, i_on, n_called, aw_enabled, k_times, mean_mod;
    double mean, max, r, r_, target, igain, e_i;
    /***************************************************/

    vector<double> lowPass(vector<double> x, vector<double> y){
      double cutoff = 0.1;
      double rc = 1/(2*M_PI*cutoff); // cutoff freq of 30Hz
      double dt = 0.01;
      double alpha = dt/(rc+dt);
      vector<double> result;
      result.resize(12);
      for (int i = 0; i < 12; i++){
        result.at(i) = alpha * x.at(i) + (1-alpha) * y.at(i);
      }
      return result;
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

    void closedLoop(double t_init){
      int j = 11;
      r = quintic(t_init);
      double e = r - mean;
      // P controller
      double k = 1 / 255.0;
      // I controller
      e_i += e * period * i_on;

      target = e * k * scale + igain * e_i;

      int saturation = 0;
      // Clamping the output to PWM to [-70%; 70%]
      if (target > 70){
        target = 70;
        saturation = 1;
      }else if (target < -70){
        target = -70;
        saturation = 1;
      }
      // Anti-windup;
      // if sign(error, control output) && in saturation, then turn off the integral
      if (aw_enabled && ((saturation == 1 && e > 0) || (saturation == -1 && e < 0))){
        i_on = 0;
      }else{
        i_on = 1;
      }

      ipwm->setRefDutyCycle(j, target);
      // yInfo() << "Error: " << e << " Target: " << target << " Mean: " << mean;
      yInfo() << "Integral error: " << igain * e_i;
    }

    int index_home(){
      drvHandL.view(ipos);
      int j = 11;
      imod->setControlMode(j, VOCAB_CM_POSITION);
      double target = 0;
      ipos->positionMove(j, target);
      bool done = false;
      while (!done){
          std::this_thread::sleep_for(0.1s);;
          ipos->checkMotionDone(&done);
      }
      return 1;
    }

    double quintic(double t_init){
      double a0 = r_;
      double a3 = 10 * (ref - r_) / pow(dt, 3);
      double a4 = -15 * (ref - r_) / pow(dt, 4);
      double a5 = 6 * (ref - r_) / pow(dt, 5);
      double t = Time::now() - t_init;
      double res = 0;
      if (t > dt){
        return ref;
      }else{
        res = a0 + a3 * pow(t,3) + a4 * pow(t,4) + a5 * pow(t,5);
      }
      return res;
    }

    void stop(){
      mutexControl.lock();
      drvHandL.view(ipos);
      ipos->stop();
      mutexControl.unlock();
    }


    bool reset(){
      if(!openHand(robot, "left_arm"))
          return false;
      return true;
    }

    bool readTactileData(){
      yarp::sig::Vector *tactData = tactPort.read(false);
      if (tactData != nullptr){
        data.resize(12);
        for (size_t i = 0; i < 12; i++){

          if (k_times < 50){
            data.at(i) = abs(255 - (*tactData)[36+i]);
            data_offset.at(i) += data.at(i);
          }else{
            data.at(i) = abs(255 - (*tactData)[36+i]) - data_offset.at(i)/50;
          }
        }
        k_times++;
      }
      if (!data.empty()){
        // cout << "Some";
        // data_filtered = lowPass(data, data_filtered);
        // mean = reduce(data.begin(), data.end()) / data.size();
        if (mean_mod == 0){
          // mean = (data_filtered[1] + data_filtered[6]) + 0.5*(data_filtered[9] + data_filtered[8] + data_filtered[10]) + 0.25*(data_filtered[0] + data_filtered[7] + data_filtered[2] + data_filtered[5]);
          mean = (data[1] + data[6]) + 0.5*(data[9] + data[8] + data[10]) + 0.25*(data[0] + data[7] + data[2] + data[5]);
          mean = mean / 4.5;
          mean = mean/100 * 255;
        }else if(mean_mod == 1){
          mean = 0.5*(data[1] + data[6]);
        }else{
          mean = data[6];
        }
        // max = *max_element(data.begin(), data.end());
        // yInfo() << "Index finger: max " << *max_element(data.begin(), data.end()) << " mean " << mean;
      }
      // std::stringstream result;
      // std::copy(data.begin(), data.end(), std::ostream_iterator<int>(result, " "));
      // yInfo() << result.str();
      return true;
    }

    /***************************************************/
    bool openHand(const string &robot, const string &arm){
      PolyDriver &drvHand=(arm=="right_arm"?drvHandR:drvHandL);
      Property optJoint;
      optJoint.put("device","remote_controlboard");
      optJoint.put("remote","/"+robot+"/" +arm);
      optJoint.put("local","/position/force/"+arm);
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
        robot=rf.check("robot", Value("icubSim")).asString();
        igain = rf.check("igain", Value(0.01)).asFloat32();
        trajectory_time = rf.check("trajectory_time", Value(10.0)).asFloat32();
        period = rf.check("period", Value(1.0)).asFloat32();
        scale = rf.check("scale", Value(1)).asInt32();
        aw_enabled = rf.check("aw_enabled", Value(1)).asInt32();
        mean_mod = rf.check("mean_mod", Value(0)).asInt32();
        ang_thr = rf.check("ang_thr", Value(45.0)).asFloat32();

        if(!openHand(robot, "left_arm"))
            return false;

        tactPort.open("/tactile");
        dataPort.open("/controllerData");
        // Network::connect("/icub/skin/left_hand", "/tactile");
        rpcPort.open("/serviceForce");
        attach(rpcPort);
        state = State::ready;
        emerStop = 0;
        i_on = 1;
        n_called = 1;
        r = 0;
        target = 0;
        e_i = 0;
        k_times = 0;
        data_offset.resize(12);
        data_filtered.resize(12);
        for (int i = 0; i < 12; i++){
          data_offset.at(i) = 0;
          data_filtered.at(i) = 0;
        }
        drvHandL.view(ipwm);
        drvHandL.view(imod);
        drvHandL.view(ienc);
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

        int j = 11;
        imod->setControlMode(j, VOCAB_CM_POSITION);
        double target = 0;
        drvHandL.view(ipos);
        ipos->positionMove(j, target);
        bool done = false;
        while (!done){
            std::this_thread::sleep_for(0.1s);;
            ipos->checkMotionDone(&done);
        }
        drvHandL.close();
        rpcPort.close();
        // myfile.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();

        if (cmd=="set_mode")
        {
            int mode = command.get(1).asInt32();
            if (mode){
              setState(State::ready);
            }else{
              setState(State::idle);
            }
            reply.addString("Done");
        }else if (cmd == "set_reference"){
            dt = command.get(1).asFloat32();
            ref = command.get(2).asInt32();
            // r_ = command.get(3).asInt32();
            t0 = Time::now();
            e_i = 0;
            r_ = r;
            setState(State::reference);
            reply.addString("Done");
        }else if (cmd == "pause"){
          setState(State::idle);
        }else if (cmd == "homeIndex"){
            setState(State::home_index);
            reply.addString("Done");
        }else if (cmd == "stop"){
          emerStop = 1;
          yInfo() << "Stop has been pressed";
          stop();
          setState(State::stop);
          reply.addString("Done");
        }else if (cmd == "set_closedloop"){
          setState(State::prepare);
          reply.addString("Done");

        }else if(cmd == "start"){
          emerStop = 0;
          setState(State::ready);
          yInfo() << "Reset has been pressed";
          // reset();
          reply.addString("Done");
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

        // std::chrono::steady_clock::time_point a = std::chrono::steady_clock::now();
        // Timer clock; // Timer<milliseconds, steady_clock>
        //
        // clock.tick();
        // /* code you want to measure */
        readTactileData();
        yarp::sig::Vector& data = dataPort.prepare();
        data.resize(3);
        data[0] = r;
        data[1] = mean;
        data[2] = target;
        dataPort.write();
        State localState = getState();
        int joint = 11;
        double enc;
        ienc->getEncoder(joint, &enc);
        if (enc > ang_thr){
          setState(State::home_index);
        }
        if (localState == State::ready){
          // yInfo() << "At your service...";
        }else if (localState == State::reference){
            closedLoop(t0);
        }else if (localState == State::idle){
          yInfo() << "Pause...";
        }else if (localState == State::prepare){
          yInfo() << "Setting parameters for closed loop control";
          int j = 11;
          imod->setControlMode(j, VOCAB_CM_PWM);
          setState(State::ready);
        }else if (localState == State::home_index){
          yInfo() << "Going home";
          if(index_home())
            setState(State::ready);
        }else{
          yInfo() << "Nothing";
        }
        // std::this_thread::sleep_for(1s);
        // std::cout << getPeriod() << std::endl;
        // std::chrono::steady_clock::time_point b = std::chrono::steady_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(b - a);
        // std::cout << "Duration is: " << duration.count() << std::endl;
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
