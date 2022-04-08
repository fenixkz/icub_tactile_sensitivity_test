# ICub simulation to test the sensitivity of a tactile sensor in Gazebo

### How to run
First, compile it.
```
$ mkdir -p build/
$ cmake ../
$ make 
$ make install
```

Then in different terminals run:
```
$ yarpserver
$ cd app/scripts/
$ yarpmanager
```
In _yarpmanager_ launch in tactile_sensitivity_system all the processes. After that, run the executable of main.cpp either in the manager or in
```
$ cd build/bin/
$ ./my-project
```
Also, we need to send commands through RPC, it can be done via terminal by:
```
$ yarp rpc /service
>> upLeft
>> closeFingers
>> goRight
>> goLeft
>> touch
>> stop
>> start
```
