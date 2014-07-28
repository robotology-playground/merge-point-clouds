#include <iostream>
#include <string>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>


#define LEFT_ARM_HOME_POS_X	-0.20
#define LEFT_ARM_HOME_POS_Y	-0.35
#define LEFT_ARM_HOME_POS_Z	0.12

#define RIGHT_ARM_HOME_POS_X	-0.20
#define RIGHT_ARM_HOME_POS_Y	0.35
#define RIGHT_ARM_HOME_POS_Z	0.12

#define MAX_ARM_TRAJ_TIME  0.8

#define TORSO_HOME_POS_ROLL		0.0
#define TORSO_HOME_POS_PITCH	0.0
#define TORSO_HOME_POS_YAW		0.0

#define TORSO_ACCELERATION_YAW		50.0
#define TORSO_ACCELERATION_PITCH	50.0
#define TORSO_ACCELERATION_ROLL		50.0

#define MAX_TORSO_VELOCITY 50.0
#define KP				15.0
#define MAX_TORSO_TRAJ_TIME  3.0

#define GAZE_HOME_POS_X		-0.4
#define GAZE_HOME_POS_Y		0.0
#define GAZE_HOME_POS_Z		0.0


#define ACK                     VOCAB3('a','c','k')
#define HOME					VOCAB4('h','o','m','e')
#define SET                     VOCAB3('s','e','t')


#define ARM                     VOCAB3('a','r','m')
#define HEAD                    VOCAB4('h','e','a','d')
#define TORSO                   VOCAB4('b','o','d','y')
#define GAZE                    VOCAB4('g','a','z','e')

#define LOOK_AT					VOCAB4('l','o','o','k')
#define TRACK					VOCAB4('t','r','a','c')
#define GOTO					VOCAB4('g','o','t','o')

#define NEXT					VOCAB4('n','e','x','t')
#define STOP					VOCAB4('s','t','o','p')
#define RUN						VOCAB3('r','u','n')

#define BLOCK					VOCAB4('b','l','o','c')

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


class TorsoModule:public RFModule
{
    RpcServer handlerPort; //a port to handle messages
    double period;
	string moduleName;
	string robotName;
	Vector leftArmHomePosition;
	Vector leftArmHomeOrientation;
	Vector rightArmHomePosition;
	Vector rightArmHomeOrientation;
	Vector gazeHomePosition;
	Vector torsoHomePosition;
	Vector torsoAcceleration;
	Matrix waypoints;

	PolyDriver clientGazeCtrl;
	PolyDriver clientArmLeft;
	PolyDriver clientArmRight;
	PolyDriver clientTorso;
	
	IGazeControl *igaze;
	ICartesianControl *icartLeft;
	ICartesianControl *icartRight;
	IVelocityControl *itorsoVelocity;
	IControlMode2 *itorsoMode;
	IEncoders *iTorsoEncoder;

	int startupGazeContextID;
	int startupArmLeftContextID;
	int startupArmRightContextID;
	int index;

	double maxTorsoVelocity;
	double kp;
	double maxTorsoTrajTime;
	double maxArmTrajTime;

	public:

    double getPeriod()
    {
		return period;
    }

    bool updateModule()
    {
		return true;
    }

	bool computeArmOr()
	{
		Matrix R(3,3);
		R(0,0)= -1.0; R(0,1)= 0.0; R(0,2)= 0.0;
		R(1,0)= 0.0; R(1,1)= 0.0; R(1,2)=-1.0; 
		R(2,0)=-0.0; R(2,1)= -1.0; R(2,2)= 0.0;
		leftArmHomeOrientation = rightArmHomeOrientation = dcm2axis(R);
		return true;
	}

	bool exploreTorso(Vector target)
	{
		Vector torsoInitialJoints;
		Vector torsoActualJoints;
		Vector torsoVelocityCommand;
		Vector torsoAccCommand;
		Vector error;
		int jointsNumber=0;
		int i;
		double time= 0.0;
		
		itorsoVelocity->getAxes(&jointsNumber);
		
		torsoAccCommand.resize(jointsNumber);
		torsoVelocityCommand.resize(jointsNumber);
		torsoInitialJoints.resize(jointsNumber);
		torsoActualJoints.resize(jointsNumber);
		error.resize(jointsNumber);

		for(i =0; i< jointsNumber;i++);
			torsoAccCommand[i] = torsoAcceleration[i];

		itorsoVelocity->setRefAccelerations(torsoAccCommand.data());


		if (!iTorsoEncoder->getEncoders(torsoInitialJoints.data())){
			cout<<"Error in reading encoders."<<endl;
			return false;
		}
		
		VectorOf<int> modes(3);
		modes[0]=modes[1]=modes[2]=VOCAB_CM_VELOCITY;
		itorsoMode->setControlModes(modes.getFirst());

		torsoVelocityCommand = kp * (target-torsoInitialJoints);
		time = Time::now();


		while (norm(torsoVelocityCommand)>0.1){
			if(Time::now()-time>maxTorsoTrajTime){
				cout<<"Max time reached."<<endl;
				break;
			}
			if (norm(torsoVelocityCommand)>maxTorsoVelocity)
				torsoVelocityCommand = maxTorsoVelocity/norm(torsoVelocityCommand)*torsoVelocityCommand;
			
			itorsoVelocity->velocityMove(torsoVelocityCommand.data());
			
			if (!iTorsoEncoder->getEncoders(torsoActualJoints.data())){
				cout<<"Error in reading encoders."<<endl;				
				return false;
			}
			
			torsoVelocityCommand = kp * (target-torsoActualJoints);
			//cout<<"Actual: " << torsoActualJoints[0] << " - " << torsoActualJoints[1] << " - "<< torsoActualJoints[2]<<endl;
			Time::delay(0.02);

		}
		cout<<"Stopping velocity."<<endl;
		itorsoVelocity->stop();
		return true;
	}

    bool respond(const Bottle& command, Bottle& reply) 
    {
		
		if(command.size()==0)
        {
            reply.addString("No command received.");
            return true;
			
        }

		switch(command.get(0).asVocab()){
		case HOME:
			if(command.size()>1)
					switch(command.get(1).asVocab()){
                        case ARM:
							icartLeft->goToPoseSync(leftArmHomePosition,leftArmHomeOrientation);
							icartRight->goToPoseSync(rightArmHomePosition,rightArmHomeOrientation);
							icartRight->waitMotionDone(0.1,2);
							icartLeft->waitMotionDone(0.1,2);
							reply.addString("Arm home position reached.");
                            return true;
						case TORSO:
							exploreTorso(torsoHomePosition);
							reply.addString("Torso home position reached.");
							return true;
						case GAZE:
							igaze->lookAtFixationPoint(gazeHomePosition);
							igaze->waitMotionDone(0.1,2);
							reply.addString("Gaze home position reached.");
							return true;
						default:
							reply.addString("Wrong device for home position.");
							return true;
						}
			else{
				
				icartLeft->goToPoseSync(leftArmHomePosition,leftArmHomeOrientation);
				icartRight->goToPoseSync(rightArmHomePosition,rightArmHomeOrientation);
				igaze->lookAtFixationPoint(gazeHomePosition);
				exploreTorso(torsoHomePosition);
				igaze->waitMotionDone(0.1,2);
				icartRight->waitMotionDone(0.1,2);
				icartLeft->waitMotionDone(0.1,2);
				reply.addString("Ok, moving to home position.");
				return true;
			}
		case LOOK_AT:
			if(command.size()==4){
				Vector gazePosition(3);
				gazePosition[0] = command.get(1).asDouble();
				gazePosition[1] = command.get(2).asDouble();
				gazePosition[2] = command.get(3).asDouble();
				cout << igaze->lookAtFixationPoint(gazePosition)<<endl;
				igaze->waitMotionDone(0.2,3);
				reply.addString("Gaze position reached.");
				return true;
			}
			else{
				reply.addString("Wrong number of coordinates.");
				return true;
			}
		case TRACK:
			if(command.size()==3)
				switch(command.get(1).asVocab()){
                        case ARM:
							if (command.get(2).asString() == "on"){
								icartLeft->setTrackingMode(true);
								icartRight->setTrackingMode(true);
								reply.addString("Arm tracking mode enabled.");
							}
							else if (command.get(2).asString() == "off"){
								icartLeft->setTrackingMode(false);
								icartRight->setTrackingMode(false);
								reply.addString("Arm tracking mode disabled.");
							}
							else
								reply.addString("Wrong parameter: on/off");
							return true;
						case GAZE:
							if (command.get(2).asString() == "on"){
								igaze->setTrackingMode(true);
								reply.addString("Gaze tracking mode enabled.");
							}
							else if (command.get(2).asString() == "off"){
								igaze->setTrackingMode(false);
								reply.addString("Gaze tracking mode disabled.");
							}
							else
								reply.addString("Wrong parameter for trac: on/off");
							return true;
						default:
								reply.addString("Wrong device for tracking mode.");
								return true;

			}
			else{
				reply.addString("Missing parameters for track.");
				return true;
			}

		case BLOCK:
			if(command.size()==3)
				switch(command.get(1).asVocab()){
						case GAZE:
							if (command.get(2).asString() == "on"){
								igaze->blockEyes(true);
								reply.addString("Gaze blocking mode enabled.");
							}
							else if (command.get(2).asString() == "off"){
								igaze->blockEyes(false);
								reply.addString("Gaze blocking mode disabled.");
							}
							else
								reply.addString("Wrong parameter for blocking: on/off");
							return true;
						default:
								reply.addString("Wrong device for blocking mode.");
								return true;

			}
			else{
				reply.addString("Missing parameters for block.");
				return true;
			}

		case GOTO:
			if(command.size()==4){
				Vector torsoTarget(3);
				torsoTarget[0]  = command.get(1).asDouble();
				torsoTarget[1]  = command.get(2).asDouble();
				torsoTarget[2]  = command.get(3).asDouble();
				exploreTorso(torsoTarget);
				reply.addString("Torso position reached.");
			}
			else{
				reply.addString("Missing parameters for goto.");
				return true;
			}
			return true;
			
		case RUN: 
			reply.addString("Ready for exploration. next or stop?");
			return true;
		case NEXT:
			exploreTorso(waypoints.getRow(index));
			index++;
			if (index > 4){
				reply.addString("Waypoint reached. End of waypoints.");
				index = 0;
			}
			else{
				reply.addString("Waypoint reached. next or stop?");
			}
			return true;
		case STOP:
			index = 0;
			reply.addString("Run stopped. Index reset.");
			return true;
			

		default:
                RFModule::respond(command,reply);
				return true;
		}
        return true;

    }
		
    bool configure(yarp::os::ResourceFinder &rf)
    {
		Vector newDof, curDof;

		cout<<"Configuring module!"<<endl;

		moduleName=rf.check("name",Value("torsoModule")).asString().c_str();
		robotName=rf.check("robot",Value("icubSim")).asString().c_str();
		period=rf.check("period",Value(0.2)).asDouble();
		kp=rf.check("kp",Value(KP)).asDouble();
		maxTorsoTrajTime=rf.check("torsoTime",Value(MAX_TORSO_TRAJ_TIME)).asDouble();
        maxTorsoVelocity=rf.check("maxTorsoVelocity",Value(MAX_TORSO_VELOCITY)).asDouble();
		maxArmTrajTime=rf.check("armTime",Value(MAX_ARM_TRAJ_TIME)).asDouble();

		handlerPort.open(("/"+moduleName+"/rpc:i").c_str());
        attach(handlerPort);


		Property optionGaze;
		optionGaze.put("device","gazecontrollerclient");
		optionGaze.put("remote","/iKinGazeCtrl");
		optionGaze.put("local",("/"+moduleName+"/gaze").c_str());
		if(!clientGazeCtrl.open(optionGaze)){
			cout<<"Error opening gaze client!"<<endl;
			return false;
		}
		clientGazeCtrl.view(igaze);
		igaze->storeContext(&startupGazeContextID);
		cout<<"Setting:"<<endl;
		gazeHomePosition.push_back(GAZE_HOME_POS_X);
		gazeHomePosition.push_back(GAZE_HOME_POS_Y);
		gazeHomePosition.push_back(GAZE_HOME_POS_Z);
		
		Property leftArmOption;
		leftArmOption.put("device","cartesiancontrollerclient");
		leftArmOption.put("remote",("/"+robotName+"/cartesianController/left_arm").c_str());
		leftArmOption.put("local",("/"+moduleName+"/left_arm").c_str());

		if(!clientArmLeft.open(leftArmOption)){
			cout<<"Error opening left arm client!"<<endl;
			return false;
		}
		
		clientArmLeft.view(icartLeft);
		icartLeft->storeContext(&startupArmLeftContextID);

		leftArmHomePosition.push_back(LEFT_ARM_HOME_POS_X);
		leftArmHomePosition.push_back(LEFT_ARM_HOME_POS_Y);
		leftArmHomePosition.push_back(LEFT_ARM_HOME_POS_Z);
		
		cout<<"MAX ARM TRAJ TIME: "<< maxArmTrajTime<<endl;
		icartLeft->setTrajTime(maxArmTrajTime);

		icartLeft->getDOF(curDof);
        newDof=curDof;
		newDof[3]=0;
		icartLeft->setDOF(newDof,curDof);
		

		Property rightArmOption;
		rightArmOption.put("device","cartesiancontrollerclient");
		rightArmOption.put("remote",("/"+robotName+"/cartesianController/right_arm").c_str());
		rightArmOption.put("local",("/"+moduleName+"/right_arm").c_str());

		if(!clientArmRight.open(rightArmOption)){
			cout<<"Error opening right arm client!"<<endl;
			return false;
		}
		
		clientArmRight.view(icartRight);
		icartRight->storeContext(&startupArmRightContextID);

		rightArmHomePosition.push_back(RIGHT_ARM_HOME_POS_X);
		rightArmHomePosition.push_back(RIGHT_ARM_HOME_POS_Y);
		rightArmHomePosition.push_back(RIGHT_ARM_HOME_POS_Z);
				
		icartRight->setTrajTime(maxArmTrajTime);

		icartRight->getDOF(curDof);
        newDof=curDof;
		newDof[3]=0;
		icartRight->setDOF(newDof,curDof);

		computeArmOr();

		Property torsoOptions;
		torsoOptions.put("device", "remote_controlboard");
		torsoOptions.put("remote",("/"+robotName+"/torso").c_str());
		torsoOptions.put("local",("/"+moduleName+"/torso").c_str()); 
	
		if(!clientTorso.open(torsoOptions)){
			cout<<"Error opening torso client!"<<endl;
			return false;
		}
		
		clientTorso.view(itorsoVelocity);
		clientTorso.view(itorsoMode);
		torsoHomePosition.push_back(TORSO_HOME_POS_ROLL);
		torsoHomePosition.push_back(TORSO_HOME_POS_PITCH);
		torsoHomePosition.push_back(TORSO_HOME_POS_YAW);

		torsoAcceleration.push_back(TORSO_ACCELERATION_ROLL);
		torsoAcceleration.push_back(TORSO_ACCELERATION_PITCH);
		torsoAcceleration.push_back(TORSO_ACCELERATION_YAW);

		clientTorso.view(iTorsoEncoder);


		waypoints.resize(5,3);
		waypoints(0,0) = 10.0; waypoints(0,1) = 10.0; waypoints(0,2) = 20.0; 
		waypoints(1,0) = 30.0; waypoints(1,1) = 20.0; waypoints(1,2) = 20.0; 
		waypoints(2,0) = 0.0; waypoints(2,1) = 0.0; waypoints(2,2) = 0.0; 
		waypoints(3,0) = -10.0; waypoints(3,1) = -10.0; waypoints(3,2) = 20.0; 
		waypoints(4,0) = -30.0; waypoints(4,1) = -20.0; waypoints(4,2) = 20.0; 
		index = 0;
		cout<<endl;
        return true;
    }

    bool interruptModule()
    {
        cout<<"Interrupt caught!"<<endl;
		cout<<endl;
        return true;
    }

    bool close()
    {
        handlerPort.close();

		igaze->stopControl();
		igaze->restoreContext(startupGazeContextID);
		igaze->deleteContext(startupGazeContextID);

		if (clientGazeCtrl.isValid())
			clientGazeCtrl.close();

		icartLeft->stopControl();
		icartLeft->restoreContext(startupArmLeftContextID);
		icartLeft->deleteContext(startupArmLeftContextID);

		if (clientArmLeft.isValid())
			clientArmLeft.close();

		icartRight->stopControl();
		icartRight->restoreContext(startupArmRightContextID);
		icartRight->deleteContext(startupArmRightContextID);

		if (clientArmLeft.isValid())
			clientArmLeft.close();

		itorsoVelocity->stop();
		
		if (clientTorso.isValid())
		clientTorso.close();
		
		cout<<endl;
        return true;
    }
};

int main(int argc, char * argv[])
{

	YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
	TorsoModule module;
	ResourceFinder rf;

    if (!yarp.checkNetwork())
    {
        cout<<"YARP server not available!"<<endl;
        return false;
    }

	rf.configure(argc, argv);

	cout<<"Running module..."<<endl;
    
	return module.runModule(rf);
}


