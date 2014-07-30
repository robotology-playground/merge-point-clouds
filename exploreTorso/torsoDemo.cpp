#include <iostream>
#include <string>
#include <gsl/gsl_math.h>
#include <yarp/sig/all.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <iCub/ctrl/math.h>
#include <yarp/os/RpcClient.h>


#define LEFT_ARM_HOME_POS_X	-0.25
#define LEFT_ARM_HOME_POS_Y	-0.25
#define LEFT_ARM_HOME_POS_Z	0.14

#define RIGHT_ARM_HOME_POS_X	-0.25
#define RIGHT_ARM_HOME_POS_Y	0.25
#define RIGHT_ARM_HOME_POS_Z	0.14

#define MAX_ARM_TRAJ_TIME  1.0

#define TORSO_HOME_POS_ROLL		0.0
#define TORSO_HOME_POS_PITCH	0.0
#define TORSO_HOME_POS_YAW		0.0

#define TORSO_ACCELERATION_YAW		1e9
#define TORSO_ACCELERATION_PITCH	1e9
#define TORSO_ACCELERATION_ROLL		1e9

#define MAX_TORSO_VELOCITY 20.0
#define KP				0.9
#define KD				0.2
#define KI				0.2
#define DT				0.05
#define MAX_TORSO_TRAJ_TIME  4.0

#define GAZE_HOME_POS_X		-0.50
#define GAZE_HOME_POS_Y		0.0
#define GAZE_HOME_POS_Z		0.12
#define DEFAULT_VERGENCE	5.0


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

#define GET						VOCAB3('g','e','t')

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


class TorsoModule:public RFModule
{
    RpcServer handlerPort;
	RpcClient objectsPort;

    double period;
	string moduleName;
	string robotName;
	string OPCName;
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
	int currentGazeContextID;
	int startupArmLeftContextID;
	int currentArmLeftContextID;
	int startupArmRightContextID;
	int currentArmRightContextID;
	int index;

	double maxTorsoVelocity;
	double kp;
	double maxTorsoTrajTime;
	double maxArmTrajTime;

	bool running;

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
		R(1,0)= 0.0; R(1,1)= sin(CTRL_DEG2RAD*30); R(1,2)=-cos(CTRL_DEG2RAD*30); 
		R(2,0)=-0.0; R(2,1)= -cos(CTRL_DEG2RAD*30); R(2,2)= -sin(CTRL_DEG2RAD*30);
		rightArmHomeOrientation = dcm2axis(R);

		R(0,0)= -1.0; R(0,1)= 0.0; R(0,2)= 0.0;
		R(1,0)= 0.0; R(1,1)= -sin(CTRL_DEG2RAD*30); R(1,2)=-cos(CTRL_DEG2RAD*30); 
		R(2,0)=-0.0; R(2,1)= -cos(CTRL_DEG2RAD*30); R(2,2)= sin(CTRL_DEG2RAD*30);
		leftArmHomeOrientation = dcm2axis(R);

		return true;
	}

	bool exploreTorso(Vector target)
	{
		Vector torsoInitialJoints;
		Vector torsoActualJoints;
		Vector torsoVelocityCommand;
		Vector torsoAccCommand;
		Vector error,integral,derivative,preError;
		int jointsNumber=0;
		int i;
		double time= 0.0;
		
		itorsoVelocity->getAxes(&jointsNumber);
		
		torsoAccCommand.resize(jointsNumber);
		torsoVelocityCommand.resize(jointsNumber);
		torsoInitialJoints.resize(jointsNumber);
		torsoActualJoints.resize(jointsNumber);
		
		error.resize(jointsNumber);
		integral.resize(jointsNumber);
		derivative.resize(jointsNumber);
		preError.resize(jointsNumber);
		
		
		preError[0] = 0.0;
		preError[1] = 0.0;
		preError[2] = 0.0;

		integral[0] = 0.0;
		integral[1] = 0.0;
		integral[2] = 0.0;

		derivative[0] = 0.0;
		derivative[1] = 0.0;
		derivative[2] = 0.0;

		

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
		

		error = target-torsoInitialJoints;
		integral = integral + (error * DT);
		derivative = (error - preError) / DT; 
		preError = error; 
		
		torsoVelocityCommand = kp * error + KI * integral + KD * derivative;
		time = Time::now();

		while (norm(error)>0.2){
			if(Time::now()-time>maxTorsoTrajTime){
				cout<<"Max time reached."<<endl;
				itorsoVelocity->stop();
				return true;
			}
			itorsoVelocity->velocityMove(torsoVelocityCommand.data());
			
			if (!iTorsoEncoder->getEncoders(torsoActualJoints.data())){
				cout<<"Error in reading encoders."<<endl;		
				itorsoVelocity->stop();
				return false;
			}
			
			error = target-torsoActualJoints;
			integral = integral + (error * DT);
			derivative = (error - preError) / DT; 
			preError = error;
			torsoVelocityCommand = kp * error + KI * integral + KD * derivative;
		
			Time::delay(DT);

		}
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
							int tempCxL,tempCxR;
							icartLeft->storeContext(&tempCxL);
							icartLeft->storeContext(&tempCxR);
							icartLeft->restoreContext(currentArmLeftContextID);
							icartLeft->restoreContext(currentArmRightContextID);

							icartLeft->goToPositionSync(leftArmHomePosition);
							icartRight->goToPositionSync(rightArmHomePosition);
							
							icartRight->waitMotionDone(0.1,2);
							icartLeft->waitMotionDone(0.1,2);
							
							icartLeft->restoreContext(tempCxR);
							icartLeft->restoreContext(tempCxL);
							icartLeft->deleteContext(tempCxR);
							icartLeft->deleteContext(tempCxL);
							reply.addString("Arm home position reached.");
                            return true;
						case TORSO:
							exploreTorso(torsoHomePosition);
							reply.addString("Torso home position reached.");
							return true;
						case GAZE:
							int tempCx;
							igaze->storeContext(&tempCx);
							igaze->restoreContext(currentGazeContextID);
							
							igaze->lookAtFixationPoint(gazeHomePosition);
							igaze->waitMotionDone(0.1,2);

							igaze->restoreContext(tempCx);
							igaze->deleteContext(tempCx);
							reply.addString("Gaze home position reached.");
							return true;
						default:
							reply.addString("Wrong device for home position.");
							return true;
						}
			else{
				int tempCxL,tempCxR,tempCx;
				icartLeft->storeContext(&tempCxL);
				icartLeft->storeContext(&tempCxR);
				icartLeft->restoreContext(currentArmLeftContextID);
				icartLeft->restoreContext(currentArmRightContextID);
				
				igaze->storeContext(&tempCx);
				igaze->restoreContext(currentGazeContextID);


				icartLeft->goToPositionSync(leftArmHomePosition);
				icartRight->goToPositionSync(rightArmHomePosition);
				igaze->lookAtFixationPoint(gazeHomePosition);
				exploreTorso(torsoHomePosition);
				igaze->waitMotionDone(0.1,2);
				icartRight->waitMotionDone(0.1,2);
				icartLeft->waitMotionDone(0.1,2);

				icartLeft->restoreContext(tempCxR);
				igaze->restoreContext(tempCx);
				icartLeft->restoreContext(tempCxL);
				icartLeft->deleteContext(tempCxR);
				icartLeft->deleteContext(tempCxL);
				igaze->deleteContext(tempCx);
							
				reply.addString("Ok, moving to home position.");
				return true;
			}
		case LOOK_AT:
			if(command.size()==2){
				Bottle bAsk,bReply, bGet;
				bAsk.addVocab(Vocab::encode("ask"));
				Bottle &bTempAsk=bAsk.addList().addList();
				bTempAsk.addString("name");
				bTempAsk.addString("==");
				bTempAsk.addString(command.get(1).asString());
				objectsPort.write(bAsk,bReply);
				cout<<"first"<<endl;
				if(bReply.size()==0 || bReply.get(0).asVocab()!=Vocab::encode("ack") || bReply.get(1).asList()->check("id")==false || 
					bReply.get(1).asList()->find("id").asList()->size()==0){
						reply.addVocab(Vocab::encode("nack"));
						return true;
				}
				bGet.addVocab(Vocab::encode("get"));
				Bottle &bTempGet=bGet.addList().addList();
				bTempGet.addString("id");
				bTempGet.addInt(bReply.get(1).asList()->find("id").asList()->get(0).asInt());
				objectsPort.write(bGet,bReply);
				cout<<"second"<<endl;
				if(bReply.size()==0 || bReply.get(0).asVocab()!=Vocab::encode("ack") || bReply.get(1).asList()->check("position_3d")==false ||
					bReply.get(1).asList()->find("position_3d").asList()->size()==0){
						reply.addVocab(Vocab::encode("nack"));
						return true;
				}
				cout<<"third"<<endl;
                Vector gazePosition(3);
				gazePosition[0] = bReply.get(1).asList()->find("position_3d").asList()->get(0).asDouble();
                gazePosition[1] = bReply.get(1).asList()->find("position_3d").asList()->get(1).asDouble();
                gazePosition[2] = bReply.get(1).asList()->find("position_3d").asList()->get(2).asDouble();

				int tempCx;
				igaze->storeContext(&tempCx);
				igaze->restoreContext(currentGazeContextID);
							
				igaze->lookAtFixationPoint(gazePosition);
				igaze->waitMotionDone(0.2,3);
				
				igaze->restoreContext(tempCx);
				igaze->deleteContext(tempCx);
				
				reply.addString("Gaze position reached.");
				
				return true;
			}

			else if(command.size()==4){
				Vector gazePosition(3);
				int tempCx;

				gazePosition[0] = command.get(1).asDouble();
				gazePosition[1] = command.get(2).asDouble();
				gazePosition[2] = command.get(3).asDouble();
				
				igaze->storeContext(&tempCx);
				igaze->restoreContext(currentGazeContextID);
							
				igaze->lookAtFixationPoint(gazePosition);
				igaze->waitMotionDone(0.2,3);
				
				igaze->restoreContext(tempCx);
				igaze->deleteContext(tempCx);
				reply.addString("Gaze position reached.");
				return true;
			}
			else{
				reply.addString("Wrong number of parameters for lookAt.");
				return true;
			}
		case GET:
			if(command.size()==2){
				Bottle bAsk,bReply, bGet;
				bAsk.addVocab(Vocab::encode("ask"));
				Bottle &bTempAsk=bAsk.addList().addList();
				bTempAsk.addString("name");
				bTempAsk.addString("==");
				bTempAsk.addString(command.get(1).asString());
				objectsPort.write(bAsk,bReply);
				if(bReply.size()==0 || bReply.get(0).asVocab()!=Vocab::encode("ack") || bReply.get(1).asList()->check("id")==false || 
					bReply.get(1).asList()->find("id").asList()->size()==0){
						reply.addVocab(Vocab::encode("nack"));
						return true;
				}
				bGet.addVocab(Vocab::encode("get"));
				Bottle &bTempGet=bGet.addList().addList();
				bTempGet.addString("id");
				bTempGet.addInt(bReply.get(1).asList()->find("id").asList()->get(0).asInt());
				objectsPort.write(bGet,bReply);
				if(bReply.size()==0 || bReply.get(0).asVocab()!=Vocab::encode("ack") || bReply.get(1).asList()->check("position_2d_left")==false ||
					bReply.get(1).asList()->find("position_2d_left").asList()->size()==0){
						reply.addVocab(Vocab::encode("nack"));
						return true;
				}
                Vector objPosition(4);
				objPosition[0] = bReply.get(1).asList()->find("position_2d_left").asList()->get(0).asInt();
                objPosition[1] = bReply.get(1).asList()->find("position_2d_left").asList()->get(1).asInt();
                objPosition[2] = bReply.get(1).asList()->find("position_2d_left").asList()->get(2).asInt();
                objPosition[3] = bReply.get(1).asList()->find("position_2d_left").asList()->get(3).asInt();
				reply.addVocab(VOCAB3('a','c','k'));
				Bottle &coord  = reply.addList();
				coord.addInt((int)(objPosition[0]+objPosition[2])/2);
				coord.addInt((int)(objPosition[1]+objPosition[3])/2);
				return true;
			}
			else{
				reply.addString("Wrong number of parameters for get.");
				return true;
			}
		case TRACK:
			if(command.size()==3)
				switch(command.get(1).asVocab()){
                        case ARM:
							if (command.get(2).asString() == "on"){
int tempCxL,tempCxR;
							//icartLeft->storeContext(&tempCxL);
							//icartLeft->storeContext(&tempCxR);
							//icartLeft->restoreContext(currentArmLeftContextID);
							//icartLeft->restoreContext(currentArmRightContextID);

								icartLeft->setTrackingMode(true);
								icartRight->setTrackingMode(true);
								icartRight->storeContext(&currentArmRightContextID);
								icartLeft->storeContext(&currentArmLeftContextID);
						//icartLeft->restoreContext(tempCxR);
						//	icartLeft->restoreContext(tempCxL);
						//	icartLeft->deleteContext(tempCxR);
						//	icartLeft->deleteContext(tempCxL);

								reply.addString("Arm tracking mode enabled.");

							}
							else if (command.get(2).asString() == "off"){
int tempCxL,tempCxR;
							//icartLeft->storeContext(&tempCxL);
							//icartLeft->storeContext(&tempCxR);
							//icartLeft->restoreContext(currentArmLeftContextID);
							//icartLeft->restoreContext(currentArmRightContextID);

								icartLeft->setTrackingMode(false);
								icartRight->setTrackingMode(false);

								icartRight->storeContext(&currentArmRightContextID);
								icartLeft->storeContext(&currentArmLeftContextID);

						//icartLeft->restoreContext(tempCxR);
						//	icartLeft->restoreContext(tempCxL);
						//	icartLeft->deleteContext(tempCxR);
						//	icartLeft->deleteContext(tempCxL);
								reply.addString("Arm tracking mode disabled.");
							}
							else
								reply.addString("Wrong parameter: on/off");
							return true;
						case GAZE:
							if (command.get(2).asString() == "on"){
                                int tempCx;
    							//igaze->storeContext(&tempCx);
    							//igaze->restoreContext(currentGazeContextID);

								igaze->setTrackingMode(true);

                                igaze->storeContext(&currentGazeContextID);
								//igaze->restoreContext(tempCx);
							    //igaze->deleteContext(tempCx);

								reply.addString("Gaze tracking mode enabled.");
							}
							else if (command.get(2).asString() == "off"){

								int tempCx;
    							//igaze->storeContext(&tempCx);
    							//igaze->restoreContext(currentGazeContextID);

								igaze->setTrackingMode(false);

                                igaze->storeContext(&currentGazeContextID);
								//igaze->restoreContext(tempCx);
							    //igaze->deleteContext(tempCx);

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
			if(command.size()>1)
				switch(command.get(1).asVocab()){
						case GAZE:
							if(command.size()==3){
                                // int tempCx;
    							//igaze->storeContext(&tempCx);
    							//igaze->restoreContext(currentGazeContextID);

								igaze->blockEyes(command.get(2).asDouble());
                                
                                igaze->storeContext(&currentGazeContextID);
								//igaze->restoreContext(tempCx);
							    //igaze->deleteContext(tempCx);

								reply.addString("Gaze blocking mode enabled.");
							}
							else{
								//int tempCx;
    							//igaze->storeContext(&tempCx);
    							//igaze->restoreContext(currentGazeContextID);

								igaze->blockEyes(DEFAULT_VERGENCE);
                                
                                igaze->storeContext(&currentGazeContextID);
								//igaze->restoreContext(tempCx);
							    //igaze->deleteContext(tempCx);
								
								reply.addString("Default vergence set.");
							}
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
			running = true;
			reply.addString("Ready for exploration. next or stop?");
			return true;
		case NEXT:
			if(running){
				exploreTorso(waypoints.getRow(index));
				index++;
				if (index > 2){
					running = false;
					index = 0;
					reply.addString("Waypoint reached. End of waypoints.");
					}
				else{
					reply.addString("Waypoint reached. next or stop?");
				}
			}
			else
				reply.addString("Not running.");
			return true;
		case STOP:
			index = 0;
			running = false;
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
		robotName=rf.check("robot",Value("icub")).asString().c_str();
		OPCName=rf.check("OPC",Value("memory")).asString().c_str();

		period=rf.check("period",Value(0.2)).asDouble();
		kp=rf.check("kp",Value(KP)).asDouble();
		maxTorsoTrajTime=rf.check("torsoTime",Value(MAX_TORSO_TRAJ_TIME)).asDouble();
        maxTorsoVelocity=rf.check("maxTorsoVelocity",Value(MAX_TORSO_VELOCITY)).asDouble();
		maxArmTrajTime=rf.check("armTime",Value(MAX_ARM_TRAJ_TIME)).asDouble();

		handlerPort.open(("/"+moduleName+"/rpc:i").c_str());
        attach(handlerPort);

		objectsPort.open(("/"+moduleName+"/OPC:io").c_str());
		if(!objectsPort.addOutput(("/"+OPCName+"/rpc").c_str())){
			cout<<"Error connecting to OPC client!"<<endl;
			return false;
		}


		Property optionGaze;
		optionGaze.put("device","gazecontrollerclient");
		optionGaze.put("remote","/iKinGazeCtrl");
		optionGaze.put("local",("/"+moduleName+"/gaze").c_str());
		if(!clientGazeCtrl.open(optionGaze)){
			cout<<"Error opening gaze client!"<<endl;
			return false;
		}
		clientGazeCtrl.view(igaze);
		igaze->restoreContext(0);
		igaze->storeContext(&startupGazeContextID);
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
		icartLeft->restoreContext(0);
		icartLeft->storeContext(&startupArmLeftContextID);

		leftArmHomePosition.push_back(LEFT_ARM_HOME_POS_X);
		leftArmHomePosition.push_back(LEFT_ARM_HOME_POS_Y);
		leftArmHomePosition.push_back(LEFT_ARM_HOME_POS_Z);
		
		icartLeft->setTrajTime(maxArmTrajTime);
		
		icartLeft->storeContext(&currentArmLeftContextID);

		Property rightArmOption;
		rightArmOption.put("device","cartesiancontrollerclient");
		rightArmOption.put("remote",("/"+robotName+"/cartesianController/right_arm").c_str());
		rightArmOption.put("local",("/"+moduleName+"/right_arm").c_str());

		if(!clientArmRight.open(rightArmOption)){
			cout<<"Error opening right arm client!"<<endl;
			return false;
		}

		clientArmRight.view(icartRight);
		icartRight->restoreContext(0);
		icartRight->storeContext(&startupArmRightContextID);

		rightArmHomePosition.push_back(RIGHT_ARM_HOME_POS_X);
		rightArmHomePosition.push_back(RIGHT_ARM_HOME_POS_Y);
		rightArmHomePosition.push_back(RIGHT_ARM_HOME_POS_Z);
				
		icartRight->setTrajTime(maxArmTrajTime);

		icartRight->storeContext(&currentArmRightContextID);

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


		waypoints.resize(3,3);
		waypoints(0,0) = 0.0; waypoints(0,1) = -15.0; waypoints(0,2) = 10.0; 
		waypoints(1,0) = 0.0; waypoints(1,1) = 15.0; waypoints(1,2) = 10.0; 
		waypoints(2,0) = 0.0; waypoints(2,1) = 0.0; waypoints(2,2) = 20.0; 
		
		index = 0;
		running = false;


		/*Bottle bAdd, bReply;
		bAdd.addVocab(Vocab::encode("add"));
		Bottle &bTempAdd=bAdd.addList();

		Bottle &bEntity=bTempAdd.addList();
		bEntity.addString("entity"); bEntity.addString("action");

		Bottle &bName=bTempAdd.addList();
		bName.addString("name"); bName.addString("ball");

		Bottle &bX= bTempAdd.addList();
		bX.addString("position_3d"); 
		Bottle &coord = bX.addList();	
		coord.addDouble(-0.05);
		coord.addDouble(0.0);
		coord.addDouble(0.2);

		objectsPort.write(bAdd,bReply);
		cout<<bReply.get(0).asVocab()<<endl;
		*/

        return true;
    }

    bool interruptModule()
    {

        cout<<"Interrupt caught!"<<endl;
		cout<<endl;
		handlerPort.interrupt();
		objectsPort.interrupt();
        return true;
    }

    bool close()
    {
        handlerPort.close();
		objectsPort.close();

		igaze->stopControl();
		igaze->restoreContext(startupGazeContextID);
		igaze->deleteContext(startupGazeContextID);
		igaze->deleteContext(currentGazeContextID);

		if (clientGazeCtrl.isValid())
			clientGazeCtrl.close();

		icartLeft->stopControl();
		icartLeft->restoreContext(startupArmLeftContextID);
		icartLeft->deleteContext(startupArmLeftContextID);
		icartLeft->deleteContext(currentArmLeftContextID);

		if (clientArmLeft.isValid())
			clientArmLeft.close();

		icartRight->stopControl();
		icartRight->restoreContext(startupArmRightContextID);
		icartRight->deleteContext(startupArmRightContextID);
		icartRight->deleteContext(currentArmRightContextID);

		if (clientArmLeft.isValid())
			clientArmLeft.close();

		itorsoVelocity->stop();
		
		if (clientTorso.isValid())
		clientTorso.close();
		
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


