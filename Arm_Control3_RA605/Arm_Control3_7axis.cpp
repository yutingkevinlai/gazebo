//AR07 robot 路徑規劃由virtualbox內winxp傳過來

#include <kdl/frames.hpp>
//#include <gazebo-2.0/gazebo/math/Pose.hh>
#include <kdl/jntarray.hpp>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Matrix4.hh>
#include <gazebo/math/Pose.hh>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Matrix4.hh>
#include <stdio.h>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <ostream>
#include <fstream>
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/vector3d.pb.h"
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <stdio.h>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <ostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include <pose_estimation_result.pb.h>

#define COUT_PREFIX "\033[1;34m" << "[ArmControl] " << "\033[0m"
#define CERR_PREFIX "\033[1;31m" << "[ArmControl] " << "\033[0m"

using namespace gazebo;

class SocketClient
{
private:
    int socket1;//建立socket
    //WSADATA wsaData;
    char* ip;//server的位址
    struct sockaddr_in server;
    socklen_t len ;
    char sentdata[80];

public:

 SocketClient()
    {
    	memset(sentdata, '\0',80);
    	//WSAStartup(MAKEWORD(2,1),&wsaData);
    	len = sizeof(server);
    	//strcpy(ip,"127.0.0.1");
    	server.sin_family = AF_INET;
    	server.sin_port = htons(12345);  //設定port
    	std::string IP("127.0.0.1");
    	server.sin_addr.s_addr = inet_addr(IP.c_str()/*ip*/);
    	socket1 = socket(AF_INET,SOCK_DGRAM,0);//設為UDP
    }
    void Send(char * m_data)
    {
    	std::string data_string(m_data);
    	//strcpy(sentdata,m_data);
    	sendto(socket1,data_string.c_str(),sizeof(data_string.c_str())*strlen(data_string.c_str()),0,(struct sockaddr*)&server,len);
    	////////////////////////////////////////↑這裡要乘上字串的長度,否則 sizeof(sentdata)就只是這指標的大小而已 :4
    }
    void CloseSocket()
    {
    	close(socket1);
    }
};
class SocketServer
{
public:
    char* Receive()//這是block function
    {
        std::cout<<recvfrom(socket1,buffer,sizeof(buffer),0,(struct sockaddr*)&client,&len) ;
        return buffer;//雖然是宣告靜態陣列 但回傳是指標 ,要用char*收 char[512]不行
    }
    void CloseSocket()
    {
        close(socket1);
    }
    ~SocketServer()
    {
    	std::cout<<"~SocketServer()"<<std::endl;
    	CloseSocket();
    }
    SocketServer()
    {
        //WSAStartup(MAKEWORD(2,1),&wsaData);//初始windows socket application,並設定版本MAKEWORD(2,1)
        len = sizeof(client);
        local.sin_family = AF_INET;
        local.sin_port = htons(12345);  //設定port
        //std::string IP("140.113.150.69");
        local.sin_addr.s_addr = INADDR_ANY;//inet_addr(IP.c_str());
        socket1 = socket(AF_INET,SOCK_DGRAM,0);//設定為UDP
        memset(buffer, '\0',80);
        bind(socket1,(struct sockaddr*)&local,sizeof(local));

    }
private:
    int socket1;//建立socket
    //WSADATA wsaData;
    struct sockaddr_in local;
    struct sockaddr_in client;
    socklen_t len;
    char buffer[80];
};





class Arm_Control3_7axis: public ModelPlugin
{
public:
~Arm_Control3_7axis()
{

}

public:
void Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
	this->model = _parent;
	world = model->GetWorld();
	//開一個thread,監聽鍵盤輸入
	_Thread = boost::thread(&Arm_Control3_7axis::Keyboard_Input, this);
	//初始化與evaluation通信
	//init
	joint_v.push_back(model->GetJoint("joint1"));
	joint_v.push_back(model->GetJoint("joint2"));
	joint_v.push_back(model->GetJoint("joint3"));
	joint_v.push_back(model->GetJoint("joint4"));
	joint_v.push_back(model->GetJoint("joint5"));
	joint_v.push_back(model->GetJoint("joint6"));

	std::vector<float> joint_sapce_point(6,0.0);
	list_page=0;
	joint_angle_list[0].push_back(joint_sapce_point);
	joint_angle_list[0].push_back(joint_sapce_point);
	joint_angle_list[1].push_back(joint_sapce_point);
	joint_angle_list[1].push_back(joint_sapce_point);
	std::cerr<<"Load() "<<joint_angle_list[list_page].size()<<std::endl;
	user_input=0;
	generatePathDone=false;
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&Arm_Control3_7axis::cyclic_task, this, _1));
}
public:
void Keyboard_Input()
{
	std::cout << "waiting for connection...";
	while (1)
	{


		SocketServer server;
		char *receivedata;
		receivedata=server.Receive();
		char command;
		float value[6];
		std::cerr<<"Receive Data"<<std::endl;
		sscanf(receivedata,"%c %f %f %f %f %f %f",&command,&value[0],&value[1],&value[2],&value[3],&value[4],&value[5]);
		printf("%c %f %f %f %f %f %f\n",command,value[0],value[1],value[2],value[3],value[4],value[5]);
		if(command=='j')
			setJointSpacePose(value[0],value[1],value[2],value[3],value[4],value[5]);
		else if(command=='c')
			setCartesianSpacePose(value[0],value[1],value[2],value[3],value[4],value[5]);



//		if (std::cin.get() == '\n')    //按下enter,停止目前動作
//		{
//
//			std::cin>>user_input;
//
//			if(user_input==5)
//			{
//				std::cerr<<"actual pose"<<std::endl;
//				std::cerr<<model->GetLink("link6")->GetWorldPose()<<std::endl;
//				std::cout<<"KDL_FK"<<std::endl;
//				KDL::Chain chain;
//
//				chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.03,0.0,0.373))));
//				chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.33))));
//				chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.057))));
//				chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.4,0.0,0))));
//				chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0,0,0))));
//				chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0,0.0))));
//
//				KDL::ChainFkSolverPos_recursive fksolver(chain);
//				KDL::JntArray jointpositions(6) ;
//				jointpositions.data(0,0)=joint_v[0]->GetAngle(0).Radian();
//				jointpositions.data(1,0)=joint_v[1]->GetAngle(0).Radian();
//				jointpositions.data(2,0)=joint_v[2]->GetAngle(0).Radian();
//				jointpositions.data(3,0)=joint_v[3]->GetAngle(0).Radian();
//				jointpositions.data(4,0)=joint_v[4]->GetAngle(0).Radian();
//				jointpositions.data(5,0)=joint_v[5]->GetAngle(0).Radian();
//				KDL::Frame cartpos;
//				fksolver.JntToCart(jointpositions,cartpos);
//				std::cout<<cartpos<<std::endl;
//			}
//			if(user_input==4)
//				getJointAngle();
//			if(user_input==3)
//			{
//				std::cerr<<"generate path"<<std::endl;
//				setJointSpacePose(0.5,0.5,0.5,0.5,0.5,0.5);
//			}
//			if(user_input==2)
//				setCartesianSpacePose(0.364826,0.191546,0.3479,1.95978,1.30557,1.53203);
//		}
	}
}
void setJointSpacePose(float j1,float j2,float j3,float j4,float j5,float j6)
{
	_generateJointAngleList(j1,j2,j3,j4,j5,j6);

	//point to new page
	if(list_page == 0)
		list_page = 1;
	else
		list_page = 0;

}
void setCartesianSpacePose(float x,float y,float z,float a,float b,float c)
{
	float ik_solution[6]={0};
	_kDL_IK(x,y,z,a,b,c,ik_solution);

	_generateJointAngleList(ik_solution[0],ik_solution[1],ik_solution[2],ik_solution[3],ik_solution[4],ik_solution[5]);

	//point to new page
	if(list_page==0)
		list_page=1;
	else
		list_page=0;
}

void getJointAngle()
{
	//std::cerr<<"====================================="<<std::endl;
	//for(int i=0;i<6;i++)
	//	std::cerr<<theta[i]<<"\t[]";
	//std::cerr<<"====================================="<<std::endl;
}
void _kDL_IK(float x,float y,float z,float a,float b,float c,float *ik_solution)
{
    KDL::Chain chain;
    KDL::JntArray q_init,q_sol(6);
    Eigen::Matrix<double,6,1> L;
    KDL::Frame cartesian_pose;
    //convert xyzabc to kdl frame
    cartesian_pose=KDL::Frame(KDL::Rotation::RPY(a,b,c),KDL::Vector(x,y,z));
    //build kdl chain
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.03,0.0,0.373))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.33))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.057))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.4,0.0,0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0,0,0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0,0.0))));
    //initialize
    q_init=KDL::JntArray(6);
    q_init.data.setRandom();
    q_init.data *= M_PI;
    L(0)=1;
    L(1)=1;
    L(2)=1;
    L(3)=0.01;
    L(4)=0.01;
    L(5)=0.01;
	KDL::ChainIkSolverPos_LMA solver(chain,L);
	solver.CartToJnt(q_init,cartesian_pose,q_sol);

	ik_solution[0]=q_sol.data(0,0);
	ik_solution[1]=q_sol.data(1,0);
	ik_solution[2]=q_sol.data(2,0);
	ik_solution[3]=q_sol.data(3,0);
	ik_solution[4]=q_sol.data(4,0);
	ik_solution[5]=q_sol.data(5,0);

	std::cerr<<"_kDL_IK solution"<<std::endl;
	for(int i=0;i<6;i++)
		std::cerr<<ik_solution[i]<<"\t";
	std::cerr<<std::endl;
}
void _generateJointAngleList(float j1,float j2,float j3,float j4,float j5,float j6)
{
	int write_list_page;
	if(list_page==0)
		write_list_page=1;
	else
		write_list_page=0;
	std::cerr<<"_generateJointAngleList"<<std::endl;
	joint_angle_list[write_list_page].clear();
	float joint_sapce_point_end[6]={j1,j2,j3,j4,j5,j6};
	float joint_sapce_point_start[6];
	for(int i=0;i<6;i++)//get current joint position
		joint_sapce_point_start[i]=0;

	float joint_step[6];
	float total_step=200;
	for(int i=0;i<6;i++)
	{
		joint_step[i]=(joint_sapce_point_end[i]- joint_sapce_point_start[i])/total_step;
		std::cerr<<joint_sapce_point_start[i]<<"\t"<<joint_sapce_point_end[i]<<"\t"<<joint_step[i]<<std::endl;
	}
	std::vector<float> joint_sapce_point(6,0);//first joint space point should be joint_sapce_point_start
	for(int i=0;i<6;i++)
		joint_sapce_point[i]=joint_sapce_point_start[i];
	joint_angle_list[write_list_page].push_back(joint_sapce_point);

	for(int i=1;i<total_step;i++)
	{
		std::vector<float> joint_sapce_point(6,0);
		for(int j=0;j<6;j++)
		{
			joint_sapce_point[j]=joint_angle_list[write_list_page][i-1][j]+joint_step[j];
			std::cerr<<joint_sapce_point[j]<<",";
		}
		std::cerr<<std::endl;
		joint_angle_list[write_list_page].push_back(joint_sapce_point);
	}
	std::reverse(joint_angle_list[write_list_page].begin(),joint_angle_list[write_list_page].end());
	std::cerr<<"_generateJointAngleList...done  "<<joint_angle_list[write_list_page].size()<<std::endl;
}
public:
void cyclic_task(const common::UpdateInfo & /*_info*/)
{
	//if(user_input==0||user_input==5||user_input==2)
	//{
	if(joint_angle_list[list_page].size()>2)
	{
		std::cerr << joint_angle_list[list_page].size() << std::endl;
		std::vector<float> joint_sapce_point = joint_angle_list[list_page].back();
		joint_angle_list[list_page].pop_back();

		for(int i=0 ; i<6 ; i++)
			std::cerr << joint_sapce_point[i] << "\t";

		std::cerr<<std::endl;

		//shm_data->w_pos[0] = -4229 + (0) *80*131072/360;
		joint_v[0]->SetPosition(0,double(joint_sapce_point[0]/**57.2957*/));
		joint_v[1]->SetPosition(0,double(joint_sapce_point[1]/**57.2957*/));
		joint_v[2]->SetPosition(0,double(joint_sapce_point[2]/**57.2957*/));
		joint_v[3]->SetPosition(0,double(joint_sapce_point[3]/**57.2957*/));
		joint_v[4]->SetPosition(0,double(joint_sapce_point[4]/**57.2957*/));
		joint_v[5]->SetPosition(0,double(joint_sapce_point[5]/**57.2957*/));

		// SetAngle 在gazebo7就不能用了，必需換成 SetPosition //

		//joint_v[0]->SetAngle(0,math::Angle(joint_sapce_point[0]/**57.2957*/));
		//joint_v[1]->SetAngle(0,math::Angle(joint_sapce_point[1]/**57.2957*/));
		//joint_v[2]->SetAngle(0,math::Angle(joint_sapce_point[2]/**57.2957*/));
		//joint_v[3]->SetAngle(0,math::Angle(joint_sapce_point[3]/**57.2957*/));
		//joint_v[4]->SetAngle(0,math::Angle(joint_sapce_point[4]/**57.2957*/));
		//joint_v[5]->SetAngle(0,math::Angle(joint_sapce_point[5]/**57.2957*/));

		common::Time::MSleep(10);
	}
	else
	{
		//std::cerr<<joint_angle_list[list_page].size()<<std::endl;
		std::vector<float> joint_sapce_point=joint_angle_list[list_page][1];

		joint_v[0]->SetPosition(0,double(joint_sapce_point[0]/**57.2957*/));
		joint_v[1]->SetPosition(0,double(joint_sapce_point[1]/**57.2957*/));
		joint_v[2]->SetPosition(0,double(joint_sapce_point[2]/**57.2957*/));
		joint_v[3]->SetPosition(0,double(joint_sapce_point[3]/**57.2957*/));
		joint_v[4]->SetPosition(0,double(joint_sapce_point[4]/**57.2957*/));
		joint_v[5]->SetPosition(0,double(joint_sapce_point[5]/**57.2957*/));

		// SetAngle 在gazebo7就不能用了，必需換成 SetPosition //

		//joint_v[0]->SetAngle(0,math::Angle(joint_sapce_point[0]/**57.2957*/));
		//joint_v[1]->SetAngle(0,math::Angle(joint_sapce_point[1]/**57.2957*/));
		//joint_v[2]->SetAngle(0,math::Angle(joint_sapce_point[2]/**57.2957*/));
		//joint_v[3]->SetAngle(0,math::Angle(joint_sapce_point[3]/**57.2957*/));
		//joint_v[4]->SetAngle(0,math::Angle(joint_sapce_point[4]/**57.2957*/));
		//joint_v[5]->SetAngle(0,math::Angle(joint_sapce_point[5]/**57.2957*/));

		for(int i=0;i<6;i++)
			std::cerr<<joint_sapce_point[i]<<"\t";
		std::cerr<<std::endl;
		common::Time::MSleep(10);
		//common::Time::Sleep(3);
		//getJointAngle();
	}
	//}

	//防撞防撞check

//	for(int i=0;i<6;i++)
//	{
//		theta[i]=joint_v[i]->GetAngle(0).Radian();
//	}
}


public:

private:
physics::ModelPtr model;
physics::WorldPtr world;
std::vector<physics::JointPtr> joint_v;
event::ConnectionPtr updateConnection;
boost::thread _Thread;
float theta[6];
float input_theta[6];
int user_input;
int list_page;
std::vector<std::vector<float> > joint_angle_list[2];
bool generatePathDone;
};

GZ_REGISTER_MODEL_PLUGIN(Arm_Control3_7axis);
