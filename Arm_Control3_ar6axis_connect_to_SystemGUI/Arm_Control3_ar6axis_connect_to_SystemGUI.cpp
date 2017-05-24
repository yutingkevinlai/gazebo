//AR07 robot 路徑規劃由virtualbox內winxp傳過來
#include <stdio.h>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <ostream>
#include <boost/bind.hpp>
#include <gazebo/math/Pose.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Matrix4.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/vector3d.pb.h"
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
#include <kdl/jntarray.hpp>
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

typedef std::vector<gazebo::math::Angle> Six_Ang;
typedef std::vector<Six_Ang> Ang_List_V;
std::ostream& operator<<(std::ostream &out, Six_Ang &Six_Ang_param)
{
	out << Six_Ang_param[0] << " " << Six_Ang_param[1] << " "
			<< Six_Ang_param[2] << " " << Six_Ang_param[3] << " "
			<< Six_Ang_param[4] << " " << Six_Ang_param[5];
	return out;
}

using namespace gazebo;
using namespace std;

class cout_debug
{
public:
cout_debug(){}
};
void operator<<(cout_debug out,std::string someword)
{//DEBUG開關在這
	//cout<<"\033[5;31m"<<someword<<endl;
}
cout_debug coutdebug;
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

//TODO: 這功能好像沒必要做一個class = =
class ListAllModel
{
private:
int num_model_on_field;
physics::WorldPtr world;

public:
ListAllModel()
{
}
public:
ListAllModel(physics::ModelPtr param_model)
{
	world = param_model->GetWorld();
	num_model_on_field = 0;
}
public:
void ForceUpdate()    //強迫觸發
{
	num_model_on_field = 0;
}
public:
void Update()    //這式自動觸發的 一旦在場上的的modell數量有變化
{

	if (world->GetModelCount() != num_model_on_field)    //在場上的的modell數量有變化
	{
		num_model_on_field = world->GetModelCount();
		std::cout << "\n\nNow There are " << num_model_on_field
				<< " models on the field" << std::endl;
		for (int i = 0; i < num_model_on_field; i++)    //列出所有在場上的model名稱
		{
			std::cout << i << "\t" << world->GetModel(i)->GetName() << "\t("
					<< world->GetModel(i)->GetWorldPose().pos << ")"
					<< std::endl;
		}
	}
}
};
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class MoveTo
{
private:

double velocityOfjoint[6];
int check_joint[12];
physics::ModelPtr model;
physics::WorldPtr world;
physics::Link_V link;
physics::Joint_V joint;
Six_Ang currentang;
char axis[6];
public:
Six_Ang ang;
public:
physics::Link_V axis_aligner;
MoveTo()
{
}
public:
int HasReached()
{
	//check_joint[1]如果是1,表示目前第1軸位置比目標位置大
	//check_joint[0]如果是1,表示目前第1軸位置比目標位置小
	// check_joint[1]與check_joint[0]同時等於1,表示目前第1軸位置比目標位置大,同時,第1軸位置比目標位置小
	//也就是說第1軸已達到目標位置
	if (check_joint[1] && check_joint[0] && check_joint[3] && check_joint[2]
			&& check_joint[5] && check_joint[4] && check_joint[7]
			&& check_joint[6] && check_joint[9] && check_joint[8]
			&& check_joint[11] && check_joint[10])
		return 1;
	else
		return 0;
}
public:
void ResetReached()
{
	for (int i = 0; i < 12; i++)
	{
		check_joint[i] = 0;
	}
}
public:
void SetReached()
{
	for (int i = 0; i < 12; i++)
	{
		check_joint[i] = 1;
	}
}

public:
MoveTo(physics::ModelPtr param_model)    //TODO: constructor要做詳細一點,model的各種參數 就直接給constructor當作參數
{
	world = param_model->GetWorld();
	model = param_model;
	//將"reached"這狀態歸0 TODO:用這函式取代啊,ResetReached()
	for (int i = 0; i < 6; i++)
	{
		check_joint[2 * i] = 0;
		check_joint[2 * i + 1] = 0;
	}
	//各軸角速度 亂設1
	velocityOfjoint[0] = 1;
	velocityOfjoint[1] = 1;
	velocityOfjoint[2] = 1;
	velocityOfjoint[3] = 1;
	velocityOfjoint[4] = 1;
	velocityOfjoint[5] = 1;
	//各軸目前角度
	currentang = Six_Ang(6, 0);
	ang.push_back((math::Angle) 0);
	ang.push_back((math::Angle) 0);
	ang.push_back((math::Angle) 0);
	ang.push_back((math::Angle) 0);
	ang.push_back((math::Angle) 0);
	ang.push_back((math::Angle) 0);
	//各軸旋轉方向
    axis[0]='z';
    axis[1]='x';
    axis[2]='x';
    axis[3]='y';
    axis[4]='x';
    axis[5]='y';
	//將各link存進vector
	link.push_back(model->GetLink("link1"));
	link.push_back(model->GetLink("link2"));
	link.push_back(model->GetLink("link3"));
	link.push_back(model->GetLink("link4"));
	link.push_back(model->GetLink("link5"));
	link.push_back(model->GetLink("link6"));
	//將各joint存進vector
	joint.push_back(model->GetJoint("joint1"));
	joint.push_back(model->GetJoint("joint2"));
	joint.push_back(model->GetJoint("joint3"));
	joint.push_back(model->GetJoint("joint4"));
	joint.push_back(model->GetJoint("joint5"));
	joint.push_back(model->GetJoint("joint6"));
	//將各axis_aligner存進vector
	//因為在設定角速度時,方向對於世界座標的,於是需要axis_aligner,將角速度的方向改用各軸的local座標表示
	axis_aligner.push_back(model->GetLink("base"));
	axis_aligner.push_back(model->GetLink("link1"));
	axis_aligner.push_back(model->GetLink("link2"));
	axis_aligner.push_back(model->GetLink("link3"));
	axis_aligner.push_back(model->GetLink("link4"));
	axis_aligner.push_back(model->GetLink("link5"));
}
//只是設定目的地而已,還不會動,"reached"這狀態是0的話,才會動
public:
void SetDestination(Six_Ang param_ang)
{
	ang.assign(param_ang.begin(), param_ang.end());
}
//回傳目前各軸角度
public:
Six_Ang GetJointAngs()
{
	return ang;
}
//只設定特定軸的位置
//TODO:應該將"想改變的軸"&"軸位置"當作這函數的參數,比較方便使用
public:
void SetSpecificJoint()
{
	std::cout << "\nangle of each joint" << std::endl;
	for (int i = 0; i < 6; i++)
		std::cout << i + 1 << ", " << ang[i] << std::endl;

	int n;
	float input_angle;
	std::cout << "input joint angle" << std::endl;
	scanf("%d %f", &n, &input_angle);
	ang[n - 1] = (math::Angle) input_angle;
	std::cin.ignore();
	ResetReached();
}
//回到初始位置

//一些固定的點
public:void Set2ndPosition()
{
	ang[0] = (math::Angle)(0.55);
	ang[1] = (math::Angle)(-1.1);
	ang[2] = (math::Angle)(-0.4);
	ang[3] = (math::Angle)(0);
	ang[4] = (math::Angle)(1.5);
	ang[5] = (math::Angle)(0);
}
public:void SetGoHome()
{
	ang[0] = (math::Angle) (0);
	ang[1] = (math::Angle) (0);
	ang[2] = (math::Angle) (0);
	ang[3] = (math::Angle) (0);
	ang[4] = (math::Angle) (0);
	ang[5] = (math::Angle) (0);
}













//設定各軸速度,也可以不用設,預設好都是1
public:
void SetVelocityOfJoint(double param_velocityOfjoint[6])
{
	for (int i = 0; i < 6; i++)
	{
		velocityOfjoint[i] = param_velocityOfjoint[i];
	}
}
//就定位後就固定住,好像只有Update()會用到
public:
void SetPositionDirectly()
{
	for (int i = 0; i < 6; i++)
	{
		link[i]->SetAngularVel(math::Vector3(0, 0, 0));
		joint[i]->SetHighStop(0, ang[i] + math::Angle(0.001));
		joint[i]->SetLowStop(0, ang[i] + math::Angle(-0.001));
	}
}
//gazebo每個update迴圈都會呼叫這 update()
//讓model動起來的重要部分
//忘了type=0是要幹嘛的
public:
void Update(int type = 0)
{
//	cout<<check_joint[1]<<check_joint[0]<<" "
//			<<check_joint[3]<<check_joint[2]<<" "
//			<<check_joint[5]<<check_joint[4]<<" "
//			<<check_joint[7]<<check_joint[6]<<" "
//			<<check_joint[9]<<check_joint[8]<<" "
//			<<check_joint[11]<<check_joint[10]<<endl;

		if (check_joint[1] && check_joint[0] && check_joint[3] && check_joint[2]
			&& check_joint[5] && check_joint[4] && check_joint[7]
			&& check_joint[6] && check_joint[9] && check_joint[8]
			&& check_joint[11] && check_joint[10])
	{
		SetPositionDirectly();    //就定位後就固定住
	} else    //各joint還沒就定位,就繼續跑
	{
		for (int i = 0; i < 6; i++)
		{
			if (type == 0)
				SetEachLinkPosition(i);
		}
	}
}
//控制各軸到達指定位置
private:
void SetEachLinkPosition(int nth)
{
	if (!(check_joint[nth * 2 + 1] && check_joint[nth * 2]))
	//if(check_joint[((nth*2-1)<0?12:(nth*2-1))]&&check_joint[((nth*2-2)<0?12:(nth*2-2))]&&!(check_joint[nth*2+1]&&check_joint[nth*2]))
	{
		//cout<<"正在移動"<<nth+1<<"軸 "<<axis[nth]<<"方向"<<endl;
		//限制軸的活定範圍
		//好像沒啥實際功用@@?
		joint[nth]->SetHighStop(0, math::Angle(4));
		joint[nth]->SetLowStop(0, math::Angle(-4));
		if (((joint[nth]->GetAngle(0) - ang[nth])
				* (joint[nth]->GetAngle(0) - ang[nth])) < 0.00001)
		{
			//目前軸位置與目標位置誤差小於0.00001 ,就判定為到達
			check_joint[nth * 2 + 1] = 1;
			check_joint[nth * 2] = 1;
		}
		// 目前軸位置比目標位置小,就繼續加
		else if (joint[nth]->GetAngle(0) <= ang[nth])
		{
			//if(nth==1)std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" > "<<joint[nth]->GetAngle(0)<<" "<<velocityOfjoint[nth]<<std::endl;
			//軸有3種可能的軸向
			if (axis[nth] == 'x')
			{
				link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(velocityOfjoint[nth], 0, 0)));
			}
			if (axis[nth] == 'y')
				link[nth]->SetAngularVel(
						axis_aligner[nth]->GetWorldPose().rot.RotateVector(
								math::Vector3(0, velocityOfjoint[nth], 0)));
			if (axis[nth] == 'z')
				link[nth]->SetAngularVel(
						axis_aligner[nth]->GetWorldPose().rot.RotateVector(
								math::Vector3(0, 0, velocityOfjoint[nth])));
			check_joint[nth * 2 + 1] = 1;
		}
		// 目前軸位置比目標位置大,就繼續減
		else if (joint[nth]->GetAngle(0) > ang[nth])
		{
			//if(nth==1)std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" < "<<joint[nth]->GetAngle(0)<<" "<<velocityOfjoint[nth]<<std::endl;
			//軸有3種可能的軸向
			if (axis[nth] == 'x')
		{
				link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(-velocityOfjoint[nth], 0, 0)));
			//cout<<axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(velocityOfjoint[nth], 0, 0))<<endl;
		}
			if (axis[nth] == 'y')
			{
				link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0, -velocityOfjoint[nth], 0)));
				//cout<<	axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0, -velocityOfjoint[nth], 0))<<endl;

			}
			if (axis[nth] == 'z')
				link[nth]->SetAngularVel(
						axis_aligner[nth]->GetWorldPose().rot.RotateVector(
								math::Vector3(0, 0, -velocityOfjoint[nth])));
			check_joint[nth * 2] = 1;
		}
	} else            //以下是處理好的
	{
		joint[nth]->SetHighStop(0, ang[nth] + math::Angle(0.001));
		joint[nth]->SetLowStop(0, ang[nth] + math::Angle(-0.001));
	}
}
//按照順序來,第一軸到達後,第二軸才能動,
// 第二軸到達後,第三軸才能動,依序下去...
//這功能好像用不太到了
//TODO:可以砍了
private:
void SetEachLinkPositionOrder(int nth)
{
	//std::cout<<" SetEachLinkPositionOrder"<<std::endl;
	//printf("%d %d %d %d %d %d %d %d %d %d %d %d \n",check_joint[1],check_joint[0],check_joint[3],check_joint[2],check_joint[5],check_joint[4],check_joint[7],check_joint[6],check_joint[9],check_joint[8],check_joint[11],check_joint[10]);
	//printf("%f %f %f %f %f %f \n",velocityOfjoint[0],velocityOfjoint[1],velocityOfjoint[2],velocityOfjoint[3],velocityOfjoint[4],velocityOfjoint[5]);

	if (check_joint[((nth * 2 - 1) < 0 ? 12 : (nth * 2 - 1))]
			&& check_joint[((nth * 2 - 2) < 0 ? 12 : (nth * 2 - 2))]
			&& !(check_joint[nth * 2 + 1] && check_joint[nth * 2]))
	{
		for (int i = 0; i < 6; i++)
		{
			if (i != nth)
				link[i]->SetAngularVel(math::Vector3(0, 0, 0));
		}
		//以下是處理好的
		for (int i = 0; i < nth; i++)
		{
			joint[i]->SetHighStop(0, ang[i] + math::Angle(0.001));
			joint[i]->SetLowStop(0, ang[i] + math::Angle(-0.001));
		}
		//正要處理
		joint[nth]->SetHighStop(0, math::Angle(4));
		joint[nth]->SetLowStop(0, math::Angle(-4));
		if (((joint[nth]->GetAngle(0) - ang[nth])
				* (joint[nth]->GetAngle(0) - ang[nth])) < 0.00001)
		{
			std::cout << "Order : " << axis[nth] << nth
					<< " on the position! \n";
			check_joint[nth * 2 + 1] = 1;
			check_joint[nth * 2] = 1;
		} else if (joint[nth]->GetAngle(0) <= ang[nth])
		{
			//std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" > "<<joint[nth]->GetAngle(0)<<std::endl;
			if (axis[nth] == 'y')
				link[nth]->SetAngularVel(
						axis_aligner[nth]->GetWorldPose().rot.RotateVector(
								math::Vector3(0, velocityOfjoint[nth], 0)));
			if (axis[nth] == 'z')
				link[nth]->SetAngularVel(
						axis_aligner[nth]->GetWorldPose().rot.RotateVector(
								math::Vector3(0, 0, velocityOfjoint[nth])));
			check_joint[nth * 2 + 1] = 1;
		} else if (joint[nth]->GetAngle(0) > ang[nth])
		{
			//std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" < "<<joint[nth]->GetAngle(0)<<std::endl;
			if (axis[nth] == 'y')
				link[nth]->SetAngularVel(
						axis_aligner[nth]->GetWorldPose().rot.RotateVector(
								math::Vector3(0, -velocityOfjoint[nth], 0)));
			if (axis[nth] == 'z')
				link[nth]->SetAngularVel(
						axis_aligner[nth]->GetWorldPose().rot.RotateVector(
								math::Vector3(0, 0, -velocityOfjoint[nth])));
			check_joint[nth * 2] = 1;
		}
	}
}
//印出末端(T6)?tool?位置
public:
void PrintTipPose_KDL()
{
    std::cout<<"KDL_FK"<<std::endl;
    KDL::Chain chain;
    using namespace KDL;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0,0.075,0.345))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.27))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0,0.106,0.09))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0,0.189,0))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0,0,0))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0,0.145,0))));
    KDL::ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    KDL::JntArray jointpositions(6) ;
    jointpositions.data(0,0)=ang[0].Radian();
    jointpositions.data(1,0)=ang[1].Radian();
    jointpositions.data(2,0)=ang[2].Radian();
    jointpositions.data(3,0)=ang[3].Radian();
    jointpositions.data(4,0)=ang[4].Radian();
    jointpositions.data(5,0)=ang[5].Radian();
    KDL::Frame cartpos;
    fksolver.JntToCart(jointpositions,cartpos);
    cartpos.p.data[1]=cartpos.p.data[1]-0.5;
    std::cout<<"pose :\n"<<cartpos<<std::endl;
}
//回傳 末端(T6)?tool?位置
public:
math::Pose GetTipPose()
{
	math::Pose plampose = model->GetLink("gripper::plam")->GetWorldPose();
	math::Pose tippose = plampose
			+ math::Pose(plampose.rot.RotateVector(math::Vector3(0, 0, -0.05)),
					math::Quaternion(0, 0, 0));
	return tippose;
}
};
typedef boost::shared_ptr<MoveTo> MoveToPtr;
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
class SocketClient
{
private:
    int socket1;//建立socket
    //WSADATA wsaData;
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
    	std::string IP("192.168.56.101");
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
        cout<<recvfrom(socket1,buffer,sizeof(buffer),0,(struct sockaddr*)&client,&len) ;
        return buffer;//雖然是宣告靜態陣列 但回傳是指標 ,要用char*收 char[512]不行
    }
    void CloseSocket()
    {
        close(socket1);
    }
    ~SocketServer()
    {
    	cout<<"~SocketServer()"<<endl;
    	CloseSocket();
    }
    SocketServer()
    {
        //WSAStartup(MAKEWORD(2,1),&wsaData);//初始windows socket application,並設定版本MAKEWORD(2,1)
        len = sizeof(client);
        local.sin_family = AF_INET;
        local.sin_port = htons(12344);  //設定port
        std::string IP("192.168.56.101");
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


//產生路徑規劃
class TrajectoryPlanning
{
private:
KDL::Chain chain;
KDL::JntArray q_init;
Eigen::Matrix<double, 6, 1> L;
KDL::Frame start_pos_cart;
KDL::Frame end_pos_cart;
Six_Ang start_ang_joint;
Six_Ang result_ang_joint;
Six_Ang end_ang_joint;
Ang_List_V ang_list;
physics::WorldPtr world;
MoveToPtr movetoptr;
SocketClient client;
double vel[6];
std::string targetname;
transport::PublisherPtr publisher_traj;
public:
void SetTargetModel(std::string param_name)
{
	targetname = param_name;
}


public:
TrajectoryPlanning(){}
TrajectoryPlanning(MoveToPtr param_movetoptr, physics::ModelPtr param_model)
{
	world=param_model->GetWorld();
	movetoptr=param_movetoptr;
	//起點軸座標
	start_ang_joint = Six_Ang(6, 0);
	//終點軸座標
	end_ang_joint = Six_Ang(6, 0);
	//??我也忘了
	result_ang_joint = Six_Ang(6, 0);
	//解IK需要初始值,其實好像用不到
	srand(time(NULL));
	using namespace KDL;
	//KDL初始化,設定dh model
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0,0.075,0.345))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.27))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0,0.106,0.09))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0,0.189,0))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0,0,0))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0,0.145,0))));
	//解IK需要初始值,其實好像用不到
	q_init = KDL::JntArray(6);
	q_init.data.setRandom();
	q_init.data *= M_PI;
	//這 不太清楚 照sample code打的
	L(0) = 1;
	L(1) = 1;
	L(2) = 1;
	L(3) = 0.01;
	L(4) = 0.01;
	L(5) = 0.01;
}
//輸入起點終點 產生中間一連串路徑點
public:
void GenerateTrajToObject(string whichitem,char type,transport::PublisherPtr puber)
{publisher_traj=puber;
	int modelID;
	for(uint i=0;i<world->GetModelCount();i++)
	{
		if(whichitem.compare(world->GetModel(i)->GetName())==0)
		{
			modelID=i;
			break;
		}
	}
	GenerateTrajToObject(modelID,type,puber);
}

public:
void GenerateTrajToObject(int whichitem,char type,transport::PublisherPtr puber)
{
	math::Pose targetpose;
	//減0.5是因為 手臂base在-0.5
	targetpose = world->GetModel(whichitem)->GetWorldPose()- math::Pose(math::Vector3(0,-0.5, 0), math::Quaternion(0, 0, 0));
	math::Pose tipppose = movetoptr->GetTipPose()- math::Pose(math::Vector3(0 ,-0.5, 0), math::Quaternion(0, 0, 0));
	Calculate_beta(tipppose, targetpose,type,puber);
	std::cout << "exit tra.Calculate\n";
}

public:
void GenerateTrajToObject_RCL(int whichitem,char type)
{
	math::Pose targetpose;
	//減0.5是因為 手臂base在-0.5
	targetpose = world->GetModel(whichitem)->GetWorldPose()- math::Pose(math::Vector3(0,-0.5,  0), math::Quaternion(0, 0, 0));
	math::Pose tipppose = movetoptr->GetTipPose()- math::Pose(math::Vector3(0,-0.5, 0), math::Quaternion(0, 0, 0));
	math::Pose base_offset=math::Pose(math::Vector3(0,0,0.25), math::Quaternion(0,0,0));
	targetpose=targetpose-base_offset;
	cout<<"modified target pose: "<<targetpose<<endl;

	Calculate_RCL(targetpose,type,movetoptr->GetJointAngs());
	std::cout << "exit tra.Calculate_RCL\n";
}
Ang_List_V GetAngList()
{
	return ang_list;
};

public:
int Calculate_RCL(gazebo::math::Pose end_pose_cart,char type,Six_Ang start_angle)
{

	float buffer_height = 0.015;
	using namespace KDL;
	//gazebo的pose轉成KDL的Frame
	//夾爪的抓取rot設得跟工件一樣
	Frame end_pose_in_KDL;
	Frame via_pose_in_KDL;
	if (type == 'h')//從側邊夾取 horizontal
	{
		math::Vector3 tmpx = end_pose_cart.rot.GetZAxis();
		math::Vector3 tmpy = tmpx.Cross(math::Vector3(0, 0, 1));
		math::Vector3 tmpz = tmpx.Cross(tmpy);
		math::Vector3 x = tmpx.Normalize();
		math::Vector3 y = tmpy.Normalize();
		math::Vector3 z = tmpz.Normalize();
		double roll, pitch, yaw;
		KDL::Rotation tmpr(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z);
		tmpr.GetRPY(roll, pitch, yaw);
		math::Quaternion aligner(roll, pitch, yaw);

		math::Vector3 tmp_pos = end_pose_cart.pos+ aligner.RotateVector(math::Vector3(0, 0, -1 * buffer_height));
		KDL::Vector tmpvec(tmp_pos.x, tmp_pos.y, tmp_pos.z);
		end_pose_in_KDL =Frame(KDL::Rotation(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z),
				//此時的tmpvec是終點的xyz
				tmpvec);

		tmp_pos = end_pose_cart.pos+ aligner.RotateVector(math::Vector3(0, 0, -0.15));
		tmpvec=KDL::Vector(tmp_pos.x, tmp_pos.y, tmp_pos.z);
		via_pose_in_KDL =Frame(KDL::Rotation(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z),
						//此時的tmpvec是終點的xyz
						tmpvec);

	}
	if (type == 'v')//從垂直方向夾取 vertical
	{
		math::Vector3 tmpx= end_pose_cart.rot.GetXAxis();
		math::Vector3 tmpy= end_pose_cart.rot.GetYAxis();
		math::Vector3 tmpz= end_pose_cart.rot.GetZAxis();
		math::Vector3 x= -1*tmpx.Normalize();
		math::Vector3 y= tmpy.Normalize();
		math::Vector3 z= -1*tmpz.Normalize();
		double roll, pitch, yaw;
		//所以夾爪面向是y
		KDL::Rotation tmpr(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z);
		tmpr.GetRPY(roll, pitch, yaw);
		math::Quaternion aligner(roll, pitch, yaw);
		math::Vector3 tmp_pos = end_pose_cart.pos+ aligner.RotateVector(math::Vector3(0, -1 * buffer_height, 0));
		KDL::Vector tmpvec = KDL::Vector(tmp_pos.x, tmp_pos.y, tmp_pos.z);
		end_pose_in_KDL=Frame(KDL::Rotation(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z),
				//此時的tmpvec是終點的xyz
				tmpvec);

		tmp_pos = end_pose_cart.pos+ aligner.RotateVector(math::Vector3(0, 0, -0.15));
		tmpvec=KDL::Vector(tmp_pos.x, tmp_pos.y, tmp_pos.z);
		via_pose_in_KDL =Frame(KDL::Rotation(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z),
						//此時的tmpvec是終點的xyz
						tmpvec);
	}
	///////////////////////////////////////////////////////
	//////以上得到終點
	//送起點 joint angle

	cout<<"start_angle:"<<start_angle<<endl;
	char tmp[80]="0";
	//start_angle  印成字串 給給RcL
	sprintf(tmp,"%f %f %f %f %f %f",start_angle[0].Radian(),start_angle[1].Radian(),start_angle[2].Radian(),start_angle[3].Radian(),start_angle[4].Radian(),start_angle[5].Radian());
	client.Send(tmp);//送出起點
	printf("start angle: %s\n",tmp);



	//送via point,因為要垂直接近
	//轉成x y z r p y
	cout<<"end_xyz:"<<via_pose_in_KDL.p.x()<<" "<<via_pose_in_KDL.p.y()<<" "<<via_pose_in_KDL.p.z()<<endl;
	double rcl_roll,rcl_pitch,rcl_yaw;//給RcL的參數
	via_pose_in_KDL.M.GetRPY(rcl_roll,rcl_pitch,rcl_yaw);
	sprintf(tmp,"%f %f %f %f %f %f",via_pose_in_KDL.p.x(),via_pose_in_KDL.p.y(),via_pose_in_KDL.p.z(),rcl_roll/3.14*180,rcl_pitch/3.14*180,rcl_yaw/3.14*180);
	client.Send(tmp);//送出終點
	printf("via point: %s\n",tmp);
	//送終點
	//轉成轉成x y z r p y
	cout<<"end_xyz:"<<end_pose_in_KDL.p.x()<<" "<<end_pose_in_KDL.p.y()<<" "<<end_pose_in_KDL.p.z()<<endl;
	end_pose_in_KDL.M.GetRPY(rcl_roll,rcl_pitch,rcl_yaw);
	sprintf(tmp,"%f %f %f %f %f %f",end_pose_in_KDL.p.x(),end_pose_in_KDL.p.y(),end_pose_in_KDL.p.z(),rcl_roll/3.14*180,rcl_pitch/3.14*180,rcl_yaw/3.14*180);
	client.Send(tmp);//送出終點
	printf("end pose: %s\n",tmp);

	//開始從rcl_server_gui接收
	SocketServer server;
	char *buffer="\0";
	buffer=server.Receive();
	ang_list.clear();//用前先重置,以免舊的路資資料沒被蓋掉
	while (strcmp(buffer, "end") != 0)
	{
		float num[6];
		sscanf(buffer, "%f %f %f %f %f %f ", &num[0], &num[1], &num[2],	&num[3], &num[4], &num[5]);
		//轉成弧度
		for(int i=0;i<6;i++)
			num[i]=num[i]/180*3.1415926;


		printf("Receive: %f %f %f %f %f %f  \n", num[0], num[1], num[2],num[3], num[4], num[5]);
		//assign array to vector
		Six_Ang six_ang(num, num + sizeof(num) / sizeof(float));

		ang_list.push_back(six_ang);
		buffer = server.Receive();
	}

	cout<<ang_list.size()/sizeof(Six_Ang)<<" pts totally"<<endl;
	std::cout << "Trajectory from  RCL done!" << std::endl;


}
public:
int Calculate_beta(gazebo::math::Pose start_pose_cart,gazebo::math::Pose end_pose_cart,char type,transport::PublisherPtr publisher_traj )
{

	float buffer_height = 0.015;

	using namespace KDL;
	//gazebo的pose轉成KDL的Frame

		math::Matrix3 rot_start = start_pose_cart.rot.GetAsMatrix3();
		KDL::Vector tmpvec(start_pose_cart.pos.x, start_pose_cart.pos.y,
				start_pose_cart.pos.z);

		Frame start_pose_in_KDL(KDL::Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1),
		//此時的tmpvec是起點的xyz
				tmpvec);

		Frame end_pose_in_KDL;
		//夾爪的抓取rot設得跟工件一樣
	if (type == 'h')	//從側邊夾取
	{
		math::Vector3 tmpz = end_pose_cart.rot.GetZAxis();
		math::Vector3 tmpx = tmpz.Cross(math::Vector3(0, 0, 1));
		math::Vector3 tmpy = tmpz.Cross(tmpx);
		math::Vector3 x = tmpx.Normalize();
		math::Vector3 y = tmpy.Normalize();
		math::Vector3 z = tmpz.Normalize();
		double roll, pitch, yaw;
		//所以夾爪面向是y
		KDL::Rotation tmpr(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z);
		tmpr.GetRPY(roll, pitch, yaw);
		math::Quaternion aligner(roll, pitch, yaw);
		math::Vector3 tmp_pos = end_pose_cart.pos
				+ aligner.RotateVector(math::Vector3(0, -1 * buffer_height, 0));
		tmpvec = KDL::Vector(tmp_pos.x, tmp_pos.y, tmp_pos.z);
		end_pose_in_KDL=Frame(
				KDL::Rotation(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z),
				//此時的tmpvec是終點的xyz
				tmpvec);
	}
	else if(type == 'v')	//從垂直方向夾取
	{
		//cout<<"socket pose: "<<end_pose_cart<<endl;
//		math::Vector3 tmpx= end_pose_cart.rot.GetXAxis();
//		math::Vector3 tmpy= end_pose_cart.rot.GetYAxis();
//		math::Vector3 tmpz= end_pose_cart.rot.GetZAxis();
//		math::Vector3 z = -1*tmpx.Normalize();
//		math::Vector3 x = -1*tmpy.Normalize();
//		math::Vector3 y = -1*tmpz.Normalize();
		double roll, pitch, yaw;
		//所以夾爪面向是y

//TODO:這邊投機,直接將pose設往下,而不是根據物件的pose
		KDL::Rotation tmpr(1,0,0,0,0,1,0,-1,0);
		cout<<"kdl rotation:\n"<<tmpr<<endl;
		tmpr.GetRPY(roll, pitch, yaw);
		math::Quaternion aligner(roll, pitch, yaw);
		tmpvec = KDL::Vector(end_pose_cart.pos.x, end_pose_cart.pos.y, end_pose_cart.pos.z);
		end_pose_in_KDL=Frame(tmpr,
				//此時的tmpvec是終點的xyz
				tmpvec);
		cout<<"end_pose_in_KDL\n"<<end_pose_in_KDL<<endl;
	}


	//以上得到路徑的起點與終點
	//接著,以下使用kdl的路徑規劃

	Path_RoundedComposite* path = new Path_RoundedComposite(0.2, 0.01,
			new RotationalInterpolation_SingleAxis());
	//設定要經過的frame
	path->Add(end_pose_in_KDL);

	path->Add(
			KDL::Frame(
					KDL::Rotation(end_pose_in_KDL.M.UnitX(),
							end_pose_in_KDL.M.UnitY(),
							end_pose_in_KDL.M.UnitZ()),
					KDL::Vector(end_pose_in_KDL.p.data[0],
							end_pose_in_KDL.p.data[1],
							end_pose_in_KDL.p.data[2] + 0.15)));
	cout<<"path->Add("<<endl;
	path->Add(start_pose_in_KDL);
	path->Finish();
	//設置速度速度profile
	VelocityProfile* velpref = new VelocityProfile_Trap(0.05, 0.2);
	velpref->SetProfile(0, path->PathLength());
	Trajectory* traject = new Trajectory_Segment(path, velpref);

	// use the trajectory
	//產生路徑後,將各點解IK,得到6軸,存入ang_list
	int n = chain.getNrOfJoints();
	cout<<"ChainIkSolverPos_LMA solver(chain, L);"<<endl;
	ChainIkSolverPos_LMA solver(chain, L);
	cout<<"JntArray q_sol(n);"<<endl;
	JntArray q_sol(n);
	//自己設initial condition
	cout<<"q_init.data(0, 0) =0.1;"<<endl;
	q_init.data(0, 0) =0.1;
	cout<<"	q_init.data(1, 0) =0.1;"<<endl;
	q_init.data(1, 0) =0.1;
	q_init.data(2, 0) =0.1;
	q_init.data(3, 0) =0.1;
	q_init.data(4, 0) =0.1;
	q_init.data(5, 0) =0.1;

	double dt = 0.1;
	cout<<"ang_list.clear();"<<endl;
	ang_list.clear();//用前先重置,以免舊的路資資料沒被蓋掉












	cout<<"for (double t = 0.0; t <= traject->Duration(); t += dt)"<<endl;
	for (double t = 0.0; t <= traject->Duration(); t += dt)
	{
		solver.CartToJnt(q_init, traject->Pos(t), q_sol);
		q_init.data.setRandom();        //每次初始值都隨機
		q_init.data *= M_PI;

		for (int q = 0; q < 6; q++)
		{
			//std::cout<<"..."<<std::endl;
			if (q_sol.data(q, 0) > M_PI)
				q_sol.data(q, 0) = q_sol.data(q, 0) - 2 * M_PI;
			if ((q_sol.data(q, 0) + M_PI) < 0)
				q_sol.data(q, 0) = q_sol.data(q, 0) + 2 * M_PI;
			result_ang_joint[q] = q_sol.data(q, 0);

		}
		q_init = q_sol;        //上個點的解 當新點的初始值
		q_sol.data(1, 0) = 3;        //進入while的條件

		Six_Ang each_result;
		each_result.push_back(result_ang_joint[0].Radian());
		each_result.push_back(result_ang_joint[1].Radian());
		each_result.push_back(result_ang_joint[2].Radian());
		each_result.push_back(result_ang_joint[3].Radian());
		each_result.push_back(result_ang_joint[4].Radian());
		each_result.push_back(result_ang_joint[5].Radian());
		//cout<< each_result<<endl;
		ang_list.push_back(each_result);
	}
	std::cout << ang_list.size() << " pts" << std::endl;
	std::cout << "Calculate_beta done!" << std::endl;
    std::reverse(ang_list.begin(),ang_list.end());
}
public:void ReverseAngList()
{
	std::reverse(ang_list.begin(),ang_list.end());
}
public:
double* GetJointVel()
{
	return vel;
}
public:

};



typedef const boost::shared_ptr<const my::msgs::PoseEstimationResult> ConstMsgsPoseEstimationResultPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Request> ConstMsgsRequestPtr;
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//沿著設定好的路線走,每個iteration,都送出一點
class AlongTrajectory
{
private:
physics::ModelPtr model;
physics::WorldPtr world;
//這list就是存放一連串要經過的點
Ang_List_V ang_list;
//紀錄目前走到第幾個點
int nth_step;
MoveToPtr movetoptr;
public:
AlongTrajectory()
{
	nth_step = 0;
}
public:
AlongTrajectory(MoveToPtr param_movetoptr, physics::ModelPtr param_model)
{
	movetoptr = param_movetoptr;
	model = param_model;
	world = model->GetWorld();
	nth_step = 0;

}
//設定路徑點清單
public:
void SetAngList(Ang_List_V param_ang_list)
{
	ang_list.assign(param_ang_list.begin(), param_ang_list.end());
}
//路徑點清單index歸0
public:
void Resetnth_step()
{
	nth_step = 0;
	movetoptr->ResetReached();
}
void Setnth_step()
{
	nth_step = ang_list.size();//假裝已到路徑終點
	movetoptr->SetReached();
}




public:void FinalPosition()
{


	//終點index
	int position_num=1;
	std::cin>>position_num;
	//一連串終點位置
		float 	detination[27]={
								-0.3,-0.3,0.3,
								-0.3,-0.4,0.3,
								-0.3,-0.5,0.3,
								-0.4,-0.3,0.3,
								-0.4,-0.4,0.3,
								-0.4,-0.5,0.3,
								-0.5,-0.3,0.3,
								-0.5,-0.4,0.3,
								-0.5,-0.5,0.3};

	std::cout<<"AlongTrajectory::FinalPosition\n";
	TrajectoryPlanning tra;
	math::Pose targetpose;
	//因為機器手臂原點不在0 0 0 所以要shift


	targetpose=math::Pose(
			detination[position_num*3-3],
			detination[position_num*3-2],
			detination[position_num*3-1],
			0,0,0);

	targetpose=targetpose-math::Pose(math::Vector3(0,-0.5, 0), math::Quaternion(0, 0, 0));

	math::Pose tipppose = movetoptr->GetTipPose()- math::Pose(math::Vector3(0 ,-0.5, 0), math::Quaternion(0, 0, 0));

	//tra.Calculate_beta(tipppose,targetpose,'v');

//	tra.Calculate_PutOnTable(
//							math::Pose(-0.743662,0.657557 ,1.14789, -3.14129 ,0.338589 ,-1.94315),
//							targetpose);
			//math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
	std::cout<<"exit tra.Calculate\n";
	ang_list=tra.GetAngList();
	std::reverse(ang_list.begin(),ang_list.end());
}















//TODO:以下幾個函數功能大同小異,應該想個方法整合

public:
void Update()
{
	//printf("%d %d %d\n",movetoptr->HasReached(),nth_step ,ang_list.size());
	if (movetoptr->HasReached() && nth_step < ang_list.size())
	{
		//cout<<"還沒到"<<endl;
		//每次都要重置"Reached"狀態,手臂才會往下一個位置前進
		movetoptr->ResetReached();
		//設定下一點
		movetoptr->SetDestination(ang_list[nth_step]);

		nth_step++;
		//為了讓他只顯示一次
		if (nth_step == (ang_list.size()))
		{
			std::cout << "reach the destination" << std::endl;
		}

	}

	movetoptr->Update();                //這是不要用繼承會比較好?!
}
};

class Gripper
{
private:
int grip;
physics::ModelPtr model;
physics::WorldPtr world;
physics::ModelPtr target_for_cheat_grip;

public:
Gripper()
{
}
public:
void SetCheatTarget(physics::ModelPtr param_model)
{
	target_for_cheat_grip = param_model;
}
public:
Gripper(physics::ModelPtr param_model)
{
	grip = 0;
	model = param_model;
	world = model->GetWorld();
}
public:
void Grip()
{
	grip = 1;
}
public:
void Grip_cheat()
{
	grip = 1;

	//std::cout<<"Grip_cheat : "<<target_for_cheat_grip->GetName()<<std::endl;
	//world->GetModel("result_visualize_socket")->SetWorldPose(math::Pose(10,10,10,0,0,0));
//	target_for_cheat_grip->SetEnabled( true );
//	target_for_cheat_grip->SetStatic( false );
//	model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),target_for_cheat_grip->GetLink("link"));

	//下面是單一model時debug用
	world->GetModel("socket")->SetEnabled(true);
	world->GetModel("socket")->SetStatic(false);
	model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),
			world->GetModel("socket")->GetLink("link"));
}
public:
void Realse_cheat()
{
	grip = 0;

	math::Pose dummypose;
	math::Pose plampose = model->GetLink("gripper::plam")->GetWorldPose();
	dummypose = plampose
			+ math::Pose(plampose.rot.RotateVector(math::Vector3(0, 0, -0.05)),
					math::Quaternion(0, 0, 0));
	model->GetLink("gripper::dummy_for_glue")->SetWorldPose(dummypose);

//	target_for_cheat_grip->SetEnabled( true );
//	target_for_cheat_grip->SetStatic( false );
	//下面是單一model時debug用
	world->GetModel("socket")->SetEnabled(true);
	world->GetModel("socket")->SetStatic(false);

	model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),
			model->GetLink("gripper::dummy_for_glue"));
}

public:
void Release()
{
	grip = 0;
}
public:
int IsGrip()
{
	return grip;
}
public:
void Update()
{
	if (grip == 0)
	{
		model->GetLink("gripper::right_finger")->AddRelativeForce(
				math::Vector3(0, 5, 0));
		model->GetLink("gripper::left_finger")->AddRelativeForce(
				math::Vector3(0, -5, 0));
	}
	if (grip == 1)
	{
		model->GetLink("gripper::right_finger")->AddRelativeForce(
				math::Vector3(0, -5, 0));
		model->GetLink("gripper::left_finger")->AddRelativeForce(
				math::Vector3(0, 5, 0));
	}
}
};

class KDL_IK
{
private:
MoveToPtr movetoptr;
KDL::Chain chain;
KDL::JntArray q_init;
Eigen::Matrix<double, 6, 1> L;
Six_Ang ang;
public:
KDL_IK()
{
}
public:
KDL_IK(MoveToPtr param_movetoptr)
{
	movetoptr = param_movetoptr;
	movetoptr->SetDestination(Six_Ang(6, 0));
	using namespace KDL;

    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0,0.075,0.345))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.27))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0,0.106,0.09))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0,0.189,0))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0,0,0))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0,0.145,0))));
	ang = Six_Ang(6, 0);
	q_init = KDL::JntArray(6);
	q_init.data.setRandom();
	q_init.data *= M_PI;
	movetoptr->SetReached();    //先設為reached,否則不是reached的話,他就會開始走

}
public:
int CalculateIK(float x, float y, float z)
{
	using namespace KDL;

	int n = chain.getNrOfJoints();
	Eigen::Matrix<double, 6, 1> L;
	L(0) = 1;
	L(1) = 1;
	L(2) = 1;
	L(3) = 0.01;
	L(4) = 0.01;
	ChainIkSolverPos_LMA solver(chain, L);
	JntArray q_sol(n);
	q_sol.data(1, 0) = 3;    //進入while的條件
	int retval, count = 0;
	//夾爪方向往下
	Frame pos_goal;
//    while(retval||(q_sol.data(1,0)<-1.57)||(q_sol.data(1,0)>1.57))//不要大於1.57,超出這範圍,會撞到地
//    {
//      std::cout<<"fgfrg \n  "<<count++;
//      if(count>10)
//      {
//              return 0;
//      }
	//每次初始值都隨機
	//q_init.data.setRandom();
	//q_init.data *= M_PI;
	Six_Ang ang_for_init = movetoptr->GetJointAngs();
	q_init.data(0, 0) = ang_for_init[0].Radian();
	q_init.data(1, 0) = ang_for_init[1].Radian();
	q_init.data(2, 0) = ang_for_init[2].Radian();
	q_init.data(3, 0) = ang_for_init[3].Radian();
	q_init.data(4, 0) = ang_for_init[4].Radian();
	q_init.data(5, 0) = ang_for_init[5].Radian();

	//以目前位置當作初始值
	//按照這隻手臂的初始狀態來看，夾爪所指方向是y方向
	pos_goal = Frame(KDL::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0),
			KDL::Vector(x, y, z));
	retval = solver.CartToJnt(q_init, pos_goal, q_sol);
	//std::cout<<"IK solver return value: "<<retval<<std::endl;
	// }

	std::cout << "q_sol : " << q_sol.data << std::endl;
	for (int i = 0; i < 6; i++)
	{
		if (q_sol.data(i, 0) > M_PI)
			q_sol.data(i, 0) = q_sol.data(i, 0) - 2 * M_PI;
		if ((q_sol.data(i, 0) + M_PI) < 0)
			q_sol.data(i, 0) = q_sol.data(i, 0) + 2 * M_PI;
		ang[i] = q_sol.data(i, 0);
	}

	movetoptr->ResetReached();
	movetoptr->SetDestination(ang);
}
};

class Arm_Control3_ar6axis: public ModelPlugin
{
public:
~Arm_Control3_ar6axis()
{

}

public:
void Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
	this->model = _parent;
	world = model->GetWorld();
	//開一個thread,監聽鍵盤輸入
	_Thread = boost::thread(&Arm_Control3_ar6axis::Keyboard_Input, this);
	//初始化與evaluation通信
	node = transport::NodePtr(new transport::Node());
	node->Init("default");


	//註冊每次iteration都會呼叫這函式
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&Arm_Control3_ar6axis::OnUpdate, this, _1));
}
public:
void Keyboard_Input()
{
	std::cout << "waiting for input...";
	while (1)
	{
		if (std::cin.get() == '\n')    //按下enter,停止目前動作
		{

			std::cout << "\n Menu:\n\n"
					<< "10\t Subscribe\n"
					<< "11\t UnSubscribe\n"
					<<endl;


			//std::cout << "\n Menu:\n\n"
			//		<< "0\t go to(joint space)\n"
			//		<< "1\t go home \n"
			//		<< "2\t go to the object (KDL)\n"
			//		<<"22\t go to the object (RCL)\n"
//					<< "3\t from Evaluation Platform (KDL)\n"
//					<<"33\t from Evaluation Platform (RCL)\n"
//					<< "4\t grip_cheat\n"
//					<< "5\t release_cheat\n"
//					<< "6\t list all models\n"
//					<< "7\t 2nd position \n"//二次定位
//					<< "8\t print Tip pose\n"
//					<< "9\t to finalPosition\n"
//					<< "10\t Subscribe\n"
//					<< "11\t UnSubscribe\n"
//					<<endl;
			scanf("%d", &userinput);
			std::cin.ignore();

			if (userinput == 0)    //到達空間中某ㄧ點
			{
				movetoptr->SetSpecificJoint();
				movetoptr->ResetReached();     //重置'Reached'的狀態,才能開始跑
			}
			if (userinput == 1)    //先到達預備位置
			{
				alongtraj.Setnth_step();//不再執行路徑
				movetoptr->SetGoHome();    //設定預設終點
				// double vel[6]={.75,.75,.75,.75,.75,.75};//給定速度(也可以不給,因為有預設的了)
				//movetoptr->SetVelocityOfJoint(vel);//給定速度(也可以不給,因為有預設的了)
				movetoptr->ResetReached();                //重置'Reached'的狀態,才能開始跑
			}
			if (userinput == 2)
			{
			    int whichitem=0;
			    char type;
			    std::cout<<"which item?approaching type?\n";
			    scanf("%d",&whichitem);std::cin.ignore();
			    scanf("%c",&type);std::cin.ignore();
			    //trajplanning.GenerateTrajToObject(whichitem,type);//.GenerateTrajToObject();
				alongtraj.SetAngList(trajplanning.GetAngList());
				alongtraj.Resetnth_step();                //重置'Reached'的狀態,才能開始跑
			}
			if (userinput == 22)
			{
			    int whichitem=0;
			    char type;
			    std::cout<<"which item?approaching type?\n";
			    scanf("%d",&whichitem);std::cin.ignore();
			    scanf("%c",&type);std::cin.ignore();
			    trajplanning.GenerateTrajToObject_RCL(whichitem,type);//.GenerateTrajToObject();
				alongtraj.SetAngList(trajplanning.GetAngList());
				alongtraj.Resetnth_step();                //重置'Reached'的狀態,才能開始跑
			}

			if (userinput == 3)
			{
				int whichitem;
			    char type;
			    std::cout<<"approaching type?\n";
			    scanf("%c",&type);std::cin.ignore();
			    for(int k=0;k<world->GetModelCount();k++)
			    {
			    	if(nearestmodel.compare(world->GetModel(k)->GetName())==0)
			    	{
			    		whichitem=k;
			    		break;
			    	}
			    }
			    //trajplanning.GenerateTrajToObject(whichitem,type);//.GenerateTrajToObject();
				alongtraj.SetAngList(trajplanning.GetAngList());
				alongtraj.Resetnth_step();                //重置'Reached'的狀態,才能開始跑
			}
            if(userinput==4)
            {
                gripper.Grip_cheat();
            }
            if(userinput==5)
            {
                gripper.Realse_cheat();
            }

			if(userinput== 6)                //列出場上所有model
			{
				listallmodel.ForceUpdate();
			}
			if(userinput==7)
			{
				alongtraj.Setnth_step();//不再執行路徑
				movetoptr->Set2ndPosition();
				movetoptr->ResetReached();
			}
			if(userinput==8)
			{
				cout<<"tip pose:\n"<<movetoptr->GetTipPose()<<endl;
				movetoptr->PrintTipPose_KDL();
			}
			if(userinput==9)
			{
				alongtraj.FinalPosition();
                alongtraj.Resetnth_step();
			}
			if(userinput==10)
			{
				subscribe_GGininder = node->Subscribe("~/SystemGUI",&Arm_Control3_ar6axis::receivedGGininder, this);
				publisher_traj_ = node->Advertise< gazebo::msgs::Request >( "~/traj" );
				cout<<"subscribe the topic: "<<subscribe_GGininder->GetTopic()<<endl;
			}
			if(userinput==11)
			{
				subscribe_GGininder->Unsubscribe();
				cout<<"subscribe the topic: "<<subscribe_GGininder->GetTopic()<<endl;
			}
		}
	}
}
public:
void OnUpdate(const common::UpdateInfo & /*_info*/)
{

	listallmodel.Update();                //在場上的的modell數量有變化 就會print出model列表

	gripper.Update();
	//alongtraj比較特殊，這是控制手臂通過一連串的點，所以不能由movetoptr->update控制

	alongtraj.Update();     //movetoptr->update只能控制一個起點到一個終點

}

public:
void receivedGGininder(ConstMsgsRequestPtr &_msg)
{
	/*nearestmodel = _msg->request();//從Evalution platform得到的結果
	std::cout << "\033[7;32m" << "Arm_Control receive : " << nearestmodel
			<< "\033[m" << std::endl;
	gripper.SetCheatTarget(world->GetModel(nearestmodel));
	//這邊還沒做好 暫且comment
	//alongtraj.CatchObjectInBoxFromMsg();
*/
	std::string request=_msg->request();
	cout<<"Receive: "<<request<<"\n"
			<<request.compare("Go Home")<<"\n"
			<<request.compare("Grip")<<"\n"
			<<request.compare("Release")<<"\n"
			<<endl;

	if (request.compare("Go Home")==0)    //先到達預備位置
	{
		alongtraj.Setnth_step();//不再執行路徑
		movetoptr->SetGoHome();    //設定預設終點
		movetoptr->ResetReached();
		cout<<"exeute \"Go Home\""<<endl;
	}
	else if (request.compare("Grip")==0)    //先到達預備位置
	{
		gripper.Grip();
		cout<<"exeute \"Grip\""<<endl;
	}
	else if (request.compare("Release")==0)    //先到達預備位置
	{
		gripper.Release();
		cout<<"exeute \"Release\""<<endl;
	}
	else //抓取model
	{
		string target_model = _msg->request();
	    trajplanning.GenerateTrajToObject(target_model,'h',publisher_traj_);//.GenerateTrajToObject();
		alongtraj.SetAngList(trajplanning.GetAngList());
		alongtraj.Resetnth_step();

		 //////publish traj////////////////////
			    cout<<"\nTrajectory planning\n"<<endl;
			    std::stringstream traj_pts_ss;
			    traj_pts_ss.clear();
			    traj_pts_ss.str("");

				msgs::Request traj_points;
				traj_points.set_id( 0 );
				traj_points.set_request("LLLLLLLLL");
				cout<<"publisher_traj_->HasConnections() :"<<publisher_traj_->HasConnections()<<endl;
				while( !publisher_traj_->HasConnections() )
				{
					cout <<  "\033[1;31m" << "have no depth sensor connected!" << "\033[0m" << endl;
					gazebo::common::Time::Sleep(500 );
				}
				publisher_traj_->Publish( traj_points );
				cout<<"publisher_traj_->Publish( traj_points )"<<endl;






	}

}

public:
void Init()
{
	movetoptr = MoveToPtr(new MoveTo(model));
	listallmodel = ListAllModel(model);
	gripper = Gripper(model);
	alongtraj = AlongTrajectory(movetoptr, model);
	trajplanning= TrajectoryPlanning(movetoptr, model);
	kdlik = KDL_IK(movetoptr);
	userinput = 0;

	movetoptr->SetGoHome();
	movetoptr->ResetReached();
}
private:
int userinput;
physics::ModelPtr model;
physics::WorldPtr world;
event::ConnectionPtr updateConnection;
event::ConnectionPtr deleteEntityEvent;

ListAllModel listallmodel;
AlongTrajectory alongtraj;
TrajectoryPlanning trajplanning;
Gripper gripper;
KDL_IK kdlik;
MoveToPtr movetoptr;
boost::thread _Thread;
transport::NodePtr node;
transport::SubscriberPtr subscribe_GGininder;
transport::PublisherPtr publisher_traj_;
std::string nearestmodel;
};

GZ_REGISTER_MODEL_PLUGIN(Arm_Control3_ar6axis);
