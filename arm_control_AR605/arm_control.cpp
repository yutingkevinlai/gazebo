//AR07 robot
#include <kdl/frames.hpp>
#include <gazebo/math/Pose.hh>
#include <kdl/jntarray.hpp>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Matrix4.hh>
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
#include <ostream>
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

typedef std::vector<gazebo::math::Angle> Six_Ang;
typedef std::vector<Six_Ang> Ang_List_V;

using namespace std;
using namespace gazebo;

std::ostream&  operator << (std::ostream &out,Six_Ang &Six_Ang_param)
{
	out << Six_Ang_param[0] << " , "
		<< Six_Ang_param[1] << " , "
		<< Six_Ang_param[2] << " , "
		<< Six_Ang_param[3] << " , "
		<< Six_Ang_param[4] << " , "
		<< Six_Ang_param[5] << "\n";

	return out;
}

// don't need to use socket here
/*
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
    	std::string IP("192.168.56.101");
    	server.sin_addr.s_addr = inet_addr(IP.c_str()/*ip*///);
 /*
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
    	cout << "~SocketServer()" << endl;
    	CloseSocket();
    }
    SocketServer()
    {
        //WSAStartup(MAKEWORD(2,1),&wsaData);//初始windows socket application,並設定版本MAKEWORD(2,1)
        len = sizeof(client);
        local.sin_family = AF_INET;
        local.sin_port = htons(12345);  //設定port
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
}; // end Class SocketServer


*/


class ListAllModel
{
	private:
		int num_model_on_field;
		physics::WorldPtr world;

	public:ListAllModel(){}

	public:ListAllModel(physics::ModelPtr param_model)
	{
		world = param_model->GetWorld();
		num_model_on_field = 0;
	}

	public:void ForceUpdate() // userinput = 5
	{
		num_model_on_field = 0;
	}

	public:void Update()
	{
		if( world->GetModelCount() != num_model_on_field ) // if 在場上的 model 數量有變化
		{
			num_model_on_field = world->GetModelCount();
			std::cout << COUT_PREFIX << "Now There are " << num_model_on_field << " models on the field" << std::endl;
			for(int i=0; i<num_model_on_field; i++) // 列出所有在場上的model名稱
			{
				std::cout << "\t" << i << "\t" << world->GetModel(i)->GetName() << "\t(" << world->GetModel(i)->GetWorldPose().pos << ")" << std::endl;
			}
			std::cout << "\n" << std::endl;
		}
	}
}; // end Class ListAllModel

class MoveTo
{
	private:

		double velocityOfjoint[6];
		int check_joint[12];
		physics::ModelPtr model;
		physics::WorldPtr world;
		physics::Link_V link;
		physics::Link_V axis_aligner;
		physics::Joint_V joint;
		Six_Ang currentang;
		char axis[6];

	public:
		Six_Ang ang;

	public:MoveTo(){}

	public:int HasReached()
	{
		// check_joint[1] 如果是1,表示目前第1軸位置比目標位置大
		// check_joint[0] 如果是1,表示目前第1軸位置比目標位置小
		// check_joint[1] 與 check_joint[0] 同時等於1,表示目前第1軸位置比目標位置大,同時,第1軸位置比目標位置小
		// 也就是說第1軸已達到目標位置
		if( check_joint[1]&&check_joint[0]&&
			check_joint[3]&&check_joint[2]&&
			check_joint[5]&&check_joint[4]&&
			check_joint[7]&&check_joint[6]&&
			check_joint[9]&&check_joint[8]&&
			check_joint[11]&&check_joint[10])
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}

	public:void ResetReached()
	{
		std::cout << "MoveTo::ResetReached" << std::endl;
		for(int i=0;i<12;i++)
		{
			check_joint[i] = 0;
			//std::cout << check_joint[i];
		}
		//std::cout << std::endl;
	}

	public:void SetReached()
	{
		for(int i=0;i<12;i++)
		{
			check_joint[i] = 1;
		}
	}

	public:MoveTo(physics::ModelPtr param_model)
	{
		for(int i=0;i<6;i++)
		{
			check_joint[2*i] = 0;
			check_joint[2*i+1] = 0;
		}

		velocityOfjoint[0] = 1;
		velocityOfjoint[1] = 1;
		velocityOfjoint[2] = 1;
		velocityOfjoint[3] = 1;
		velocityOfjoint[4] = 1;
		velocityOfjoint[5] = 1;

		currentang = Six_Ang(6,0);
		ang.push_back((math::Angle)0);
		ang.push_back((math::Angle)0);
		ang.push_back((math::Angle)0);
		ang.push_back((math::Angle)0);
		ang.push_back((math::Angle)0);
		ang.push_back((math::Angle)0);
		axis[0] = 'z';
		axis[1] = 'y';
		axis[2] = 'y';
		axis[3] = 'x';
		axis[4] = 'y';
		axis[5] = 'z';
		world = param_model->GetWorld();
		model = param_model;
		link.push_back(model->GetLink("link1"));
		link.push_back(model->GetLink("link2"));
		link.push_back(model->GetLink("link3"));
		link.push_back(model->GetLink("link4"));
		link.push_back(model->GetLink("link5"));
		link.push_back(model->GetLink("link6"));
		joint.push_back(model->GetJoint("joint1"));
		joint.push_back(model->GetJoint("joint2"));
		joint.push_back(model->GetJoint("joint3"));
		joint.push_back(model->GetJoint("joint4"));
		joint.push_back(model->GetJoint("joint5"));
		joint.push_back(model->GetJoint("joint6"));
		axis_aligner.push_back(model->GetLink("base"));
		axis_aligner.push_back(model->GetLink("link1"));
		axis_aligner.push_back(model->GetLink("link2"));
		axis_aligner.push_back(model->GetLink("link3"));
		axis_aligner.push_back(model->GetLink("link4"));
		axis_aligner.push_back(model->GetLink("link5"));
	}

	public:void SetDestination(Six_Ang param_ang)
	{
		std::cout << "MoveTo::SetDestination" << std::endl;
		ang.assign(param_ang.begin(),param_ang.end());
	}

	public:Six_Ang GetJointAngs()
	{
		return ang;
	}

	public:void SetSpecificJoint()
	{
		std::cout << "\nangle of each joint" << std::endl;
		for(int i=0;i<6;i++)
		{
			std::cout << i+1 << ", " << ang[i] << std::endl;
		}
		int n;
		float input_angle;
		std::cout << "input joint angle" << std::endl;
		scanf("%d %f",&n,&input_angle);
		if((((n==1)&&(input_angle>2.9670))||((n==1)&&(input_angle<-2.9670)))||
		   (((n==2)&&(input_angle>2.2689))||((n==2)&&(input_angle<-1.4835)))||
		   (((n==3)&&(input_angle>2.9670))||((n==3)&&(input_angle<-1.9190)))||
		   (((n==4)&&(input_angle>3.3160))||((n==4)&&(input_angle<-3.3160)))||
		   (((n==5)&&(input_angle>2.1816))||((n==5)&&(input_angle<-2.1816)))||
		   (((n==6)&&(input_angle>6.2800))||((n==6)&&(input_angle<-6.2800))))
		{
			std::cout << CERR_PREFIX << "input angle exceed limits" << std::endl;
			return;
		}
		ang[n-1] = (math::Angle)input_angle;
		std::cin.ignore();
		ResetReached();
	}

	public:void SetDefaultDestination() // end-effector default destination
	{
		//0.0107359  0.0215242  -0.00251389  -0.000346409  3.1217  -3.13054
		//ang[0] = (math::Angle)(0.0107359);
		ang[0] = (math::Angle)(0.0);
		//ang[1] = (math::Angle)(0.0215242);
		ang[1] = (math::Angle)(0.0);
		//ang[2] = (math::Angle)(0.00251389);
		ang[2] = (math::Angle)(0.0);
		ang[3] = (math::Angle)(0);
		ang[4] = (math::Angle)(3.14);
		ang[5] = (math::Angle)(3.14);
		//0.0114845 , 0.0273861 , -0.00136122 , -0.000174772 , 3.11542 , -3.12992
		//0.8  0.157  2.5  2.5  1.15  0.2
	}

	public:void SetSmallTableDestination() // go to the small table
	{
		ang[0] = (math::Angle)(0.525);
		ang[1] = (math::Angle)(1.05);
		ang[2] = (math::Angle)(0.4);
		ang[3] = (math::Angle)(0.00059147);
		ang[4] = (math::Angle)(0.1);
		ang[5] = (math::Angle)(3.14159);
	}

	public:void SetTableDestination() // end-effector move to table
	{
		int input;
		scanf("%d",&input);
		if(input == 0) // small table
		{
			ang[0] = (math::Angle)(0.525);
			ang[1] = (math::Angle)(1.05);
			ang[2] = (math::Angle)(0.4);
			ang[3] = (math::Angle)(0.00059147);
			ang[4] = (math::Angle)(0.1);
			ang[5] = (math::Angle)(3.14159);

	//		ang[0]=(math::Angle)(0.8);
	//		ang[1]=(math::Angle)(0.157);
	//		ang[2]=(math::Angle)(2.5);
	//		ang[3]=(math::Angle)2.5;
	//		ang[4]=(math::Angle)(1.15);
	//		ang[5]=(math::Angle)0.25;
		}
		else if(input==1)
		{
		//-1.10174 1.10357 0.664433
			ang[0] = (math::Angle)(1.0211 );
			ang[1] = (math::Angle)(0.566093);
			ang[2] = (math::Angle)(0.789985 );
			ang[3] = (math::Angle)(0.00144849 );
			ang[4] = (math::Angle)(1.78496 );
			ang[5] = (math::Angle)(1.00123);
		}
		else if(input==2)
		{

			ang[0] = (math::Angle)(1.26428 );
			ang[1] = (math::Angle)(0.475643 );
			ang[2] = (math::Angle)(  0.967921);
			ang[3] = (math::Angle)(  0.00464885 );
			ang[4] = (math::Angle)(1.69752 );
			ang[5] = (math::Angle)(  1.21103);
		}
		else if(input==3)
		{

			ang[0] = (math::Angle)(1.54489  );
			ang[1] = (math::Angle)( 0.436451 );
			ang[2] = (math::Angle)( 1.04554 );
			ang[3] = (math::Angle)(0.00981926   );
			ang[4] = (math::Angle)(1.65953 );
			ang[5] = (math::Angle)(1.47132);
		}
		else if(input==4)
		{
		// -1.10085 1.00461 0.664927
			ang[0] = (math::Angle)(1.67171);
			ang[1] = (math::Angle)(  0.24956);
			ang[2] = (math::Angle)(  1.91472 );
			ang[3] = (math::Angle)( -0.00107202 );
			ang[4] = (math::Angle)( 0.965988  );
			ang[5] = (math::Angle)(-1.41532);
		}
		else if(input==5)
		{
		// -1.00232 1.00337 0.66543
			ang[0] = (math::Angle)(1.57322  );
			ang[1] = (math::Angle)(0.242818);
			ang[2] = (math::Angle)(  1.92257 );
			ang[3] = (math::Angle)( 0.000253391 );
			ang[4] = (math::Angle)( 0.964214 );
			ang[5] = (math::Angle)( -1.5122);
		}
		else if(input==6)
		{
		// -0.9035 1.00261 0.665944
			ang[0] = (math::Angle)( 1.47461    );
			ang[1] = (math::Angle)( 0.246254   );
			ang[2] = (math::Angle)( 1.91761    );
			ang[3] = (math::Angle)( 0.00171927 );
			ang[4] = (math::Angle)( 0.965169   );
			ang[5] = (math::Angle)( -1.60936   );
		}
	}

	public:void SetVelocityOfJoint(double param_velocityOfjoint[6])
	{
		for(int i=0;i<6;i++)
		{
			velocityOfjoint[i] = param_velocityOfjoint[i];
			//printf("%lf ",velocityOfjoint[i]);
		}
		printf("\n");

	}

	public:void SetPositionDirectly()
	{
		for(int i=0;i<6;i++) // 就定位後就固定住
		{
			link[i]->SetAngularVel(math::Vector3(0,0,0));
			joint[i]->SetHighStop(0,ang[i] + math::Angle(0.001));
			joint[i]->SetLowStop(0,ang[i] + math::Angle(-0.001));
		}
	}

	public:void Update(int type = 0) // Class MoveTo
	{
		// printf("%d %d %d %d %d %d %d %d %d %d %d %d \n",check_joint[1],check_joint[0],check_joint[3],check_joint[2],check_joint[5],check_joint[4],check_joint[7],check_joint[6],check_joint[9],check_joint[8],check_joint[11],check_joint[10]);

		if(check_joint[1]&&check_joint[0]&&check_joint[3]&&check_joint[2]&&check_joint[5]&&check_joint[4]&&check_joint[7]&&check_joint[6]&&check_joint[9]&&check_joint[8]&&check_joint[11]&&check_joint[10])
		{
			SetPositionDirectly();
			//SaveCurrentAngle();
		}
		else // 各joint還沒就定位,就繼續跑
		{
			for(int i=0;i<6;i++)
			{
				// std::cout << joint[i]->GetAngle(0) << "\t";
				if( type == 0 )
				{
					SetEachLinkPosition(i);
				}
			}
			// std::cout << std::endl;
		}
	}

	private:void SetEachLinkPosition(int nth) // Class MoveTo
	{
		//printf("%f %f %f %f %f %f \n",velocityOfjoint[0],velocityOfjoint[1],velocityOfjoint[2],velocityOfjoint[3],velocityOfjoint[4],velocityOfjoint[5]);
		if(!( check_joint[nth*2+1] && check_joint[nth*2] )) // 代表這個joint還沒到達目標
		{
			// 正要處理
			// 限制軸的活動範圍
			joint[nth]->SetHighStop(0,math::Angle(10));
			joint[nth]->SetLowStop(0,math::Angle(-10));
			if(((joint[nth]->GetAngle(0)-ang[nth])*(joint[nth]->GetAngle(0)-ang[nth]))<0.00001)
			{
				// 目前軸位置與目標位置誤差小於 0.00001 , 就判定為到達
				//std::cout<<axis[nth]<<nth<<" on the position! \n";
				check_joint[nth*2+1] = 1;
				check_joint[nth*2] = 1;
			}
			else if(joint[nth]->GetAngle(0)<=ang[nth]) // 目前軸位置比目標位置小, 就繼續加
			{
				// 軸有 3 種可能的軸向
				// if(nth==1)std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" > "<<joint[nth]->GetAngle(0)<<" "<<velocityOfjoint[nth]<<std::endl;
				if(axis[nth]=='x')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(velocityOfjoint[nth],0,0)));
				if(axis[nth]=='y')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,velocityOfjoint[nth],0)));
				if(axis[nth]=='z')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,0,velocityOfjoint[nth])));
				check_joint[nth*2+1] = 1;
			}
			else if(joint[nth]->GetAngle(0)>ang[nth]) // 目前軸位置比目標位置大, 就繼續減
			{
				//if(nth==1)std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" < "<<joint[nth]->GetAngle(0)<<" "<<velocityOfjoint[nth]<<std::endl;
				if(axis[nth]=='x')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(-velocityOfjoint[nth],0,0)));
				if(axis[nth]=='y')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,-velocityOfjoint[nth],0)));
				if(axis[nth]=='z')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,0,-velocityOfjoint[nth])));
				check_joint[nth*2] = 1;
			}
		}
		else // 以下是處理好的
		{
			joint[nth]->SetHighStop(0, ang[nth] + math::Angle(0.001));
			joint[nth]->SetLowStop(0, ang[nth] + math::Angle(-0.001));
		}
	}

	// 按照順序來,第一軸到達後,第二軸才能動,
	// 第二軸到達後,第三軸才能動,依序下去...
	// 這功能好像用不太到了
	// TODO:可以砍了
	private:void SetEachLinkPositionOrder(int nth) // Class MoveTo
	{
		//std::cout<<" SetEachLinkPositionOrder"<<std::endl;
		//printf("%d %d %d %d %d %d %d %d %d %d %d %d \n",check_joint[1],check_joint[0],check_joint[3],check_joint[2],check_joint[5],check_joint[4],check_joint[7],check_joint[6],check_joint[9],check_joint[8],check_joint[11],check_joint[10]);
		//printf("%f %f %f %f %f %f \n",velocityOfjoint[0],velocityOfjoint[1],velocityOfjoint[2],velocityOfjoint[3],velocityOfjoint[4],velocityOfjoint[5]);

		if(check_joint[((nth*2-1)<0?12:(nth*2-1))]&&check_joint[((nth*2-2)<0?12:(nth*2-2))]&&!(check_joint[nth*2+1]&&check_joint[nth*2]))
			{
			for(int i=0;i<6;i++)
			{
				if(i!=nth)link[i]->SetAngularVel(math::Vector3(0,0,0));
			}
			//以下是處理好的
			for(int i=0;i<nth;i++)
			{
				joint[i]->SetHighStop(0,ang[i]+math::Angle(0.001));joint[i]->SetLowStop(0,ang[i]+math::Angle(-0.001));
			}
			//正要處理
			joint[nth]->SetHighStop(0,math::Angle(4));joint[nth]->SetLowStop(0,math::Angle(-4));
			if(((joint[nth]->GetAngle(0)-ang[nth])*(joint[nth]->GetAngle(0)-ang[nth]))<0.00001)
			{
				std::cout<<"Order : "<<axis[nth]<<nth<<" on the position! \n";
				check_joint[nth*2+1]=1;
				check_joint[nth*2]=1;
			}
			else if(joint[nth]->GetAngle(0)<=ang[nth])
			{
				//std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" > "<<joint[nth]->GetAngle(0)<<std::endl;
				if(axis[nth]=='y')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,velocityOfjoint[nth],0)));
				if(axis[nth]=='z')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,0,velocityOfjoint[nth])));
				check_joint[nth*2+1]=1;
			}
			else if(joint[nth]->GetAngle(0)>ang[nth])
			{
				//std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" < "<<joint[nth]->GetAngle(0)<<std::endl;
				if(axis[nth]=='y')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,-velocityOfjoint[nth],0)));
				if(axis[nth]=='z')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,0,-velocityOfjoint[nth])));
				check_joint[nth*2]=1;
			}
		}
	}

	public:void PrintTipPose() // print end-effector pose
	{

		math::Pose plampose = model->GetLink("gripper::plam")->GetWorldPose();
		math::Pose tippose = plampose + math::Pose(plampose.rot.RotateVector(math::Vector3(0,0,-0.05)),math::Quaternion(0,0,0));

		std::cout << "position of tip : " << tippose << std::endl;
				 //<<link[5]->GetWorldPose().pos+link[5]->GetWorldPose().rot.RotateVector(math::Vector3(0,0,3.5))
				 //<<"\n";
		std::cout << "joint angle\n";
		for(int i=0;i<6;i++)
		{
			std::cout << ang[i] << "  ";
		}
		std::cout << std::endl;

		std::cout << "KDL_FK" << std::endl;
		KDL::Chain chain;
		using namespace KDL;
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.075,0.0,0.345))));
		chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.27))));
		chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.106,0.0,0.09))));
		chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.189,0.0,0))));
		chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0,0,0))));
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0,0,0.145))));

		KDL::ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
		KDL::JntArray jointpositions(6) ;
		jointpositions.data(0,0) = ang[0].Radian();
		jointpositions.data(1,0) = ang[1].Radian();
		jointpositions.data(2,0) = ang[2].Radian();
		jointpositions.data(3,0) = ang[3].Radian();
		jointpositions.data(4,0) = ang[4].Radian();
		jointpositions.data(5,0) = ang[5].Radian();

		KDL::Frame cartpos;
		fksolver.JntToCart(jointpositions,cartpos);
		cartpos.p.data[0] = cartpos.p.data[0]-0.5;
		std::cout << "pose :\n" << cartpos << std::endl;
	}

	public:math::Pose GetTipPose() // get end-effector pose
	{
		math::Pose plampose = model->GetLink("gripper::plam")->GetWorldPose();
		math::Pose tippose = plampose + math::Pose(plampose.rot.RotateVector(math::Vector3(0,0,-0.05)),math::Quaternion(0,0,0));

		return tippose;
	}
}; // end Class MoveTo

typedef boost::shared_ptr<MoveTo> MoveToPtr;

///////////////////////////////////////////////////////////////
//////////////// TrajectoryPlanning 產生路徑規劃 ////////////////
///////////////////////////////////////////////////////////////
class TrajectoryPlanning
{
	private:
		KDL::Chain chain;
		KDL::JntArray q_init;
		Eigen::Matrix<double,6,1> L;
		KDL::Frame start_pos_cart;
		KDL::Frame end_pos_cart;
		Six_Ang start_ang_joint;
		Six_Ang result_ang_joint;
		Six_Ang end_ang_joint;
		Ang_List_V ang_list;
		int grasp_type;
		double vel[6];
		physics::WorldPtr world;

	public:TrajectoryPlanning()
	{
		start_ang_joint = Six_Ang(6,0);
		end_ang_joint = Six_Ang(6,0);
		result_ang_joint = Six_Ang(6,0);
		srand(time(NULL));
		using namespace KDL;
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.075,0.0,0.345))));
		chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.27))));
		chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.106,0.0,0.09))));
		chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.189,0.0,0))));
		chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0,0,0))));
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0,0,0.145))));
		q_init = KDL::JntArray(6);
		q_init.data.setRandom();
		q_init.data *= M_PI;
		L(0) = 1;
		L(1) = 1;
		L(2) = 1;
		L(3) = 0.01;
		L(4) = 0.01;
		L(5) = 0.01;
	}

	public:void GraspPosePlanning(gazebo::math::Pose end_pose_cart)
	{
		// 先得到 target 的 Z axis
		math::Vector3 target_z_axis = end_pose_cart.rot.GetZAxis();
		// 得到 target_z_axis 與 world_z_axis 的 cross product
		math::Vector3 world_z_axis = math::Vector3(0,0,1);
		double dot = target_z_axis.Dot(world_z_axis);
		std::cout << "dot : " << dot << std::endl;
		// because target_z_axis and world_z_axis are already normalized, so we only need to calculate the dot product of these two vectors
		double result = acos(dot) * 180 / M_PI;
		std::cout << "result : " << result << std::endl;

		if((45 < result)&&(result < 135))
		{
			grasp_type = 0; // 從側面夾(水平)
		}
		else
		{
			grasp_type = 1; // 從上方夾
		}

	}

	public:int Calculate_beta(gazebo::math::Pose start_pose_cart,gazebo::math::Pose end_pose_cart,transport::PublisherPtr &publisher,float buffer_height=0.010 ,int num_point=100)
	{
	    using namespace KDL;
	    // comment那堆程式碼是從側面接近

	    // gazebo的pose轉成KDL的Frame
	    math::Matrix3 rot_start = start_pose_cart.rot.GetAsMatrix3();
	    KDL::Vector tmpvec(start_pose_cart.pos.x,start_pose_cart.pos.y,start_pose_cart.pos.z);
		// Frame start_pose_in_KDL(KDL::Rotation(rot_start[0][0],rot_start[0][1],rot_start[0][2],
		//                                          rot_start[1][0],rot_start[1][1],rot_start[1][2],
		//                                          rot_start[2][0],rot_start[2][1],rot_start[2][2]),tmpvec);
		// 簡單起見 pose都往下
	    //std::cout<<"start rot : "<<start_pose_cart.rot<<std::endl;
	    Frame start_pose_in_KDL(KDL::Rotation(1,0,0,
	    										0,-1,0,
												0,0,-1),tmpvec);

	    //std::cout<<"start xyz : "<<tmpvec<<std::endl;
	    //std::cout<<"start pos\n"<<start_pose_in_KDL;


	    // 夾爪的抓取rot設得跟工件一樣
		math::Vector3 tmpx = end_pose_cart.rot.GetZAxis();
		if(tmpx.z < 0){ tmpx = -tmpx; }
		math::Vector3 tmpy = tmpx.Cross(math::Vector3(0,0,1));
		math::Vector3 tmpz = tmpx.Cross(tmpy);
		math::Vector3 x = tmpx.Normalize();
		math::Vector3 y = tmpy.Normalize();
		math::Vector3 z = tmpz.Normalize();
		// 希望路徑點一致 別跟初始pose差太多 ex 3.14->-3.14

		double roll,pitch,yaw;
		KDL::Rotation tmpr(x.x,y.x,z.x,
						   x.y,y.y,z.y,
						   x.z,y.z,z.z);
		tmpr.GetRPY(roll,pitch,yaw);
		math::Quaternion aligner(roll,pitch,yaw);
		//math::Matrix3 rot_end=start_pose_cart.rot.GetAsMatrix3();
		//std::cout<<"buffer height: "<<math::Vector3(0,0,-1*buffer_height)<<std::endl;
		math::Vector3 tmp_pos = end_pose_cart.pos + aligner.RotateVector(math::Vector3(0, 0, -1*buffer_height));

		tmpvec = KDL::Vector(tmp_pos.x,tmp_pos.y,tmp_pos.z);
		Frame end_pose_in_KDL(KDL::Rotation(x.x,y.x,z.x,
											x.y,y.y,z.y,
											x.z,y.z,z.z), tmpvec);

		//std::cout<<"end xyz : "<<tmpvec<<std::endl;

		Path_RoundedComposite* path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());

		//正上方的的frame
		KDL::Frame rightup(
							KDL::Rotation(
								end_pose_in_KDL.M.UnitX(),
								end_pose_in_KDL.M.UnitY(),
								end_pose_in_KDL.M.UnitZ()),
							KDL::Vector(
								end_pose_in_KDL.p.data[0],
								end_pose_in_KDL.p.data[1],
								end_pose_in_KDL.p.data[2]+0.15));

		//設定要經過的frame
		path->Add(end_pose_in_KDL);
		path->Add(rightup);
		path->Add(start_pose_in_KDL);
		path->Finish();

		cout << end_pose_in_KDL << endl;
		cout << rightup << endl;
		cout << start_pose_in_KDL << endl;
/*
		stringstream ss;
		msgs::Request take_pic_request;
		take_pic_request.set_id( 0 );
		for(int y=0;y<3;y++)
		{
			for(int x=0;x<3;x++)
			{
				ss.clear();
				ss.str("");
				ss<<end_pose_in_KDL.M.data[y*3+x]<<"\n";
				take_pic_request.set_request(ss.str());
				publisher->Publish( take_pic_request );
				common::Time::MSleep(10);
			}

		}

		for(int x=0;x<3;x++)
		{
			ss.clear();
			ss.str("");
			ss<<end_pose_in_KDL.p.data[x]<<"\n";
			take_pic_request.set_request(ss.str());
			publisher->Publish( take_pic_request );
			common::Time::MSleep(10);
		}
		/////////////////////////////////////////////////

		for(int y=0;y<3;y++)
		{
			for(int x=0;x<3;x++)
			{
				ss.clear();
				ss.str("");
				ss<<rightup.M.data[y*3+x]<<"\n";
				take_pic_request.set_request(ss.str());
				publisher->Publish( take_pic_request );
				common::Time::MSleep(10);
			}

		}
		for(int x=0;x<3;x++)
		{
			ss.clear();
			ss.str("");
			ss<<rightup.p.data[x]<<"\n";
			take_pic_request.set_request(ss.str());
			publisher->Publish( take_pic_request );
			common::Time::MSleep(10);
		}
		/////////////////////////////////////////////////////////
		for(int y=0;y<3;y++)
		{
			for(int x=0;x<3;x++)
			{
				ss.clear();
				ss.str("");
				ss<<start_pose_in_KDL.M.data[y*3+x]<<"\n";
				take_pic_request.set_request(ss.str());
				publisher->Publish( take_pic_request );
				common::Time::MSleep(10);
			}

		}
		for(int x=0;x<3;x++)
		{
			ss.clear();
			ss.str("");
			ss<<start_pose_in_KDL.p.data[x]<<"\n";
			take_pic_request.set_request(ss.str());
			publisher->Publish( take_pic_request );
			common::Time::MSleep(10);
		}

*/

		//設置速度速度profile
		VelocityProfile* velpref = new VelocityProfile_Trap(0.05,0.2);
		velpref->SetProfile(0,path->PathLength());

		Trajectory* traject = new Trajectory_Segment(path, velpref);

		// use the trajectory
		int n = chain.getNrOfJoints();
		std::cout << "n : " << n << std::endl;
		ChainIkSolverPos_LMA solver(chain,L);
		JntArray q_sol(n);

		// 自己設initial condition (在HOME點)
		q_init.data(0,0) = (0);
		q_init.data(1,0) = (0.0);
		q_init.data(2,0) = (0.0);
		q_init.data(3,0) = 0;
		q_init.data(4,0) = 3.14;
		q_init.data(5,0) = 3.14;

		int retval=1;//,count=0;

		double dt=0.1;
		for (double t=0.0; t<=traject->Duration(); t+=dt)
		{
			retval = solver.CartToJnt(q_init,traject->Pos(t),q_sol);
			for(int q=0; q<6; q++)
			{
				if(q_sol.data(q,0) > M_PI)
				{
					q_sol.data(q,0) = q_sol.data(q,0) - 2*M_PI;
				}
				if((q_sol.data(q,0) + M_PI) < 0)
				{
					q_sol.data(q,0) = q_sol.data(q,0) + 2*M_PI;
				}
				result_ang_joint[q] = q_sol.data(q,0);
				//std::cout<<q_sol.data(q,0)<<std::endl;
			}
			//std::cout<<"exit loop\n";
			q_init = q_sol;//上個點的解 當新點的初始值

			Six_Ang each_result;
			each_result.push_back(result_ang_joint[0].Radian());
			each_result.push_back(result_ang_joint[1].Radian());
			each_result.push_back(result_ang_joint[2].Radian());
			each_result.push_back(result_ang_joint[3].Radian());
			each_result.push_back(result_ang_joint[4].Radian());
			each_result.push_back(result_ang_joint[5].Radian());
			ang_list.push_back(each_result);
		}

		//去除後面幾個變化大的
		ang_list.erase(ang_list.end()-2, ang_list.end());
		ang_list.erase(ang_list.begin(), ang_list.begin()+2);
		ang_list[0][3] = 0;
		ang_list[0][5] = 3.14;
		for(int i=1; i<ang_list.size()-1; i++)
		{
			if(ang_list[i][3] < (-3))
			{
				ang_list[i][3] = ang_list[i][3] + 2*M_PI;
			}
			if(ang_list[i][5] < 0)
			{
				ang_list[i][5] = ang_list[i][5] + 2*M_PI;
			}
			//std::cout<<ang_list[i];
		}
		ang_list.erase(ang_list.begin(),ang_list.begin());
		ang_list.erase(ang_list.end()-2,ang_list.end());

	    std::cout << COUT_PREFIX << "The trajectory has : " << ang_list.size() << " pts" << std::endl;
	    std::cout << COUT_PREFIX << "Calculate trajectory done!" << std::endl;
	}

	public:int Calculate_OnTable(gazebo::math::Pose start_pose_cart,gazebo::math::Pose end_pose_cart,float buffer_height=0.0 ,int num_point=100)
	{
		std::cout << "trajectoryPlanning::Calculate_OnTable" << std::endl;
		using namespace KDL;

		//從側面接近

		//gazebo的pose轉成KDL的Frame

	//    math::Matrix3 rot_start=start_pose_cart.rot.GetAsMatrix3();
	//    KDL::Vector tmpvec(start_pose_cart.pos.x,start_pose_cart.pos.y,start_pose_cart.pos.z);
	//    Frame start_pose_in_KDL(KDL::Rotation(rot_start[0][0],rot_start[0][1],rot_start[0][2],
	//                                          rot_start[1][0],rot_start[1][1],rot_start[1][2],
	//                                          rot_start[2][0],rot_start[2][1],rot_start[2][2]),
	//                            tmpvec);
	//
	//    //std::cout<<"start pos\n"<<start_pose_in_KDL;
	//
	//
	//    //夾爪的抓取rot設得跟工件一樣
	//    math::Vector3 tmpx=end_pose_cart.rot.GetZAxis();
	//    math::Vector3 tmpy=tmpx.Cross(math::Vector3(0,0,1));
	//    math::Vector3 tmpz=tmpx.Cross(tmpy);
	//    math::Vector3 x=tmpx.Normalize();
	//    math::Vector3 y=tmpy.Normalize();
	//    math::Vector3 z=tmpz.Normalize();
	//    double roll,pitch,yaw;
	//    KDL::Rotation tmpr(x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z);
	//    tmpr.GetRPY(roll,pitch,yaw);
	//    math::Quaternion aligner(roll,pitch,yaw);
	//    //math::Matrix3 rot_end=start_pose_cart.rot.GetAsMatrix3();
	//    math::Vector3 tmp_pos=end_pose_cart.pos+aligner.RotateVector(math::Vector3(0,0,-1*buffer_height));
	//
	//    tmpvec=KDL::Vector(tmp_pos.x,tmp_pos.y,tmp_pos.z);
	//    Frame end_pose_in_KDL(KDL::Rotation(x.x,y.x,z.x,
	//                                        x.y,y.y,z.y,
	//                                        x.z,y.z,z.z),
	//                            tmpvec);


	//
		Frame end_pose_in_KDL(
						KDL::Rotation::Quaternion(
							end_pose_cart.rot.x,
							end_pose_cart.rot.y,
							end_pose_cart.rot.z,
							end_pose_cart.rot.w),
						KDL::Vector(
							end_pose_cart.pos.x,
							end_pose_cart.pos.y,
							end_pose_cart.pos.z));

		// 延y軸轉180
		end_pose_in_KDL.M.data[0]*=-1;
		end_pose_in_KDL.M.data[3]*=-1;
		end_pose_in_KDL.M.data[6]*=-1;

		end_pose_in_KDL.M.data[2]*=-1;
		end_pose_in_KDL.M.data[5]*=-1;
		end_pose_in_KDL.M.data[8]*=-1;


		Frame start_pose_in_KDL(KDL::Rotation(1, 0, 0, 0, -1, 0, 0, 0, -1),
				KDL::Vector(start_pose_cart.pos.x,start_pose_cart.pos.y,start_pose_cart.pos.z));


	//    Frame start_pose_in_KDL(KDL::Rotation::Quaternion(
	//    					start_pose_cart.rot.x,
	//    					start_pose_cart.rot.y,
	//    					start_pose_cart.rot.z,
	//    					start_pose_cart.rot.w),
	//    				KDL::Vector(start_pose_cart.pos.x,
	//    						start_pose_cart.pos.y,
	//    						start_pose_cart.pos.z));

		//std::cout<<"\nend pose \n"<<end_pose_in_KDL;
		// 以上得到路徑的起點與終點
		// 接著使用kdl的路徑規劃

		Path_RoundedComposite* path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
		//設定要經過的frame
		path->Add(end_pose_in_KDL);
	//std::cout<<"  path->Add(KDL:\n";
		path->Add(KDL::Frame(
						KDL::Rotation(
							end_pose_in_KDL.M.UnitX(),
							end_pose_in_KDL.M.UnitY(),
							end_pose_in_KDL.M.UnitZ()),
						KDL::Vector(
							end_pose_in_KDL.p.data[0],
							end_pose_in_KDL.p.data[1],
							end_pose_in_KDL.p.data[2]+0.15))); // 正上方的Frame
		path->Add(start_pose_in_KDL);
		path->Finish();

		//設置速度速度profile
		VelocityProfile* velprof = new VelocityProfile_Trap(0.05,0.2);
		velprof->SetProfile(0,path->PathLength());

		Trajectory* traject = new Trajectory_Segment(path, velprof);

		// use the trajectory
		// 產生路徑後, 將各點解IK, 得到6軸, 存入ang_list
		int n = chain.getNrOfJoints();
		ChainIkSolverPos_LMA solver(chain,L);
		JntArray q_sol(n);
		//自己設initial condition
		q_init.data(0,0)=(0);
		q_init.data(1,0)=(0.157469);
		q_init.data(2,0)=(1.97743);
		q_init.data(3,0)=0.000160237;
		q_init.data(4,0)=(1.007);
		q_init.data(5,0)=0;

		int retval=1,count=0;
		double dt=0.1;
		for (double t=0.0; t <= traject->Duration(); t+=dt)
		{
			//std::cout<<"solve "<<t*10<<" th IK\n";
			count=0;
			while(retval||(q_sol.data(1,0)<-0.5*M_PI)||(q_sol.data(1,0)>0.5*M_PI))//不要大於1.57,超出這範圍,會撞到地
			{
				//std::cout<<++count;
				//std::cout<<count<<std::endl;
				retval = solver.CartToJnt(q_init,traject->Pos(t),q_sol);
				q_init.data.setRandom(); // 每次初始值都隨機
				q_init.data *= M_PI;
				if(count>10)
				{
					std::cout << "cannot not find IK solution\n\n";
					return 0;
				}
			}
			//std::cout<<q_sol.data<<"\n";
			//std::cout<<"\nstart ang:??\n";

			for(int q=0;q<6;q++)
			{
				//std::cout<<"..."<<std::endl;
				if(q_sol.data(q,0) > M_PI)
				{
					q_sol.data(q,0)=q_sol.data(q,0)-2*M_PI;
				}
				if((q_sol.data(q,0) + M_PI) < 0)
				{
					q_sol.data(q,0)=q_sol.data(q,0)+2*M_PI;
				}
				result_ang_joint[q]=q_sol.data(q,0);
				//std::cout<<q_sol.data(q,0)<<std::endl;
			}
			//std::cout<<"exit loop\n";
			q_init = q_sol; // 上個點的解 當新點的初始值
			q_sol.data(1,0) = 3; // 進入while的條件
			Six_Ang each_result;
			each_result.push_back(result_ang_joint[0].Radian());
			each_result.push_back(result_ang_joint[1].Radian());
			each_result.push_back(result_ang_joint[2].Radian());
			each_result.push_back(result_ang_joint[3].Radian());
			each_result.push_back(result_ang_joint[4].Radian());
			each_result.push_back(result_ang_joint[5].Radian());
			ang_list.push_back(each_result);
		}

		// 去除後面幾個變化大的
		for(int i=1;i<ang_list.size()-1;i++)
		{
			if((ang_list[i][3]-ang_list[i-1][3])*(ang_list[i][3]-ang_list[i-1][3])>0.04)
			{
				if(ang_list[i][3] > 3.13)
				{
					ang_list[i][3] = ang_list[i][3] - 2*M_PI;
				}
				else if(ang_list[i][3] < -3.13)
				{
					ang_list[i][3] = ang_list[i][3] + 2*M_PI;
				}
			}
			if(ang_list[i][5]<(math::Angle)0)
			{
				ang_list[i][5]=ang_list[i][5]+2*M_PI;
			}
			//std::cout<<ang_list[i];
		}
		ang_list.erase(ang_list.end()-1,ang_list.end());
	//    for(int i=0;i<ang_list.size();i++)
	//    	std::cout<<ang_list[i];

		std::cout << "\ttrajectory has : "<< ang_list.size() << " pts" << std::endl;
		std::cout << "Calculate_OnTable done!" << std::endl;
	}

	public:int Calculate_PutOnTable(gazebo::math::Pose start_pose_cart,gazebo::math::Pose end_pose_cart,float buffer_height=0.0 ,int num_point=100)
	{
		std::cout << "TrajectoryPlanning::Calculate_PutOnTable" << std::endl;
		//std::cout<<"\nstart pose(RAW) \n"<<start_pose_cart;
		//std::cout<<"\nend pose(RAW) \n"<<end_pose_cart;

		using namespace KDL;

	// 從側面接近
	// gazebo的pose轉成KDL的Frame
	//    math::Matrix3 rot_start=start_pose_cart.rot.GetAsMatrix3();
	//    KDL::Vector tmpvec(start_pose_cart.pos.x,start_pose_cart.pos.y,start_pose_cart.pos.z);
	//    Frame start_pose_in_KDL(KDL::Rotation(rot_start[0][0],rot_start[0][1],rot_start[0][2],
	//                                          rot_start[1][0],rot_start[1][1],rot_start[1][2],
	//                                          rot_start[2][0],rot_start[2][1],rot_start[2][2]),
	//                            tmpvec);
	//
	//    //std::cout<<"start pos\n"<<start_pose_in_KDL;
	//
	//
	//    //夾爪的抓取rot設得跟工件一樣
	//    math::Vector3 tmpx=end_pose_cart.rot.GetZAxis();
	//    math::Vector3 tmpy=tmpx.Cross(math::Vector3(0,0,1));
	//    math::Vector3 tmpz=tmpx.Cross(tmpy);
	//    math::Vector3 x=tmpx.Normalize();
	//    math::Vector3 y=tmpy.Normalize();
	//    math::Vector3 z=tmpz.Normalize();
	//    double roll,pitch,yaw;
	//    KDL::Rotation tmpr(x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z);
	//    tmpr.GetRPY(roll,pitch,yaw);
	//    math::Quaternion aligner(roll,pitch,yaw);
	//    //math::Matrix3 rot_end=start_pose_cart.rot.GetAsMatrix3();
	//    math::Vector3 tmp_pos=end_pose_cart.pos+aligner.RotateVector(math::Vector3(0,0,-1*buffer_height));
	//
	//    tmpvec=KDL::Vector(tmp_pos.x,tmp_pos.y,tmp_pos.z);
	//    Frame end_pose_in_KDL(KDL::Rotation(x.x,y.x,z.x,
	//                                        x.y,y.y,z.y,
	//                                        x.z,y.z,z.z),
	//                            tmpvec);

		Frame end_pose_in_KDL(
						KDL::Rotation::Quaternion(
							end_pose_cart.rot.x,
							end_pose_cart.rot.y,
							end_pose_cart.rot.z,
							end_pose_cart.rot.w),
						KDL::Vector(
							end_pose_cart.pos.x+0.5, // 傳過來時是世界座標,要給KDL的是手臂座標
							end_pose_cart.pos.y,
							end_pose_cart.pos.z+0.045)); // 0.045 在DHMODEL裡我是設夾爪內的小白點是tip(方便算夾取座標),避免夾爪頂部碰到版子,所以加個高度

		// 延 y 軸轉180
		end_pose_in_KDL.M.data[1]*=-1;
		end_pose_in_KDL.M.data[4]*=-1;
		end_pose_in_KDL.M.data[7]*=-1;

		end_pose_in_KDL.M.data[2]*=-1;
		end_pose_in_KDL.M.data[5]*=-1;
		end_pose_in_KDL.M.data[8]*=-1;


		math::Matrix3 rot_start=start_pose_cart.rot.GetAsMatrix3();
		//KDL::Vector tmpvec(start_pose_cart.pos.x,start_pose_cart.pos.y,start_pose_cart.pos.z);
		KDL::Vector tmpvec(start_pose_cart.pos.x+0.5,start_pose_cart.pos.y,start_pose_cart.pos.z);

	//    Frame start_pose_in_KDL(KDL::Rotation(rot_start[0][0],rot_start[0][1],rot_start[0][2],
	//                                          rot_start[1][0],rot_start[1][1],rot_start[1][2],
	//                                          rot_start[2][0],rot_start[2][1],rot_start[2][2]),tmpvec);
		// 簡單起見 pose都往下
		//std::cout<<"start rot : "<<start_pose_cart.rot<<std::endl;
		Frame start_pose_in_KDL(KDL::Rotation(1,0,0,
												0,-1,0,
												0,0,-1),tmpvec);





		std::cout << "\nstart pose(in AR605 Frame) \n" << start_pose_in_KDL;
		std::cout << "\nend pose(in AR605 Frame) \n" << end_pose_in_KDL;

		// 以上得到路徑的起點與終點
		// 接著使用kdl的路徑規劃

		Path_RoundedComposite* path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
		// 設定要經過的frame
		path->Add(end_pose_in_KDL);
		path->Add(KDL::Frame(
								KDL::Rotation(
									end_pose_in_KDL.M.UnitX(),
									end_pose_in_KDL.M.UnitY(),
									end_pose_in_KDL.M.UnitZ()),
								KDL::Vector(
										end_pose_cart.pos.x+0.5,
										end_pose_cart.pos.y,
										end_pose_cart.pos.z+0.2)));
		//std::cout<<"  path->Add(KDL:\n";
		path->Add(start_pose_in_KDL);
		path->Finish();

		// 設置速度速度profile
		VelocityProfile* velprof = new VelocityProfile_Trap(0.075,0.2);
		velprof->SetProfile(0,path->PathLength());

		Trajectory* traject = new Trajectory_Segment(path, velprof);

		// use the trajectory

		int n = chain.getNrOfJoints();
		ChainIkSolverPos_LMA solver(chain,L);
		JntArray q_sol(n);
		// 自己設initial condition
		q_init.data(0,0) = (0);
		q_init.data(1,0) = (0.157469);
		q_init.data(2,0) = (1.97743);
		q_init.data(3,0) = 0.000160237;
		q_init.data(4,0) = (1.007);
		q_init.data(5,0) = 0;

		int retval = 1, count = 0;

		double dt = 0.1;
		for (double t=0.0; t <= traject->Duration(); t+= dt)
		{
			//std::cout<<"solve "<<t*10<<" th IK\n";
			count=0;
			while(retval||(q_sol.data(1,0)<-0.5*M_PI)||(q_sol.data(1,0)>0.5*M_PI))//不要大於1.57,超出這範圍,會撞到地
			{
				++count;
				//std::cout<<count<<std::endl;
				retval = solver.CartToJnt(q_init,traject->Pos(t),q_sol);
				q_init.data.setRandom();//每次初始值都隨機
				q_init.data *= M_PI;
				if(count>5)
				{
					std::cout<<"cannot not find IK solution\n\n";
					return 0;
				}
			}
			//std::cout<<q_sol.data<<"\n";
			//std::cout<<"\nstart ang:??\n";

			for(int q=0;q<6;q++)
			{
				//std::cout<<"..."<<std::endl;
				if(q_sol.data(q,0)>M_PI)q_sol.data(q,0)=q_sol.data(q,0)-2*M_PI;
				if((q_sol.data(q,0)+M_PI)<0)q_sol.data(q,0)=q_sol.data(q,0)+2*M_PI;
				result_ang_joint[q]=q_sol.data(q,0);
				//std::cout<<q_sol.data(q,0)<<std::endl;
			}
			//std::cout<<"exit loop\n";

			q_init=q_sol;//上個點的解 當新點的初始值
			q_sol.data(1,0)=3;//進入while的條件


			Six_Ang each_result;
			each_result.push_back(result_ang_joint[0].Radian());
			each_result.push_back(result_ang_joint[1].Radian());
			each_result.push_back(result_ang_joint[2].Radian());
			each_result.push_back(result_ang_joint[3].Radian());
			each_result.push_back(result_ang_joint[4].Radian());
			each_result.push_back(result_ang_joint[5].Radian());
			ang_list.push_back(each_result);



		}
	//    for(int i=0;i<ang_list.size();i++)
	//    {
	//        for(int j=0;j<6;j++)
	//            std::cout<<ang_list[i][j]<<" ";
	//        std::cout<<std::endl;
	//    }
		std::cout << ang_list.size() << " pts" << std::endl;


		for(int i=0;i<ang_list.size();i++)
		{
			ang_list[i][5] = ang_list[i][5]+2*M_PI;
			//std::cout<<ang_list[i];
		}




	//    for(int i=0;i<6;i++)
	//    {
	//        vel[i]=0.2;
	//    }
	//    std::cout<<std::endl;
		std::cout << "Calculate_PutOnTable done!" << std::endl;
	}



	public:void Calculate(gazebo::math::Pose start_pose_cart,gazebo::math::Pose end_pose_cart,float buffer_height=1.25,int num_point=100)
	{
	//    double vel[6]={0.2,0.498,0.18,0.2,0.5255,2};
	//
	//    Six_Ang tmp;
	//    tmp.push_back(0);
	//    tmp.push_back(-0.182);
	//    tmp.push_back(-1.992);
	//    tmp.push_back(0);
	//    tmp.push_back(-0.9675);
	//    tmp.push_back(3.14);
	//    ang_list.push_back(tmp);
	//    tmp.clear();
	//    tmp.push_back(0);
	//    tmp.push_back(-0.69);
	//    tmp.push_back(-2.15);
	//    tmp.push_back(0);
	//    tmp.push_back(-0.455);
	//    tmp.push_back(3.14);
	//
	//
	//    ang_list.push_back(tmp);
	//    for(int i=1;i<11;i++)
	//    {
	//        tmp.clear();
	//        tmp.push_back(0     +i*0);
	//        tmp.push_back(-0.182-i*0.0508);
	//        tmp.push_back(-1.992-i*0.0018);
	//        tmp.push_back(0     +i*0);
	//        tmp.push_back(-0.9675+i*0.05255);
	//        tmp.push_back(3.14  +i*0);
	//
	//        ang_list.insert(ang_list.end()-1,tmp);
	//    }
	//    for(int i=1;i<11;i++)
	//    {
	//        std::cout<<ang_list[i][0]<<" "<<
	//                    ang_list[i][1]<<" "<<
	//                    ang_list[i][2]<<" "<<
	//                    ang_list[i][3]<<" "<<
	//                    ang_list[i][4]<<" "<<
	//                    ang_list[i][5]<<" "<<
	//                    "\n------------------"<<
	//                std::endl;
	//    }
	//
		using namespace KDL;

		// gazebo 的 pose 轉成 KDL 的 Frame

		math::Matrix3 rot_start=start_pose_cart.rot.GetAsMatrix3();
		KDL::Vector tmpvec(start_pose_cart.pos.x,start_pose_cart.pos.y,start_pose_cart.pos.z);
		Frame start_pose_in_KDL(KDL::Rotation(rot_start[0][0],rot_start[0][1],rot_start[0][2],
											  rot_start[1][0],rot_start[1][1],rot_start[1][2],
											  rot_start[2][0],rot_start[2][1],rot_start[2][2]), tmpvec);

		std::cout << "start pos\n" << start_pose_in_KDL;
	//    math::Matrix3 rot_end=start_pose_cart.rot.GetAsMatrix3();
	//    tmpvec=KDL::Vector(end_pose_cart.pos.x,end_pose_cart.pos.y,end_pose_cart.pos.z);
	//    Frame end_pose_in_KDL(KDL::Rotation(rot_end[0][0],rot_end[0][1],rot_end[0][2],
	//                                        rot_end[1][0],rot_end[1][1],rot_end[1][2],
	//                                        rot_end[2][0],rot_end[2][1],rot_end[2][2]),
	//                            tmpvec);




		//夾爪的抓取rot設得跟工件一樣
		math::Vector3 tmpx = end_pose_cart.rot.GetZAxis();
		math::Vector3 tmpy = tmpx.Cross(math::Vector3(0,0,1));
		math::Vector3 tmpz = tmpx.Cross(tmpy);
		math::Vector3 x = tmpx.Normalize();
		math::Vector3 y = tmpy.Normalize();
		math::Vector3 z = tmpz.Normalize();
		double roll,pitch,yaw;
		KDL::Rotation tmpr(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z);
		tmpr.GetRPY(roll,pitch,yaw);
		math::Quaternion aligner(roll,pitch,yaw);
		//math::Matrix3 rot_end=start_pose_cart.rot.GetAsMatrix3();
		math::Vector3 tmp_pos = end_pose_cart.pos+aligner.RotateVector(math::Vector3(0,0,-1*buffer_height));

		tmpvec=KDL::Vector(tmp_pos.x,tmp_pos.y,tmp_pos.z);
		Frame end_pose_in_KDL(KDL::Rotation(x.x,y.x,z.x,
											x.y,y.y,z.y,
											x.z,y.z,z.z),
								tmpvec);

		std::cout<<"\nend pose \n"<<end_pose_in_KDL;
		int n = chain.getNrOfJoints();
		ChainIkSolverPos_LMA solver(chain,L);
		JntArray q_sol(n);
		//自己設initial condition


		q_init.data(0,0)=(0);
		q_init.data(1,0)=(0.157469);
		q_init.data(2,0)=(1.97743);
		q_init.data(3,0)=0.000160237;
		q_init.data(4,0)=(1.007);
		q_init.data(5,0)=0;


		int retval=1,count=0;


	//    while(check_if_too_large<-1||check_if_too_large>1)
	//    {
			while(retval||(q_sol.data(1,0)<-0.5*M_PI)||(q_sol.data(1,0)>0.5*M_PI))//不要大於1.57,超出這範圍,會撞到地
			{

				retval = solver.CartToJnt(q_init,start_pose_in_KDL,q_sol);
				count++;
				q_init.data.setRandom();//每次初始值都隨機
				q_init.data *= M_PI;


			}
			//std::cout<<q_sol.data<<"\n";
			std::cout<<"\nstart ang:\n";
			for(int i=0;i<6;i++)
			{
				if(q_sol.data(i,0)>M_PI)q_sol.data(i,0)=q_sol.data(i,0)-2*M_PI;
				if((q_sol.data(i,0)+M_PI)<0)q_sol.data(i,0)=q_sol.data(i,0)+2*M_PI;
				start_ang_joint[i]=q_sol.data(i,0);
				std::cout<<start_ang_joint[i]<<std::endl;
			}


			q_init=q_sol;//上個點的解 當新點的初始值
			q_sol.data(1,0)=3;//進入while的條件
			while(retval||(q_sol.data(1,0)<-0.5*M_PI)||(q_sol.data(1,0)>0.5*M_PI))//不要大於1.57,超出這範圍,會撞到地
			{
				retval = solver.CartToJnt(q_init,end_pose_in_KDL,q_sol);
				count++;
				q_init.data.setRandom();//每次初始值都隨機
				q_init.data *= M_PI;

			}

			std::cout<<"\nend ang:\n";
			for(int i=0;i<6;i++)
			{
				if(q_sol.data(i,0)>M_PI)q_sol.data(i,0)=q_sol.data(i,0)-2*M_PI;
				if((q_sol.data(i,0)+M_PI)<0)q_sol.data(i,0)=q_sol.data(i,0)+2*M_PI;
				end_ang_joint[i]=q_sol.data(i,0);
				std::cout<<end_ang_joint[i]<<std::endl;
			}
	//        check_if_too_large=end_ang_joint[0].Radian()-start_ang_joint[0].Radian();
	//    }



		for(int i=0;i<6;i++)
		{
			vel[i]=(end_ang_joint[i].Radian()-start_ang_joint[i].Radian());

		}


		Six_Ang tmp;

		tmp.push_back(start_ang_joint[0].Radian());
		tmp.push_back(start_ang_joint[1].Radian());
		tmp.push_back(start_ang_joint[2].Radian());
		tmp.push_back(start_ang_joint[3].Radian());
		tmp.push_back(start_ang_joint[4].Radian());
		tmp.push_back(start_ang_joint[5].Radian());
		ang_list.push_back(tmp);
		tmp.clear();
		tmp.push_back(end_ang_joint[0].Radian());
		tmp.push_back(end_ang_joint[1].Radian());
		tmp.push_back(end_ang_joint[2].Radian());
		tmp.push_back(end_ang_joint[3].Radian());
		tmp.push_back(end_ang_joint[4].Radian());
		tmp.push_back(end_ang_joint[5].Radian());
		ang_list.push_back(tmp);
		double step=1.0/num_point;
		std::cout<<"step"<<step<<std::endl;
		for(int i=1;i<num_point;i++)
		{
			tmp.clear();
			tmp.push_back(math::Angle(start_ang_joint[0].Radian()+step*vel[0]*i));
			tmp.push_back(math::Angle(start_ang_joint[1].Radian()+step*vel[1]*i));
			tmp.push_back(math::Angle(start_ang_joint[2].Radian()+step*vel[2]*i));
			tmp.push_back(math::Angle(start_ang_joint[3].Radian()+step*vel[3]*i));
			tmp.push_back(math::Angle(start_ang_joint[4].Radian()+step*vel[4]*i));
			tmp.push_back(math::Angle(start_ang_joint[5].Radian()+step*vel[5]*i));

			ang_list.insert(ang_list.end()-1,tmp);
		}
	//        for(int i=1;i<ang_list.size();i++)
	//    {
	//        std::cout<<ang_list[i][0]<<" "<<
	//                    ang_list[i][1]<<" "<<
	//                    ang_list[i][2]<<" "<<
	//                    ang_list[i][3]<<" "<<
	//                    ang_list[i][4]<<" "<<
	//                    ang_list[i][5]<<" "<<
	//                    "\n------------------"<<
	//                std::endl;
	//    }

		for(int i=0;i<6;i++)
		{
			std::cout<<vel[i]<<" ";
		}
		std::cout<<std::endl;
	}

	public:double* GetJointVel()
	{
		return vel;
	}

	public:Ang_List_V GetAngList()
	{
		return ang_list;
	}

}; // end Class TrajectoryPlanning


typedef const boost::shared_ptr<const my::msgs::PoseEstimationResult > ConstMsgsPoseEstimationResultPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Request > ConstMsgsRequestPtr;

// 沿著設定好的路線走,每個iteration,都送出一點
class AlongTrajectory
{
	private:
		physics::ModelPtr model;
		physics::WorldPtr world;
		Ang_List_V ang_list; // 這個 list 就是存放一連串要經過的點
		int nth_step; // 紀錄目前走到第幾個點
		MoveToPtr movetoptr;
		std::string targetname;

	public:AlongTrajectory()
	{
		nth_step = 0;
	}

	public:void SetNearestModel(std::string param_name) // Class alongTrajectory
	{
		targetname = param_name;
	}

	public:AlongTrajectory(MoveToPtr param_movetoptr,physics::ModelPtr param_model)
	{
		movetoptr = param_movetoptr;
		model = param_model;
		world = model->GetWorld();
		nth_step = 0;
	}

	public:void SetAngList(Ang_List_V param_ang_list) // 設定路徑點清單
	{
		ang_list.assign(param_ang_list.begin(),param_ang_list.end());
	}

	public:void Resetnth_step() // 路徑點清單index歸0
	{
		nth_step = 0;
		movetoptr->ResetReached();
	}

	public:void CatchObjectOnTable() // 僅僅計算需要經過的點位，如果要跑的話需要由 update 來跑
	{
		std::cout << "approach\n";
		TrajectoryPlanning tra;
		// 投機作法這邊要改
		math::Pose targetpose;

		int whichitem = 0;
		std::cout << "which item?\n";
		scanf("%d",&whichitem);
		std::cin.ignore();
		targetpose = world->GetModel(whichitem)->GetWorldPose() - math::Pose(math::Vector3(-0.5,0,0),math::Quaternion(0,0,0));
		//因為機器手臂原點不在0 0 0 所以要shift
		math::Pose tipppose = movetoptr->GetTipPose() - math::Pose(math::Vector3(-0.5,0,0),math::Quaternion(0,0,0));

		tra.Calculate_OnTable(tipppose,targetpose);

		std::cout << "exit tra.Calculate\n";
		ang_list = tra.GetAngList();
		std::reverse(ang_list.begin(),ang_list.end());
	//    double *tmpvel ;//這部份似乎有問題
	//    tmpvel=tra.GetJointVel();
	//    for(int i=0;i<6;i++)
	//    {
	//        if(tmpvel[i]<0)
	//            tmpvel[i]=tmpvel[i]*-1;
	//        tmpvel[i]=tmpvel[i]*5;
	//        if(tmpvel[i]<1)
	//            tmpvel[i]=.5;
	//    }
	//    movetoptr->SetVelocityOfJoint(tmpvel);

	}

	public:void CatchObjectOnTableFromMsg() // 僅僅計算需要經過的點位，如果要跑的話需要由 update 來跑
	{
		std::cout << "approach\n";
		TrajectoryPlanning tra;
		math::Pose targetpose;
		// 因為機器手臂原點不在 (0 0 0) 所以要 shift
		targetpose = world->GetModel(targetname)->GetWorldPose() - math::Pose(math::Vector3(-0.5,0,0),math::Quaternion(0,0,0));
		math::Pose tipppose = movetoptr->GetTipPose() - math::Pose(math::Vector3(-0.5,0,0),math::Quaternion(0,0,0));

		tra.Calculate_OnTable( tipppose,targetpose );

		std::cout << "Calculate_OnTable done!\n";
		ang_list = tra.GetAngList();
		std::reverse(ang_list.begin(),ang_list.end());
	}

	public:void PutOnTable() // Class AlongTrajectory
	{
		// 終點index
		int position_num = 1;
		std::cin >> position_num;
		//一連串終點位置
		float destination[27]={
					-0.5,0.5,0.123684,
					-0.4,0.5,0.123684,
					-0.3,0.5,0.123684,

					-0.5,0.4,0.123684,
					-0.4,0.4,0.123684,
					-0.3,0.4,0.123684,

					-0.5,0.3,0.123684,
					-0.4,0.3,0.123684,
					-0.3,0.3,0.123684};


		std::cout << "AlongTrajectory::PutOnTable" << std::endl;;
		TrajectoryPlanning tra;
		math::Pose targetpose;

		targetpose = math::Pose(
				destination[position_num*3-3],
				destination[position_num*3-2],
				destination[position_num*3-1],
				0,0,0);

		//傳過去的是世界座標
		tra.Calculate_PutOnTable( movetoptr->GetTipPose(), targetpose );
				//math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
		std::cout << "exit tra.Calculate_PutOnTable" << std::endl;
		ang_list = tra.GetAngList();
		std::reverse(ang_list.begin(),ang_list.end());
	}

	public:void CatchObjectInBoxFromMsg()
	{
		// 僅僅計算需要經過的點位，如果要跑的話需要由 update 來跑
		std::cout << "CatchObjectInBoxFromMsg \n";
		std::cout << "receive target model form Evaluation Platform\n\n";
		std::cout << "target: " << targetname << std::endl;
		TrajectoryPlanning tra;
		math::Pose targetpose;
		// 因為機器手臂原點不在 (0, 0, 0) 所以要 shift
		targetpose = world->GetModel(targetname)->GetWorldPose() - math::Pose(math::Vector3(-0.5,0,0),math::Quaternion(0,0,0));
		math::Pose tippose = movetoptr->GetTipPose() - math::Pose(math::Vector3(-0.5,0,0),math::Quaternion(0,0,0));
		transport::PublisherPtr p;
		tra.Calculate_beta( tippose, targetpose, p );
		std::cout << "Calculating trajectory done!\n";
		ang_list = tra.GetAngList();

		std::reverse(ang_list.begin(),ang_list.end());

		for(int i=0;i<4;i++)
		{
			std::cout << ang_list[i];
		}
	//    double *tmpvel ;//這部份似乎有問題
	//    tmpvel=tra.GetJointVel();
	//    for(int i=0;i<6;i++)
	//    {
	//        if(tmpvel[i]<0)
	//            tmpvel[i]=tmpvel[i]*-1;
	//        tmpvel[i]=tmpvel[i]*5;
	//        if(tmpvel[i]<1)
	//            tmpvel[i]=.5;
	//    }
	//    movetoptr->SetVelocityOfJoint(tmpvel);

	}

	public:void CatchObjectInBox(transport::PublisherPtr &publisher)
	{
		//std::cout<<"approach\n";
		TrajectoryPlanning tra;
		// 投機作法這邊要改
		math::Pose targetpose;
		// 因為機器手臂原點不在0 0 0 所以要shift
		int whichitem = 0;
		std::cout << "which item?\n";
		scanf("%d",&whichitem);
		std::cin.ignore();
		targetpose = world->GetModel(whichitem)->GetWorldPose() - math::Pose(math::Vector3(-0.5,0,0),math::Quaternion(0,0,0));
		math::Pose tipppose = movetoptr->GetTipPose() - math::Pose(math::Vector3(-0.5,0,0),math::Quaternion(0,0,0));
		tra.Calculate_beta(tipppose,targetpose,publisher);
		std::cout << "exit tra.Calculate\n";
		ang_list = tra.GetAngList();
		std::reverse(ang_list.begin(),ang_list.end());
	}

	public:void Up()
	{
		//0.8  0.157  2.5  2.5  1.15  0.2
		std::reverse(ang_list.begin(), ang_list.end());
		return;
	}

	public:void PutDown()
	{
		// 0.8  0.157  2.5  2.5  1.15  0.2
		std::cout << "PutDown\n";
		TrajectoryPlanning tra;
		int whichitem = 0;
		std::cout << "which item?\n";
		scanf("%d",&whichitem);
		std::cin.ignore();

		tra.Calculate(movetoptr->GetTipPose(),
						math::Pose(math::Vector3(
											world->GetModel(whichitem)->GetWorldPose().pos.x,
											world->GetModel(whichitem)->GetWorldPose().pos.y,
											world->GetModel(whichitem)->GetWorldPose().pos.z),
											math::Quaternion(0,0,0)),0,3);
				//math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
		std::cout<<"exit tra.Calculate\n";
		ang_list=tra.GetAngList();
	}

	public:void Pick()
	{
		//0.8  0.157  2.5  2.5  1.15  0.2
		std::cout << "PickU\n";
		TrajectoryPlanning tra;
		int whichitem = 0;
		std::cout << "which item?\n";
		scanf("%d",&whichitem);
		std::cin.ignore();
		tra.Calculate(movetoptr->GetTipPose(), world->GetModel(whichitem)->GetWorldPose(), 0.25, 10);
				//math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
		std::cout << "exit tra.Calculate\n";
		ang_list = tra.GetAngList();
	}

	public:void Update() // Class AlongTrajectory
	{
		if( movetoptr->HasReached() && nth_step!=(ang_list.size()-1) )
		{
			movetoptr->ResetReached();
			std::cout << "\tafter movetoptr->ResetReached()" << std::endl;
			std::cout << "\tnth_step : " << nth_step << std::endl;
			std::cout << "\tang_list.size() : " << ang_list.size() << std::endl;

			//std::cout << "angle_list" << std::endl;
			//for(int i=0;i<6;i++)
			//{
			//	std::cout << i+1 << ", " << ang_list[i] << std::endl;
			//}

			//std::cout << "\t" << ang_list[nth_step+1] << std::endl;

			movetoptr->SetDestination(ang_list[nth_step]);
			// 避免超出範圍
			// 如果 ang_list.size()=10, 在 nth_step=10 還可以進入IF, IF內再加1, 下次再進入IF就會超出大小
			if( nth_step < (ang_list.size()-1) )
				nth_step++;
			// cout << nth_step << " ";
			// cout << ang_list[nth_step];

			// 為了讓他只顯示一次
			if( nth_step == (ang_list.size()-1) )
			{
				std::cout << "reach the destination" << std::endl;
			}
		}

		movetoptr->Update();
	}

}; // end class AlongTrajectorys

class Gripper
{
	private:
		int grip;
		physics::ModelPtr model;
		physics::WorldPtr world;
		physics::ModelPtr target_for_cheat_grip;

	public:Gripper(){} // class Gripper

	public:void SetCheatTarget(physics::ModelPtr param_model) // class Gripper
	{
		target_for_cheat_grip = param_model;
	}

	public:Gripper(physics::ModelPtr param_model) // class Gripper
	{
		grip = 0;
		model = param_model;
		world = model->GetWorld();
	}

	public:void Grip() // connect to update event, grip = 1 -> grip, grip = 0 -> release
	{
		grip = 1;
		// 因為 Gazebo 沒辦法去夾取，所以把物體黏在grip中間的小球
		world->GetModel("result_visualize_socket")->SetWorldPose(math::Pose(10,10,10,0,0,0));
		target_for_cheat_grip->SetEnabled( true );
		target_for_cheat_grip->SetStatic( false );
		model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),target_for_cheat_grip->GetLink("link"));
		return;
	}

	public:void Grip_cheat() // class Gripper
	{
		grip = 1;

		world->GetModel("result_visualize_socket")->SetWorldPose(math::Pose(10,10,10,0,0,0));
		target_for_cheat_grip->SetEnabled( true );
		target_for_cheat_grip->SetStatic( false );
		model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),target_for_cheat_grip->GetLink("link"));
		//   下面是單一model時debug用
		//    world->GetModel("socket")->SetEnabled( true );
		//    world->GetModel("socket")->SetStatic( false );
		//    model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),world->GetModel("socket")->GetLink("link"));
	}

	public:void Release_cheat() // class Gripper
	{
		grip = 0;

		math::Pose dummypose;
		math::Pose plampose = model->GetLink("gripper::plam")->GetWorldPose();
		dummypose = plampose+math::Pose(plampose.rot.RotateVector(math::Vector3(0,0,-0.05)),math::Quaternion(0,0,0));

		model->GetLink("gripper::dummy_for_glue")->SetWorldPose(dummypose);

		target_for_cheat_grip->SetEnabled( true );
		target_for_cheat_grip->SetStatic( false );
		//下面是單一model時debug用
		//   world->GetModel("socket")->SetEnabled( true );
		//	world->GetModel("socket")->SetStatic( false );

		 model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),model->GetLink("gripper::dummy_for_glue"));
	}

	public:void Release() // class Gripper
	{
		grip = 0;

		math::Pose dummypose;
		math::Pose plampose = model->GetLink("gripper::plam")->GetWorldPose();
		dummypose = plampose+math::Pose(plampose.rot.RotateVector(math::Vector3(0,0,-0.05)),math::Quaternion(0,0,0));

		model->GetLink("gripper::dummy_for_glue")->SetWorldPose(dummypose);

		target_for_cheat_grip->SetEnabled( true );
		target_for_cheat_grip->SetStatic( false );

		model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),model->GetLink("gripper::dummy_for_glue"));

		return;
	}

	public:int IsGrip() // class Gripper
	{
		return grip;
	}

	public:void Update() // update gripper motion
	{
		if( grip == 0 )
		{
			model->GetLink("gripper::right_finger")->AddRelativeForce(math::Vector3(0,6,0)); // red one
			model->GetLink("gripper::left_finger")->AddRelativeForce(math::Vector3(0,-6,0)); // blue one
			// math::Vector3(0,5,0) is too small for the gripper to move
		}
		if( grip == 1 )
		{
			model->GetLink("gripper::right_finger")->AddRelativeForce(math::Vector3(0,-6,0)); // red one
			model->GetLink("gripper::left_finger")->AddRelativeForce(math::Vector3(0,6,0)); // blue one
	//        model->GetJoint("gripper::right_joint")->SetHighStop(1,math::Angle(-.625));
	//        model->GetJoint("gripper::right_joint")->SetLowStop(1,math::Angle(-.625));
	//        model->GetJoint("gripper::left_joint")->SetHighStop(1,math::Angle(.625));
	//        model->GetJoint("gripper::left_joint")->SetLowStop(1,math::Angle(.625));
		}
	}

}; // end class Gripper

class KDL_IK
{
	private:
		MoveToPtr movetoptr;
		KDL::Chain chain;
		KDL::JntArray q_init;
		Eigen::Matrix<double,6,1> L;
		Six_Ang ang;

	public:KDL_IK(){} // class KDL_IK

	public:KDL_IK(MoveToPtr param_movetoptr) // class KDL_IK
	{
		movetoptr = param_movetoptr;
		movetoptr->SetDestination(Six_Ang(6,0));
		using namespace KDL;

		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.075,0.0,0.345))));	// (0,0.075,0.345)
		chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.27))));		// (0.0,0.0,0.27)
		chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.106,0.0,0.09))));	// (0,0.106,0.09)
		chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.189,0.0,0))));		// (0,0.189,0)
		chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0,0,0))));				// (0,0,0)
		chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0,0,0.145))));			// (0,0.145,0)

		ang = Six_Ang(6,0);
		q_init = KDL::JntArray(6);
		q_init.data.setRandom();
		q_init.data *= M_PI;
		movetoptr->SetReached(); // 先設為reached,否則不是reached的話,他就會開始走
	}

	public:int CalculateIK(float x,float y ,float z) // class KDL_IK
	{
		using namespace KDL;

		int n = chain.getNrOfJoints();
		Eigen::Matrix< double,6,1 > L; // L is a 6X1 matrix
		L(0) = 1;
		L(1) = 1;
		L(2) = 1;
		L(3) = 0.01;
		L(4) = 0.01;

		ChainIkSolverPos_LMA solver( chain,L );
		JntArray q_sol( n );
		//q_sol.data(1,0) = 3; // q_sol array 的第二個元素設成 3 // 進入while的條件

		int retval;//,count=0;
		//夾爪方向往下
		Frame goal_pos;
		//  while( retval || (q_sol.data(1,0) < -1.57) || (q_sol.data(1,0) > 1.57)) // 不要大於1.57,超出這範圍,會撞到地
		//  {
		//	std::cout<<"fgfrg \n  "<<count++;
		//	if(count>10)
		//	{
		//		return 0;
		//	}
		// 每次初始值都隨機
		// q_init.data.setRandom();
		// q_init.data *= M_PI;
		Six_Ang ang_for_init = movetoptr->GetJointAngs(); // initial: get angles of each joint
		q_init.data(0,0) = ang_for_init[0].Radian();
		q_init.data(1,0) = ang_for_init[1].Radian();
		q_init.data(2,0) = ang_for_init[2].Radian();
		q_init.data(3,0) = ang_for_init[3].Radian();
		q_init.data(4,0) = ang_for_init[4].Radian();
		q_init.data(5,0) = ang_for_init[5].Radian();

		// 以目前位置當作初始值
		// 按照這隻手臂的初始狀態來看，夾爪所指方向是 y 方向 ??????????????????????????????
		goal_pos = Frame(KDL::Rotation(-1, 0, 0, 0, 1, 0, 0, 0, -1), KDL::Vector(x,y,z));

		/*	Function CartToJnt
		 *
		 *	Parameters:
		 *		q_init		initial joint position.
		 *		T_base_goal	goal position expressed with respect to the robot base.
		 *		q_out		joint position that achieves the specified goal position (if successful).
		 *	Returns:
		 *		 0 			if successful
		 *		-1 			the gradient of $ E $ towards the joints is too small
		 *		-2			if joint position increments are too small
		 *		-3			if number of iterations is exceeded.
		 */
		retval = solver.CartToJnt( q_init, goal_pos, q_sol );
		std::cout << COUT_PREFIX << "IK solver return value : " << retval << std::endl;
		// }

		// std::cout << "q_sol (degree) : " << (q_sol.data*360/M_PI) << std::endl;
		std::cout << "q_sol : " << std::endl;
		std::cout << q_sol.data << std::endl;
		for(int i=0;i<6;i++)
		{
			if( q_sol.data(i,0) > M_PI )
			{
				q_sol.data(i,0) = q_sol.data(i,0) - 2*M_PI;
			}
			if(( q_sol.data(i,0) + M_PI ) < 0)
			{
				q_sol.data(i,0) = q_sol.data(i,0) + 2*M_PI;
			}
			ang[i] = q_sol.data(i,0);
		}

		movetoptr->ResetReached();
		movetoptr->SetDestination(ang);
	}
}; // end class KDL_IK

class Arm_Control : public ModelPlugin
{
	public:~Arm_Control()
	{
	}

	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)
	{
		this->model = _parent;
		world = model->GetWorld();

		// model->SetGravityMode(0);

		// open a thread to accept input
		_Thread = boost::thread( &Arm_Control::Keyboard_Input, this );

		// initial gazebo node
		node = transport::NodePtr( new transport::Node() );
		node->Init(world->GetName());


		//subscribe=node->Subscribe("~/pose_estimation/estimate_result",&Arm_Control::receivedpose,this);//還是不知道還是3個參數是幹啥小用的

		subscribe_GGininder = node->Subscribe( "~/evaluation_platform/GGininder", &Arm_Control::receivedGGininder, this );//還是不知道還是3個參數是幹啥小用的
		subscribe_GotIt = node->Subscribe( "~/evaluation_platform/take_picture_request", &Arm_Control::receivedGotIt, this);

		publisher = node->Advertise< gazebo::msgs::Request >( "~/trajectory_point" );

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Arm_Control::OnUpdate, this, _1));
	}

	public:void Keyboard_Input()
	{
		std::cout << COUT_PREFIX << "press [Enter] to Menu" << std::endl;
		std::cout << COUT_PREFIX << "waiting for input...." << std::endl;
		while(1)
		{
			if(std::cin.get() == '\n') // 按下enter,停止目前動作
			{
				std::cout << COUT_PREFIX << "Menu:\n"
						<< "\t0\t release\n"
						<< "\t1\t grip\n"
						<< "\t2\t Cartesian coordinate (m)\n"
						<< "\t3\t to catch\n"
						<< "\t4\t list all models\n"
						<< "\t5\t default position (HOME)\n"
						<< "\t6\t print pose of the end-effector\n"
						<< "\t7\t tune angle of one specified joint\n"
						<< "\t8\t from default position to target\n"
						<< "\t9\t put on the small table\n"
						<< "\t10\t stop\n"
						<< "\t11\t grip_cheat\n"
						<< "\t12\t release_cheat\n";

						scanf("%d", &userinput);
						std::cin.ignore();

						switch( userinput )
						{
						case 0: // 鬆開夾爪
							gripper.Release();
							// 有問題
							break;
						case 1: // 夾爪抓取
							gripper.Grip();
							break;
						case 2: // 到達空間中某ㄧ點
							MoveToSpecifiedPoint();
							break;
						case 3: // 直接去抓 [PoseEstimation] 的結果
							// 重置'Reached'的狀態,才能開始跑
							// alongtraj.SetAngList(trajplanning.GetAngList());
							alongtraj.Resetnth_step();
							// 有問題
							break;
						case 4: // 列出場上所有 model
							listallmodel.ForceUpdate();
							movetoptr->ResetReached();//重置'Reached'的狀態,才能開始跑
							break;
						case 5: // 到達準備位置 (HOME)
							movetoptr->SetDefaultDestination(); // 設定預設終點
							// double vel[6] = {.75,.75,.75,.75,.75,.75}; // 給定速度(也可以不給,因為有預設的了)
							// movetoptr->SetVelocityOfJoint(vel); // 給定速度(也可以不給,因為有預設的了)
							movetoptr->ResetReached();
							break;
						case 6: //印出現在手臂位置
							movetoptr->PrintTipPose();
							break;
						case 7: // 改變單一joint的角度
							movetoptr->SetSpecificJoint();
							break;
						case 8: // 計算從預備位置到分割好的物體間的路徑
							//std::cout<<"alongtraj.CatchObjectOnTable\n";
							//debug用
							//alongtraj.CatchObjectOnTable();
							alongtraj.CatchObjectOnTableFromMsg();
							break;
						case 9: // 移動到小桌子
							// 桌子的位置需要再調整
							movetoptr->SetSmallTableDestination(); // 設定桌子為終點
							movetoptr->ResetReached(); // 重置'Reached'的狀態,才能開始跑
							break;
						case 10: // 強制停止大家動作
							EmergencyStop();
							break;
						case 19:
							std::cout << "alongtraj::PutOnTable\n";
							alongtraj.PutOnTable();
							break;

						}

/*
						if( userinput == 0 ) // 鬆開夾爪
						{
							gripper.Release();
						}
						else if( userinput == 1 ) // 夾爪抓取
						{
							gripper.Grip();
						}
						else if( userinput == 2 ) // 到達空間中某ㄧ點
						{
							float goalpose[2];
							std::cout << "input (x,y,z)\n" << std::endl;
							for(int i=0;i<3;i++)
							{
								printf("%d, ",i+1);
								scanf("%f",&goalpose[i]);
								std::cin.ignore();
							}
							// x方向加1，因為因為robot在座標的（-1,0,0）
							kdlik.CalculateIK(goalpose[0]+0.5,goalpose[1],goalpose[2]);
							double vel[6] = {1,1,1,1,1,1};
							movetoptr->SetVelocityOfJoint(vel);
						}
						else if( userinput == 3 ) // 重置'Reached'的狀態,才能開始跑
						{
							alongtraj.Resetnth_step();
						}
						else if( userinput == 4 ) // 先到達預備位置
						{
							movetoptr->SetDefaultDestination();//設定預設終點
						   // double vel[6]={.75,.75,.75,.75,.75,.75};//給定速度(也可以不給,因為有預設的了)
							//movetoptr->SetVelocityOfJoint(vel);//給定速度(也可以不給,因為有預設的了)
							movetoptr->ResetReached();//重置'Reached'的狀態,才能開始跑
						}
						if( userinput == 5 ) // 列出場上所有model
						{
							listallmodel.ForceUpdate();
						}
						if( userinput == 6 ) //印出現在手臂位置
						{
							movetoptr->PrintTipPose();
						}
						if( userinput == 7 ) // 改變單一joint的角度
						{
							movetoptr->SetSpecificJoint();
						}
						if( userinput == 8 ) // 計算從預備位置到球的位置中間的路徑
						{
							//std::cout<<"alongtraj.CatchObjectInBox()\n";
							alongtraj.CatchObjectInBox(publisher);
						}
						if( userinput == 18 )//計算從預備位置到球的位置中間的路徑
						{
							//std::cout<<"alongtraj.CatchObjectOnTable\n";
							//debug用
							//alongtraj.CatchObjectOnTable();
							alongtraj.CatchObjectOnTableFromMsg();

						}
						if( userinput == 9 )
						{
							movetoptr->SetTableDestination();//設定桌子為終點
							double vel[6]={.75,.75,.75,.75,.75,.75};//給定速度(也可以不給,因為有預設的了)
							movetoptr->SetVelocityOfJoint(vel);//給定速度(也可以不給,因為有預設的了)
							movetoptr->ResetReached();//重置'Reached'的狀態,才能開始跑
						}

						if( userinput == 19 )
						{
							//std::cout<<"alongtraj PutOnTable\n";
							alongtraj.PutOnTable();
						}
						if( userinput == 10 )
						{
							for(int i=0;i<world->GetModelCount();i++)
							{
								world->GetModel(i)->SetLinearVel(math::Vector3(0,0,0));//強制停止大家動作
							}
						}
						if( userinput == 11 )
						{
							gripper.Grip_cheat();
						}
						if( userinput == 12 )
						{

							gripper.Release_cheat();
						}
						if( userinput == 13 )
						{
							alongtraj.Up();
						}
						*/
			} // if
		} // while(1)

	}

	public: void OnUpdate(const common::UpdateInfo & /*_info*/)
	{
		listallmodel.Update(); // 在場上的 model數量有變化，就會 print 出 model 列表
		gripper.Update(); // update gripper motion

		if (userinput == 3) // alongtraj 比較特殊，這是控制手臂通過一連串的點，所以不能由 movetoptr->update 控制
		{
			alongtraj.Update();
		}
		else
		{
			movetoptr->Update(); // movetoptr->update 只能控制一個起點到一個終點
		}
	}

	public:void receivedGotIt(ConstMsgsRequestPtr &_msg) // 拍照完後robot才可以開始動
	{
		std::cout << COUT_PREFIX << "Robot go to default pose" << std::endl;
		world->GetModel("ar6aixs")->SetStatic(false);
		movetoptr->SetDefaultDestination();
		movetoptr->ResetReached();
	}

	public:void receivedGGininder(ConstMsgsRequestPtr &_msg)
	{
		std::string nearestmodel = _msg->request();
		std::cout << COUT_PREFIX << "received : " << nearestmodel << std::endl;

		gripper.SetCheatTarget(world->GetModel(nearestmodel));
		alongtraj.SetNearestModel(nearestmodel);
		alongtraj.CatchObjectInBoxFromMsg();
	}

	public:void Init()
	{
		movetoptr = MoveToPtr(new MoveTo(model));
		listallmodel = ListAllModel(model);
		gripper = Gripper(model);
		alongtraj = AlongTrajectory(movetoptr,model);
		kdlik = KDL_IK(movetoptr);
		userinput = 0;
		world->GetModel("ar6aixs")->SetStatic(true); // 把robot設成static要不然一開始會影響拍照

		//movetoptr->SetDefaultDestination();
		//movetoptr->ResetReached();
	}

	public:void MoveToSpecifiedPoint() // for case 2
	{
		float goalpose[3];
		std::cout << "input (x,y,z)" << std::endl;
		for(int i=0;i<3;i++)
		{
			switch( i )
			{
			case 0:
				printf("x = ");
				break;
			case 1:
				printf("y = ");
				break;
			case 2:
				printf("z = ");
				break;
			}
			scanf("%f", &goalpose[i]);
			std::cin.ignore();
		}

		// x方向加0.5，因為因為robot在座標的（-0.5,0,0）
		kdlik.CalculateIK(goalpose[0]+0.5,goalpose[1],goalpose[2]);
		double vel[6] = {1,1,1,1,1,1}; // set up velocity
		movetoptr->SetVelocityOfJoint(vel);
	}

	public:void EmergencyStop() // for case 10
	{
		for(int i=0;i<world->GetModelCount();i++)
		{
			world->GetModel(i)->SetLinearVel(math::Vector3(0,0,0)); // 強制停止大家動作
		}
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
		transport::SubscriberPtr subscribe_GotIt;
		transport::PublisherPtr publisher;
}; // end Class Arm_Control

GZ_REGISTER_MODEL_PLUGIN(Arm_Control);
