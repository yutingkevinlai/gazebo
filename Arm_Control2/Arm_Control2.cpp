






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


typedef std::vector<gazebo::math::Angle> Six_Ang;
typedef std::vector<Six_Ang> Ang_List_V;


using namespace gazebo;


class ListAllModel
{
private:
    int num_model_on_field;
    physics::WorldPtr world;


public:ListAllModel(){}
public:ListAllModel(physics::ModelPtr param_model)
{
    world=param_model->GetWorld();
    num_model_on_field=0;
}
public:void ForceUpdate()
{
    num_model_on_field=0;
}
public:void Update()
{

    if(world->GetModelCount()!=num_model_on_field)//在場上的的modell數量有變化
    {
        num_model_on_field=world->GetModelCount();
        std::cout<<"\n\nNow There are "<<num_model_on_field<<" models on the field"<<std::endl;
        for(int i=0;i<num_model_on_field;i++)//列出所有在場上的model名稱
        {
            std::cout<<i<<"\t"<<world->GetModel(i)->GetName()<<"\t("<<world->GetModel(i)->GetWorldPose().pos<<")"<<std::endl;
        }
    }
}
};

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
    if(check_joint[1]&&check_joint[0]&&
        check_joint[3]&&check_joint[2]&&
        check_joint[5]&&check_joint[4]&&
        check_joint[7]&&check_joint[6]&&
        check_joint[9]&&check_joint[8]&&
        check_joint[11]&&check_joint[10])
        return 1;
    else
        return 0;
}
public:void ResetReached()
{
    for(int i=0;i<12;i++)
    {
        check_joint[i]=0;
    }
}
public:void SetReached()
{
    for(int i=0;i<12;i++)
    {
        check_joint[i]=1;
    }
}

public:MoveTo(physics::ModelPtr param_model)
{
    for(int i=0;i<6;i++)
    {

        check_joint[2*i]=0;
        check_joint[2*i+1]=0;
    }




    velocityOfjoint[0]=1;
    velocityOfjoint[1]=1;
    velocityOfjoint[2]=1;
    velocityOfjoint[3]=1;
    velocityOfjoint[4]=1;
    velocityOfjoint[5]=1;




    currentang=Six_Ang(6,0);
    ang.push_back((math::Angle)(0));
    ang.push_back((math::Angle)(-.182));
    ang.push_back((math::Angle)(-1.992));
    ang.push_back((math::Angle)0);
    ang.push_back((math::Angle)(-0.9675));
    ang.push_back((math::Angle)3.14);
    axis[0]='z';
    axis[1]='y';
    axis[2]='y';
    axis[3]='z';
    axis[4]='y';
    axis[5]='z';
    world=param_model->GetWorld();
    model=param_model;
    link.push_back(model->GetLink("link1_1"));
    link.push_back(model->GetLink("link2_1"));
    link.push_back(model->GetLink("link3_1"));
    link.push_back(model->GetLink("link4_1"));
    link.push_back(model->GetLink("link5_1"));
    link.push_back(model->GetLink("link6"));
    joint.push_back(model->GetJoint("joint1"));
    joint.push_back(model->GetJoint("joint2"));
    joint.push_back(model->GetJoint("joint3"));
    joint.push_back(model->GetJoint("joint4"));
    joint.push_back(model->GetJoint("joint5"));
    joint.push_back(model->GetJoint("joint6"));
    axis_aligner.push_back(model->GetLink("base"));
    axis_aligner.push_back(model->GetLink("link1_2"));
    axis_aligner.push_back(model->GetLink("link2_2"));
    axis_aligner.push_back(model->GetLink("link3_3"));
    axis_aligner.push_back(model->GetLink("link4_2"));
    axis_aligner.push_back(model->GetLink("link5_3"));
}
public:void SetDestination(Six_Ang param_ang)
{
    ang.assign(param_ang.begin(),param_ang.end());
}


public:Six_Ang GetJointAngs()
{
	return ang;
	}
public:void SetSpecificJoint()
{
    std::cout<<"\nangle of each joint"<<std::endl;
    for(int i=0;i<6;i++)
        std::cout<<i+1<<", "<<ang[i]<<std::endl;

    int n;
    float input_angle;
    std::cout<<"input joint angle"<<std::endl;
    scanf("%d %f",&n,&input_angle);
    ang[n-1]=(math::Angle)input_angle;
    std::cin.ignore();
    ResetReached();
}


public:void SetDefaultDestination()
{
    ang[0]=(math::Angle)(0);
    ang[1]=(math::Angle)(0.157469);
    ang[2]=(math::Angle)(1.3);
    ang[3]=(math::Angle)0.000160237;
    ang[4]=(math::Angle)(1.2);
    ang[5]=(math::Angle)0;
    //0.8  0.157  2.5  2.5  1.15  0.2



}

public:void SetTableDestination()
{
	int input;
	scanf("%d",&input);
	if(input==0)
		{
			ang[0]=(math::Angle)(0.8);
			ang[1]=(math::Angle)(0.24);
			ang[2]=(math::Angle)(2.45);
			ang[3]=(math::Angle)(2.6);
			ang[4]=(math::Angle)(1.15);
			ang[5]=(math::Angle)(0.2);

//
//			  ang[0]=(math::Angle)(0.8);
//			ang[1]=(math::Angle)(0.157);
//			ang[2]=(math::Angle)(2.5);
//			ang[3]=(math::Angle)2.5;
//			ang[4]=(math::Angle)(1.15);
//			ang[5]=(math::Angle)0.25;
		}
	else if(input==1)
		{
		//-1.10174 1.10357 0.664433
			ang[0]=(math::Angle)(1.66337 );
			ang[1]=(math::Angle)(0.351866);
			ang[2]=(math::Angle)(1.78132 );
			ang[3]=(math::Angle)( -0.000910296  );
			ang[4]=(math::Angle)(0.99757  );
			ang[5]=(math::Angle)(-1.42549);
		}
	else if(input==2)
		{
		//-1.00401 1.10271 0.664433
			ang[0]=(math::Angle)(1.5737 );
			ang[1]=(math::Angle)( 0.345731);
			ang[2]=(math::Angle)(  1.78907);
			ang[3]=(math::Angle)(  0.000238946 );
			ang[4]=(math::Angle)( 0.995362 );
			ang[5]=(math::Angle)( -1.51372);
		}
	else if(input==3)
		{
		//-0.904956 1.10157 0.665342
			ang[0]=(math::Angle)(1.48394  );
			ang[1]=(math::Angle)( 0.348696  );
			ang[2]=(math::Angle)(1.7845 );
			ang[3]=(math::Angle)( 0.00149802 );
			ang[4]=(math::Angle)( 0.996457 );
			ang[5]=(math::Angle)( -1.60205);
		}
	else if(input==4)
		{
		// -1.10085 1.00461 0.664927
			ang[0]=(math::Angle)(1.67171);
			ang[1]=(math::Angle)(  0.24956);
			ang[2]=(math::Angle)(  1.91472 );
			ang[3]=(math::Angle)( -0.00107202 );
			ang[4]=(math::Angle)( 0.965988  );
			ang[5]=(math::Angle)(-1.41532);
		}
	else if(input==5)
		{
		// -1.00232 1.00337 0.66543
			ang[0]=(math::Angle)(1.57322  );
			ang[1]=(math::Angle)(0.242818);
			ang[2]=(math::Angle)(  1.92257 );
			ang[3]=(math::Angle)( 0.000253391 );
			ang[4]=(math::Angle)( 0.964214 );
			ang[5]=(math::Angle)( -1.5122);
		}
	else if(input==6)
		{
		// -0.9035 1.00261 0.665944
			ang[0]=(math::Angle)(1.47461);
			ang[1]=(math::Angle)(  0.246254 );
			ang[2]=(math::Angle)( 1.91761);
			ang[3]=(math::Angle)(  0.00171927 );
			ang[4]=(math::Angle)( 0.965169 );
			ang[5]=(math::Angle)( -1.60936);
		}


}
public:void SetVelocityOfJoint(double param_velocityOfjoint[6])
{
    for(int i=0;i<6;i++)
    {
        velocityOfjoint[i]= param_velocityOfjoint[i];

        //printf("%lf ",velocityOfjoint[i]);

    }
    printf("\n");

}
public:void SetPositionDirectly()
{
    for(int i=0;i<6;i++)//就定位後就固定住
        {
            link[i]->SetAngularVel(math::Vector3(0,0,0));
            joint[i]->SetHighStop(0,ang[i]+math::Angle(0.001));
            joint[i]->SetLowStop(0,ang[i]+math::Angle(-0.001));

        }
}
public:void Update(int type=0)
{
    //printf("%d %d %d %d %d %d %d %d %d %d %d %d \n",check_joint[1],check_joint[0],check_joint[3],check_joint[2],check_joint[5],check_joint[4],check_joint[7],check_joint[6],check_joint[9],check_joint[8],check_joint[11],check_joint[10]);

    if(check_joint[1]&&check_joint[0]&&check_joint[3]&&check_joint[2]&&check_joint[5]&&check_joint[4]&&check_joint[7]&&check_joint[6]&&check_joint[9]&&check_joint[8]&&check_joint[11]&&check_joint[10])
    {
        SetPositionDirectly();
        //SaveCurrentAngle();
    }
    else//各joint還沒就定位,就繼續跑
    {

        for(int i=0;i<6;i++)
        {
            if(type==0)SetEachLinkPosition(i);
        }
    }
}
private:void SetEachLinkPosition(int nth)
{
    //printf("%f %f %f %f %f %f \n",velocityOfjoint[0],velocityOfjoint[1],velocityOfjoint[2],velocityOfjoint[3],velocityOfjoint[4],velocityOfjoint[5]);

    if(!(check_joint[nth*2+1]&&check_joint[nth*2]))
    //if(check_joint[((nth*2-1)<0?12:(nth*2-1))]&&check_joint[((nth*2-2)<0?12:(nth*2-2))]&&!(check_joint[nth*2+1]&&check_joint[nth*2]))
    {


        //正要處理
        joint[nth]->SetHighStop(0,math::Angle(4));joint[nth]->SetLowStop(0,math::Angle(-4));
        if(((joint[nth]->GetAngle(0)-ang[nth])*(joint[nth]->GetAngle(0)-ang[nth]))<0.00001)
        {
            //std::cout<<axis[nth]<<nth<<" on the position! \n";
            check_joint[nth*2+1]=1;
            check_joint[nth*2]=1;
        }
        else if(joint[nth]->GetAngle(0)<=ang[nth])
        {
            //if(nth==1)std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" > "<<joint[nth]->GetAngle(0)<<" "<<velocityOfjoint[nth]<<std::endl;
            if(axis[nth]=='y')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,velocityOfjoint[nth],0)));
            if(axis[nth]=='z')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,0,velocityOfjoint[nth])));
            check_joint[nth*2+1]=1;
        }
        else if(joint[nth]->GetAngle(0)>ang[nth])
        {
            //if(nth==1)std::cout<<axis[nth]<<nth<<" "<<ang[nth]<<" < "<<joint[nth]->GetAngle(0)<<" "<<velocityOfjoint[nth]<<std::endl;
            if(axis[nth]=='y')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,-velocityOfjoint[nth],0)));
            if(axis[nth]=='z')link[nth]->SetAngularVel(axis_aligner[nth]->GetWorldPose().rot.RotateVector(math::Vector3(0,0,-velocityOfjoint[nth])));
            check_joint[nth*2]=1;
        }
    }
    else//以下是處理好的
    {
        joint[nth]->SetHighStop(0,ang[nth]+math::Angle(0.001));
        joint[nth]->SetLowStop(0,ang[nth]+math::Angle(-0.001));
    }



}

private:void SetEachLinkPositionOrder(int nth)
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
public:void PrintTipPose()
{
    std::cout<<"postion of tip : "<<model->GetLink("link6")->GetWorldPose()<<std::endl;
             //<<link[5]->GetWorldPose().pos+link[5]->GetWorldPose().rot.RotateVector(math::Vector3(0,0,3.5))
             //<<"\n";
    std::cout<<"joint angle\n";
    for(int i=0;i<6;i++)
        std::cout<<ang[i]<<"  ";

    std::cout<<std::endl;


}
public:math::Pose GetTipPose()
{
    return math::Pose(link[5]->GetWorldPose().pos+link[5]->GetWorldPose().rot.RotateVector(math::Vector3(0,0,3.5))
                        ,link[5]->GetWorldPose().rot);
}
};
typedef boost::shared_ptr<MoveTo> MoveToPtr;

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
    double vel[6];
public:TrajectoryPlanning()
{
    start_ang_joint=Six_Ang(6,0);
    end_ang_joint=Six_Ang(6,0);
    result_ang_joint=Six_Ang(6,0);
    srand(time(NULL));
    using namespace KDL;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.5964912281))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.8421052632))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.9649122807))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.350877193))));
    q_init=KDL::JntArray(6);
    q_init.data.setRandom();
    q_init.data *= M_PI;
    L(0)=1;
    L(1)=1;
    L(2)=1;
    L(3)=0.01;
    L(4)=0.01;
    L(5)=0.01;

}

public:int Calculate_beta(gazebo::math::Pose start_pose_cart,gazebo::math::Pose end_pose_cart,float buffer_height=-0.002 ,int num_point=100)
{

    using namespace KDL;

    //comment那堆程式碼是從側面接近

//    // //gazebo的pose轉成KDL的Frame

    math::Matrix3 rot_start=start_pose_cart.rot.GetAsMatrix3();
    KDL::Vector tmpvec(start_pose_cart.pos.x,start_pose_cart.pos.y,start_pose_cart.pos.z);
    Frame start_pose_in_KDL(KDL::Rotation(rot_start[0][0],rot_start[0][1],rot_start[0][2],
                                          rot_start[1][0],rot_start[1][1],rot_start[1][2],
                                          rot_start[2][0],rot_start[2][1],rot_start[2][2]),
                            tmpvec);

    //std::cout<<"start pos\n"<<start_pose_in_KDL;


    //夾爪的抓取rot設得跟工件一樣
    math::Vector3 tmpx=end_pose_cart.rot.GetZAxis();
    math::Vector3 tmpy=tmpx.Cross(math::Vector3(0,0,1));
    math::Vector3 tmpz=tmpx.Cross(tmpy);
    math::Vector3 x=tmpx.Normalize();
    math::Vector3 y=tmpy.Normalize();
    math::Vector3 z=tmpz.Normalize();
    double roll,pitch,yaw;
    KDL::Rotation tmpr(x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z);
    tmpr.GetRPY(roll,pitch,yaw);
    math::Quaternion aligner(roll,pitch,yaw);
    //math::Matrix3 rot_end=start_pose_cart.rot.GetAsMatrix3();
    math::Vector3 tmp_pos=end_pose_cart.pos+aligner.RotateVector(math::Vector3(0,0,-1*buffer_height));

    tmpvec=KDL::Vector(tmp_pos.x,tmp_pos.y,tmp_pos.z);
    Frame end_pose_in_KDL(KDL::Rotation(x.x,y.x,z.x,
                                        x.y,y.y,z.y,
                                        x.z,y.z,z.z),
                            tmpvec);


//
//    Frame end_pose_in_KDL(KDL::Rotation::Quaternion(
//    					end_pose_cart.rot.x,
//    					end_pose_cart.rot.y,
//    					end_pose_cart.rot.z,
//    					end_pose_cart.rot.w),
//    				KDL::Vector(
//    						end_pose_cart.pos.x,
//    						end_pose_cart.pos.y,
//    						end_pose_cart.pos.z));
//
//
//    Frame start_pose_in_KDL(KDL::Rotation::Quaternion(
//    					start_pose_cart.rot.x,
//    					start_pose_cart.rot.y,
//    					start_pose_cart.rot.z,
//    					start_pose_cart.rot.w),
//    				KDL::Vector(start_pose_cart.pos.x,
//    						start_pose_cart.pos.y,
//    						start_pose_cart.pos.z));






    //std::cout<<"\nend pose \n"<<end_pose_in_KDL;
    //以上得到路徑的起點與終點
    //接著使用kdl的路徑規劃

    Path_RoundedComposite* path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
    //設定要經過的frame
    path->Add(end_pose_in_KDL);

    path->Add(KDL::Frame(
    					KDL::Rotation(
    							end_pose_in_KDL.M.UnitX(),
    							end_pose_in_KDL.M.UnitY(),
    							end_pose_in_KDL.M.UnitZ()),
						KDL::Vector(
								end_pose_in_KDL.p.data[0],
								end_pose_in_KDL.p.data[1],
								end_pose_in_KDL.p.data[2]+0.15)));
    path->Add(start_pose_in_KDL);
    path->Finish();

    //設置速度速度profile
    VelocityProfile* velpref = new VelocityProfile_Trap(0.05,0.2);
    velpref->SetProfile(0,path->PathLength());

    Trajectory* traject = new Trajectory_Segment(path, velpref);




    // use the trajectory



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
    for (double t=0.0; t <= traject->Duration(); t+= dt)
    {
    	//std::cout<<"solve "<<t*10<<" th IK\n";
    	count=0;
        while(retval||(q_sol.data(1,0)<-1.57)||(q_sol.data(1,0)>1.57))//不要大於1.57,超出這範圍,會撞到地
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
    std::cout<<ang_list.size()<<" pts"<<std::endl;







//    for(int i=0;i<6;i++)
//    {
//        vel[i]=0.2;
//    }
//    std::cout<<std::endl;
    std::cout<<"Calculate_beta done!"<<std::endl;
}


public:int Calculate_OnTable(gazebo::math::Pose start_pose_cart,gazebo::math::Pose end_pose_cart,float buffer_height=0.009 ,int num_point=100)
{

    using namespace KDL;

    //從側面接近

    // //gazebo的pose轉成KDL的Frame

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
    Frame end_pose_in_KDL(KDL::Rotation::Quaternion(
    					end_pose_cart.rot.x,
    					end_pose_cart.rot.y,
    					end_pose_cart.rot.z,
    					end_pose_cart.rot.w),
    				KDL::Vector(
    						end_pose_cart.pos.x,
    						end_pose_cart.pos.y,
    						end_pose_cart.pos.z));

//延y軸轉180
    end_pose_in_KDL.M.data[0]*=-1;
    end_pose_in_KDL.M.data[3]*=-1;
    end_pose_in_KDL.M.data[6]*=-1;

    end_pose_in_KDL.M.data[2]*=-1;
    end_pose_in_KDL.M.data[5]*=-1;
    end_pose_in_KDL.M.data[8]*=-1;



    Frame start_pose_in_KDL(KDL::Rotation::Quaternion(
    					start_pose_cart.rot.x,
    					start_pose_cart.rot.y,
    					start_pose_cart.rot.z,
    					start_pose_cart.rot.w),
    				KDL::Vector(start_pose_cart.pos.x,
    						start_pose_cart.pos.y,
    						start_pose_cart.pos.z));






    //std::cout<<"\nend pose \n"<<end_pose_in_KDL;
    //以上得到路徑的起點與終點
    //接著使用kdl的路徑規劃

    Path_RoundedComposite* path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
    //設定要經過的frame
    path->Add(end_pose_in_KDL);
std::cout<<"  path->Add(KDL:\n";
    path->Add(KDL::Frame(
    					KDL::Rotation(
    							end_pose_in_KDL.M.UnitX(),
    							end_pose_in_KDL.M.UnitY(),
    							end_pose_in_KDL.M.UnitZ()),
						KDL::Vector(
								end_pose_in_KDL.p.data[0],
								end_pose_in_KDL.p.data[1],
								end_pose_in_KDL.p.data[2]+0.15)));
    path->Add(start_pose_in_KDL);
    path->Finish();

    //設置速度速度profile
    VelocityProfile* velpref = new VelocityProfile_Trap(0.05,0.2);
    velpref->SetProfile(0,path->PathLength());

    Trajectory* traject = new Trajectory_Segment(path, velpref);




    // use the trajectory



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
    for (double t=0.0; t <= traject->Duration(); t+= dt)
    {
    	//std::cout<<"solve "<<t*10<<" th IK\n";
    	count=0;
        while(retval||(q_sol.data(1,0)<-1.57)||(q_sol.data(1,0)>1.57))//不要大於1.57,超出這範圍,會撞到地
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
    std::cout<<ang_list.size()<<" pts"<<std::endl;







//    for(int i=0;i<6;i++)
//    {
//        vel[i]=0.2;
//    }
//    std::cout<<std::endl;
    std::cout<<"Calculate_beta done!"<<std::endl;
}

public:int Calculate_PutOnTable(gazebo::math::Pose start_pose_cart,gazebo::math::Pose end_pose_cart,float buffer_height=0.009 ,int num_point=100)
{

	std::cout<<"\nstart pose(RAW) \n"<<start_pose_cart;
	std::cout<<"\nend pose(RAW) \n"<<end_pose_cart;

    using namespace KDL;

//從側面接近
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


//
    Frame end_pose_in_KDL(KDL::Rotation::Quaternion(
    					end_pose_cart.rot.x,
    					end_pose_cart.rot.y,
    					end_pose_cart.rot.z,
    					end_pose_cart.rot.w),
    				KDL::Vector(
    						end_pose_cart.pos.x,
    						end_pose_cart.pos.y,
    						end_pose_cart.pos.z-0.155));

//延y軸轉180
    end_pose_in_KDL.M.data[1]*=-1;
    end_pose_in_KDL.M.data[4]*=-1;
    end_pose_in_KDL.M.data[7]*=-1;

    end_pose_in_KDL.M.data[2]*=-1;
    end_pose_in_KDL.M.data[5]*=-1;
    end_pose_in_KDL.M.data[8]*=-1;



    Frame start_pose_in_KDL(KDL::Rotation::Quaternion(
    					start_pose_cart.rot.x,
    					start_pose_cart.rot.y,
    					start_pose_cart.rot.z,
    					start_pose_cart.rot.w),
    				KDL::Vector(start_pose_cart.pos.x,
    						start_pose_cart.pos.y,
    						start_pose_cart.pos.z));






    std::cout<<"\nstart pose \n"<<start_pose_in_KDL;
    std::cout<<"\nend pose \n"<<end_pose_in_KDL;

    //以上得到路徑的起點與終點
    //接著使用kdl的路徑規劃

    Path_RoundedComposite* path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
    //設定要經過的frame
    path->Add(end_pose_in_KDL);

    path->Add(KDL::Frame(
    					KDL::Rotation(
    							end_pose_in_KDL.M.UnitX(),
    							end_pose_in_KDL.M.UnitY(),
    							end_pose_in_KDL.M.UnitZ()),
						KDL::Vector(
								end_pose_in_KDL.p.data[0],
								end_pose_in_KDL.p.data[1],
								end_pose_in_KDL.p.data[2]+0.2)));
    std::cout<<"  path->Add(KDL:\n";
    path->Add(start_pose_in_KDL);
    path->Finish();

    //設置速度速度profile
    VelocityProfile* velpref = new VelocityProfile_Trap(0.05,0.2);
    velpref->SetProfile(0,path->PathLength());

    Trajectory* traject = new Trajectory_Segment(path, velpref);




    // use the trajectory



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
    for (double t=0.0; t <= traject->Duration(); t+= dt)
    {
    	//std::cout<<"solve "<<t*10<<" th IK\n";
    	count=0;
        while(retval||(q_sol.data(1,0)<-1.57)||(q_sol.data(1,0)>1.57))//不要大於1.57,超出這範圍,會撞到地
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
    std::cout<<ang_list.size()<<" pts"<<std::endl;







//    for(int i=0;i<6;i++)
//    {
//        vel[i]=0.2;
//    }
//    std::cout<<std::endl;
    std::cout<<"Calculate_beta done!"<<std::endl;
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

    // //gazebo的pose轉成KDL的Frame

    math::Matrix3 rot_start=start_pose_cart.rot.GetAsMatrix3();
    KDL::Vector tmpvec(start_pose_cart.pos.x,start_pose_cart.pos.y,start_pose_cart.pos.z);
    Frame start_pose_in_KDL(KDL::Rotation(rot_start[0][0],rot_start[0][1],rot_start[0][2],
                                          rot_start[1][0],rot_start[1][1],rot_start[1][2],
                                          rot_start[2][0],rot_start[2][1],rot_start[2][2]),
                            tmpvec);

    std::cout<<"start pos\n"<<start_pose_in_KDL;
//    math::Matrix3 rot_end=start_pose_cart.rot.GetAsMatrix3();
//    tmpvec=KDL::Vector(end_pose_cart.pos.x,end_pose_cart.pos.y,end_pose_cart.pos.z);
//    Frame end_pose_in_KDL(KDL::Rotation(rot_end[0][0],rot_end[0][1],rot_end[0][2],
//                                        rot_end[1][0],rot_end[1][1],rot_end[1][2],
//                                        rot_end[2][0],rot_end[2][1],rot_end[2][2]),
//                            tmpvec);




    //夾爪的抓取rot設得跟工件一樣
    math::Vector3 tmpx=end_pose_cart.rot.GetZAxis();
    math::Vector3 tmpy=tmpx.Cross(math::Vector3(0,0,1));
    math::Vector3 tmpz=tmpx.Cross(tmpy);
    math::Vector3 x=tmpx.Normalize();
    math::Vector3 y=tmpy.Normalize();
    math::Vector3 z=tmpz.Normalize();
    double roll,pitch,yaw;
    KDL::Rotation tmpr(x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z);
    tmpr.GetRPY(roll,pitch,yaw);
    math::Quaternion aligner(roll,pitch,yaw);
    //math::Matrix3 rot_end=start_pose_cart.rot.GetAsMatrix3();
    math::Vector3 tmp_pos=end_pose_cart.pos+aligner.RotateVector(math::Vector3(0,0,-1*buffer_height));

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
        while(retval||(q_sol.data(1,0)<-1.57)||(q_sol.data(1,0)>1.57))//不要大於1.57,超出這範圍,會撞到地
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
        while(retval||(q_sol.data(1,0)<-1.57)||(q_sol.data(1,0)>1.57))//不要大於1.57,超出這範圍,會撞到地
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



};

std::ostream&  operator<<(std::ostream &out,Six_Ang &Six_Ang_param)
{
	out<<Six_Ang_param[0]<<" , "
		<<Six_Ang_param[1]<<" , "
		<<Six_Ang_param[2]<<" , "
		<<Six_Ang_param[3]<<" , "
		<<Six_Ang_param[4]<<" , "
		<<Six_Ang_param[5]<<"\n";

	return out;
}


typedef const boost::shared_ptr<const my::msgs::PoseEstimationResult > ConstMsgsPoseEstimationResultPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Request > ConstMsgsRequestPtr;



class AlongTrajectory
{
private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    Ang_List_V ang_list;
    int nth_step;
    MoveToPtr movetoptr;
    std::string targetname;

public:AlongTrajectory()
{
}
public:void SetNearestModel(std::string param_name)
{
	targetname=param_name;
}
public:AlongTrajectory(MoveToPtr param_movetoptr,physics::ModelPtr param_model)
{
    movetoptr=param_movetoptr;
    model=param_model;
    world=model->GetWorld();
    nth_step=0;
}
public:void SetAngList(Ang_List_V param_ang_list)
{
    ang_list.assign(param_ang_list.begin(),param_ang_list.end());
}
public:void Resetnth_step()
{
    nth_step=0;
}


public:void CatchObjectOnTableFromMsg()
{
    std::cout<<"approach\n";
    TrajectoryPlanning tra;
    math::Pose targetpose;
    //因為機器手臂原點不在0 0 0 所以要shift
    targetpose=world->GetModel(targetname)->GetWorldPose()-math::Pose(math::Vector3(-1,0,0),math::Quaternion(0,0,0));
    std::cout<<"1.187476,3e-06,1.35262,-3.14107,0.482488,-3.14047\n";
    tra.Calculate_OnTable(math::Pose(1.187476,3e-06,1.1852,-3.14107,0.482488,-3.14047),targetpose);
            //math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
    std::cout<<"exit tra.Calculate\n";
    ang_list=tra.GetAngList();
    std::reverse(ang_list.begin(),ang_list.end());

}

public:void CatchObjectInBoxFromMsg()
{
	std::cout<<"CatchObjectInBoxFromMsg \n";
	std::cout<<"receive target model form Evaluation Platform\n\n";
	std::cout<<"target: "<<targetname<<std::endl;
    TrajectoryPlanning tra;
    math::Pose targetpose;
    //因為機器手臂原點不在0 0 0 所以要shift
    targetpose=world->GetModel(targetname)->GetWorldPose()-math::Pose(math::Vector3(-1,0,0),math::Quaternion(0,0,0));
    tra.Calculate_beta(math::Pose(0.9473, 0, 0.6315, 3.14123 ,-0.001664, -3.14148),targetpose);
            //math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
    std::cout<<"exit tra.Calculate\n";
    ang_list=tra.GetAngList();
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



public:void PutOnTable()
{

	//終點index
	int position_num=1;
	std::cin>>position_num;
	//一連串終點位置
		float 	detination[27]={



		-1.10085,0.90461,0.66492,
		-1.00232,0.90337,0.66543,
		-0.90350,0.90261,0.66594,
		-1.10085,1.00461,0.66492,
		-1.00232,1.00337,0.66543,
		-0.90350,1.00261,0.66544,
		-1.10174,1.10357,0.66443,
		-1.00401,1.10271,0.66443,
		-0.90495,1.10157,0.66534};


	std::cout<<"AlongTrajectory::PutOnTable\n";
	TrajectoryPlanning tra;
	math::Pose targetpose;
	//因為機器手臂原點不在0 0 0 所以要shift




	targetpose=math::Pose(
			detination[position_num*3-3],
			detination[position_num*3-2],
			detination[position_num*3-1],
			0,0,0);

	targetpose=targetpose-math::Pose(math::Vector3(-1,0,0),math::Quaternion(0,0,0));


	std::cout<<"public:void PutOnTable()\ntargetpose:\n"<<targetpose<<std::endl;
	tra.Calculate_PutOnTable(
							math::Pose(-0.743662,0.657557 ,1.14789, -3.14129 ,0.338589 ,-1.94315),
							targetpose);
			//math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
	std::cout<<"exit tra.Calculate\n";
	ang_list=tra.GetAngList();
	std::reverse(ang_list.begin(),ang_list.end());
}

public:void CatchObjectOnTable()
{


    std::cout<<"approach\n";
    TrajectoryPlanning tra;
    //投機作法這邊要改
    math::Pose targetpose;
    //因為機器手臂原點不在0 0 0 所以要shift
    int whichitem=0;
    std::cout<<"which item?\n";
    scanf("%d",&whichitem);
    std::cin.ignore();
    targetpose=world->GetModel(whichitem)->GetWorldPose()-math::Pose(math::Vector3(-1,0,0),math::Quaternion(0,0,0));
    std::cout<<"1.187476,3e-06,1.35262,-3.14107,0.482488,-3.14047\n";
    tra.Calculate_OnTable(math::Pose(1.187476,3e-06,1.1852,-3.14107,0.482488,-3.14047),targetpose);
            //math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
    std::cout<<"exit tra.Calculate\n";
    ang_list=tra.GetAngList();
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



public:void CatchObjectInBox()
{
    std::cout<<"approach\n";
    TrajectoryPlanning tra;
    //投機作法這邊要改
    math::Pose targetpose;
    //因為機器手臂原點不在0 0 0 所以要shift
    int whichitem=0;
    std::cout<<"which item?\n";
    scanf("%d",&whichitem);
    std::cin.ignore();
    targetpose=world->GetModel(whichitem)->GetWorldPose()-math::Pose(math::Vector3(-1,0,0),math::Quaternion(0,0,0));
    std::cout<<"1.187476,3e-06,1.35262,-3.14107,0.482488,-3.14047\n";
    tra.Calculate_beta(math::Pose(1.187476,3e-06,1.1852,-3.14107,0.482488,-3.14047),targetpose);
            //math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
    std::cout<<"exit tra.Calculate\n";
    ang_list=tra.GetAngList();
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
public:void Up()
{

//0.8  0.157  2.5  2.5  1.15  0.2

    std::reverse(ang_list.begin(), ang_list.end());
    ang_list.erase(ang_list.begin(),ang_list.begin()+2);
}
public:void PutDown()
{

//0.8  0.157  2.5  2.5  1.15  0.2

    std::cout<<"PutDown\n";
    TrajectoryPlanning tra;
    int whichitem=0;
    std::cout<<"which item?\n";
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

    std::cout<<"PickU\n";
    TrajectoryPlanning tra;
    int whichitem=0;
    std::cout<<"which item?\n";
    scanf("%d",&whichitem);
    std::cin.ignore();
    tra.Calculate(movetoptr->GetTipPose(),
                    world->GetModel(whichitem)->GetWorldPose(),0.25,10);
            //math::Pose(-26.9862 ,-0.013637,4.73187,-3.14136,-0.001928,0.003578));
    std::cout<<"exit tra.Calculate\n";
    ang_list=tra.GetAngList();

}
public:void Update()
{


    if(movetoptr->HasReached()&&nth_step!=(ang_list.size()-1))
    {


        movetoptr->ResetReached();
        movetoptr->SetDestination(ang_list[nth_step]);
        //避免超出範圍
        //如果ang_list.size()=10,在nth_step=10還可以進入IF IF內再加1,下次再進入IF 就會超出大小
        if(nth_step<(ang_list.size()-1))
        	nth_step++;


		//為了讓他只顯示一次
		if(nth_step==(ang_list.size()-1))
		{
			std::cout<<"reach the destination"<<std::endl;

		}
    }

    movetoptr->Update();

}


};

class Gripper
{
private:
    int grip;
    physics::ModelPtr model;
    physics::WorldPtr world;
    physics::ModelPtr target_for_cheat_grip;

public:Gripper(){}
public:void SetCheatTarget(physics::ModelPtr param_model)
{
	target_for_cheat_grip=param_model;
}
public:Gripper(physics::ModelPtr param_model)
{
    grip=0;
    model=param_model;
    world=model->GetWorld();
}
public:void Grip()
{
    grip=1;


}
public:void Grip_cheat()
{
    grip=1;


    std::cout<<"Grip_cheat : "<<target_for_cheat_grip->GetName()<<std::endl;
    world->GetModel("result_visualize_socket")->SetWorldPose(math::Pose(10,10,10,0,0,0));
    target_for_cheat_grip->SetEnabled( true );
    target_for_cheat_grip->SetStatic( false );
    model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),target_for_cheat_grip->GetLink("link"));



    //下面是單一model時debug用
//    world->GetModel("socket")->SetEnabled( true );
//    world->GetModel("socket")->SetStatic( false );
//    model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),world->GetModel("socket")->GetLink("link"));



}
public:void Realse_cheat()
{
    grip=0;

    math::Pose dummypose;
    math::Pose plampose=model->GetLink("gripper::plam")->GetWorldPose();
    dummypose=plampose+math::Pose(plampose.rot.RotateVector(math::Vector3(0,0,-0.05)),math::Quaternion(0,0,0));

    model->GetLink("gripper::dummy_for_glue")->SetWorldPose(dummypose);

	target_for_cheat_grip->SetEnabled( true );
	target_for_cheat_grip->SetStatic( false );
//   //下面是單一model時debug用
//   world->GetModel("socket")->SetEnabled( true );
//	world->GetModel("socket")->SetStatic( false );

	 model->GetJoint("gripper::glue")->Attach(model->GetLink("gripper::plam"),model->GetLink("gripper::dummy_for_glue"));





}

public:void Realse()
{
    grip=0;
}
public:int IsGrip()
{
    return grip;
}
public:void Update()
{
    if(grip==0)
    {
        model->GetLink("gripper::right_finger")->AddRelativeForce(math::Vector3(0,5,0));
        model->GetLink("gripper::left_finger")->AddRelativeForce(math::Vector3(0,-5,0));
    }
    if(grip==1)
    {
        model->GetLink("gripper::right_finger")->AddRelativeForce(math::Vector3(0,-5,0));
        model->GetLink("gripper::left_finger")->AddRelativeForce(math::Vector3(0,5,0));
//        model->GetJoint("gripper::right_joint")->SetHighStop(1,math::Angle(-.625));
//        model->GetJoint("gripper::right_joint")->SetLowStop(1,math::Angle(-.625));
//        model->GetJoint("gripper::left_joint")->SetHighStop(1,math::Angle(.625));
//        model->GetJoint("gripper::left_joint")->SetLowStop(1,math::Angle(.625));
    }


}

};

class KDL_IK
{
private:
    MoveToPtr movetoptr;
    KDL::Chain chain;
    KDL::JntArray q_init;
    Eigen::Matrix<double,6,1> L;
    Six_Ang ang;
public:KDL_IK(){}
public:KDL_IK(MoveToPtr param_movetoptr)
{
    movetoptr=param_movetoptr;
    movetoptr->SetDestination(Six_Ang(6,0));
    using namespace KDL;

    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.5964912281))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.8421052632))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.9649122807))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.350877193))));

    ang=Six_Ang(6,0);
    q_init=KDL::JntArray(6);
    q_init.data.setRandom();
    q_init.data *= M_PI;
    movetoptr->SetReached();//先設為reached,否則不是reached的話,他就會開始走

}
public:int CalculateIK(float x,float y ,float z)
{
    using namespace KDL;

    int n = chain.getNrOfJoints();
    Eigen::Matrix<double,6,1> L;
    L(0)=1;
    L(1)=1;
    L(2)=1;
    L(3)=0.01;
    L(4)=0.01;
    ChainIkSolverPos_LMA solver(chain,L);
    JntArray q_sol(n);
    q_sol.data(1,0)=3;//進入while的條件
    int retval,count=0;
    //夾爪方向往下
    Frame pos_goal(KDL::Rotation(-1,0,0,
    							0,1,0,
    							0,0,-1),KDL::Vector(x,y,z));
    while(retval||(q_sol.data(1,0)<-1.57)||(q_sol.data(1,0)>1.57))//不要大於1.57,超出這範圍,會撞到地
    {
    	std::cout<<" \n  "<<count++;
        if(count>10)
        {
        	std::cout<<"cannot solve !\n";
        	return 0;
        }
        //每次初始值都隨機
        //q_init.data.setRandom();
        //q_init.data *= M_PI;
        Six_Ang ang_for_init=movetoptr->GetJointAngs();
        q_init.data(0,0)=ang_for_init[0].Radian();
        q_init.data(1,0)=ang_for_init[1].Radian();
        q_init.data(2,0)=ang_for_init[2].Radian();
        q_init.data(3,0)=ang_for_init[3].Radian();
        q_init.data(4,0)=ang_for_init[4].Radian();
        q_init.data(5,0)=ang_for_init[5].Radian();

        //以目前位置當作初始值

        pos_goal=Frame(KDL::Rotation(-1,0,0,
									0,1,0,
									0,0,-1),KDL::Vector(x,y,z));
        retval = solver.CartToJnt(q_init,pos_goal,q_sol);

    }

    std::cout<<"q_sol : "<<q_sol.data<<std::endl;
    for(int i=0;i<6;i++)
        {

            if(q_sol.data(i,0)>M_PI)q_sol.data(i,0)=q_sol.data(i,0)-2*M_PI;
            if((q_sol.data(i,0)+M_PI)<0)q_sol.data(i,0)=q_sol.data(i,0)+2*M_PI;
            ang[i]=q_sol.data(i,0);
        }

    movetoptr->ResetReached();
    movetoptr->SetDestination(ang);
}






//public:void Update()
//{
//    if(!movetoptr->HasReached())
//        movetoptr->Update();
//    else
//        movetoptr->SetPositionDirectly();
//}
};

class Arm_Control2 : public ModelPlugin
{
public:~Arm_Control2()
{

}
public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
    this->model = _parent;
    world=model->GetWorld();

    boost::thread _Thread(&Arm_Control2::Keyboard_Input,this);


    node= transport::NodePtr( new transport::Node() );
    node->Init(world->GetName());

    //subscribe=node->Subscribe("~/pose_estimation/estimate_result",&Arm_Control2::receivedpose,this);//還是不知道還是3個參數是幹啥小用的

    subscribe_GGininder=node->Subscribe("~/evaluation_platform/GGininder" ,&Arm_Control2::receivedGGininder,this);//還是不知道還是3個參數是幹啥小用的



    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Arm_Control2::OnUpdate, this, _1));
}
public:void Keyboard_Input()
{

    std::cout<<"waiting for input...";

    while(1)
    {
    	if(std::cin.get()=='\n')//按下enter,停止目前動作
    	{
			std::cout <<"\n Menu:\n\n"
					<< "0\t release\n"
					<< "1\t grip\n"
					<< "2\t coordinate cartesian space\n"
					<< "3\t to catch\n"
					<< "4\t default position\n"
					<< "5\t model list\n"
					<< "6\t tip pose\n"
					<< "7\t input joint angle\n"
					<< "8\t from default position to targer\n"
					<< "9\t put on the table\n"
					<< "10\t stop\n"
					<< "11\t grip_cheat\n"
					<< "12\t release_cheat\n";
    	            scanf("%d",&userinput);
    	            std::cin.ignore();
    	            if(userinput==0)//鬆開夾爪
    	                gripper.Realse();
    	            if(userinput==1)//夾爪抓取
    	                gripper.Grip();
    	            if(userinput==2)//到達空間中某ㄧ點
    	            {
    	                float goalpose[2];
    	                std::cout<<"input (x,y,z)\n"<<std::endl;
    	                for(int i=0;i<3;i++)
    	                {
    	                    printf("%d, ",i+1);
    	                    scanf("%f",&goalpose[i]);
    	                    std::cin.ignore();
    	                }
    	                //x方向加1 因為因為robot在座標的（-1,0,0）
    	                kdlik.CalculateIK(goalpose[0]+1,goalpose[1],goalpose[2]);
    	                double vel[6]={1,1,1,1,1,1};
    	                movetoptr->SetVelocityOfJoint(vel);

    	            }
    	            if(userinput==3)//重置'Reached'的狀態,才能開始跑
    	            {
    	                alongtraj.Resetnth_step();
    	            }
    	            if(userinput==4)//先到達預備位置
    	            {
    	                movetoptr->SetDefaultDestination();//設定預設終點
    	               // double vel[6]={.75,.75,.75,.75,.75,.75};//給定速度(也可以不給,因為有預設的了)
    	                //movetoptr->SetVelocityOfJoint(vel);//給定速度(也可以不給,因為有預設的了)
    	                movetoptr->ResetReached();//重置'Reached'的狀態,才能開始跑
    	            }

    	            if(userinput==5)//列出場上所有model
    	            {
    	                listallmodel.ForceUpdate();
    	            }
    	            if(userinput==6)//印出現在手臂位置
    	            {
    	                movetoptr->PrintTipPose();
    	            }
    	            if(userinput==7)//改變單一joint的角度
    	            {
    	                movetoptr->SetSpecificJoint();
    	            }
    	            if(userinput==8)//計算從預備位置到球的位置中間的路徑
    	            {
    	                //std::cout<<"alongtraj.CatchObjectInBox()\n";
    	                alongtraj.CatchObjectInBox();
    	            }
    	            if(userinput==18)//計算從預備位置到球的位置中間的路徑
    	            {
    	                //std::cout<<"alongtraj.CatchObjectOnTable\n";
    	                //debug用
    	                //alongtraj.CatchObjectOnTable();
    	                //
    	            	double vel[6]={.75,.75,.75,.75,.75,.75};
    	            	movetoptr->SetVelocityOfJoint(vel);
    	                alongtraj.CatchObjectOnTableFromMsg();

    	            }
    	            if(userinput==9)
    	            {
    	                movetoptr->SetTableDestination();//設定桌子為終點
    	                double vel[6]={.75,.75,.75,.75,.75,.75};//給定速度(也可以不給,因為有預設的了)
    	                movetoptr->SetVelocityOfJoint(vel);//給定速度(也可以不給,因為有預設的了)
    	                movetoptr->ResetReached();//重置'Reached'的狀態,才能開始跑
    	            }

    	            if(userinput==19)
    	            {
    	            	//std::cout<<"alongtraj PutOnTable\n";
    	            	alongtraj.PutOnTable();
    	            }
    	            if(userinput==10)
    	            {

    	                for(int i=0;i<world->GetModelCount();i++)
    	                {
    	                    world->GetModel(i)->SetLinearVel(math::Vector3(0,0,0));//強制停止大家動作
    	                }
    	            }
    	            if(userinput==11)
    	            {
    	                gripper.Grip_cheat();
    	            }
    	            if(userinput==12)
    	            {
    	                gripper.Realse_cheat();
    	            }
    	            if(userinput==13)
    	            {
    	            	alongtraj.Up();
    	            }
    	        }


    }

}
public: void OnUpdate(const common::UpdateInfo & /*_info*/)
{


    listallmodel.Update();//在場上的的modell數量有變化 就會print出model列表
    gripper.Update();
	if (userinput == 3)    	//alongtraj比較特殊，這是控制手臂通過一連串的點，所以不能由movetoptr->update控制
		alongtraj.Update();
    else
        movetoptr->Update();//movetoptr->update只能控制一個起點到一個終點








}
//public:void  receivedpose(ConstMsgsPoseEstimationResultPtr &_msg)
//{
//	std::cout<<"\033[7;32m"
//			<<_msg->object_name()
//			<<"\033[m"<<std::endl;
//	for( int j = 0; j < 4; j++ )
//		{
//			for( int i = 0; i < 4; i++ )
//			{
//				std::cout<<_msg->pose_matrix4( i + j * 4 )<<" ";
//			}
//			std::cout<<std::endl;
//		}
//}
public:void  receivedGGininder(ConstMsgsRequestPtr &_msg)
{
	std::string nearestmodel=_msg->request();
	std::cout<<"\033[7;32m"
			<<"Arm_Control receive : "
			<<nearestmodel
			<<"\033[m"<<std::endl;

	gripper.SetCheatTarget(world->GetModel(nearestmodel));
	alongtraj.SetNearestModel(nearestmodel);
	alongtraj.CatchObjectInBoxFromMsg();

}

public:void Init()
{

    movetoptr=MoveToPtr(new MoveTo(model));
    listallmodel=ListAllModel(model);
    gripper=Gripper(model);
    alongtraj=AlongTrajectory(movetoptr,model);
    kdlik=KDL_IK(movetoptr);
    userinput=0;





}
private:
    int userinput;
    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
    ListAllModel listallmodel;
    AlongTrajectory alongtraj;
    Gripper gripper;
    KDL_IK kdlik;
    MoveToPtr movetoptr;

    transport::NodePtr node;
    //transport::SubscriberPtr subscribe;
    transport::SubscriberPtr subscribe_GGininder;



};


GZ_REGISTER_MODEL_PLUGIN(Arm_Control2);












