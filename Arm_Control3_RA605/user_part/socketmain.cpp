#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "robot_position_controll.h"

using namespace std;
int main()
{
	int choice;
	SimpleRobotController src;
	while(1)
	{
		cout<<"==========menu=========="<<endl;
		cout<<"(1) setJointSpacePose(0.45,0.45,0.45,0.45,0.45,0.45);"<<endl;
		cout<<"(2) setCartesianSpacePose(0.12,0.36,0.11,1.57,-1.57,1.57);"<<endl;
		cin>>choice;
		if(choice==1)
			src.setJointSpacePose(0.45,0.45,0.45,0.45,0.45,0.45);
		if(choice==2)
			src.setCartesianSpacePose(0.12,0.36,0.11,1.57,-1.57,1.57);
		
	}
}
