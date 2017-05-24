#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>

using namespace std;

class SocketClient
{
private:
    int socket1;//建立socket
    //WSADATA wsaData;
    char* ip;//server的位址
    struct sockaddr_in server;
    socklen_t len ;

public:
    SocketClient();
    void Send(char * m_data);
    void CloseSocket();
};
class SocketServer
{
public:
    char* Receive();
    void CloseSocket();
    ~SocketServer();
    SocketServer();
private:
    int socket1;//建立socket
    //WSADATA wsaData;
    struct sockaddr_in local;
    struct sockaddr_in client;
    socklen_t len;
    char buffer[80];
};



class SimpleRobotController
{
public:
	void setJointSpacePose(float j1,float j2,float j3,float j4,float j5,float j6);
	void setCartesianSpacePose(float x,float y,float z,float a,float b,float c);
	//not implement yet
	void goHome();
	void setJointSpeed(float j1,float j2,float j3,float j4,float j5,float j6);
	
private:
	float joint_speed[6];
	float home_position[6];

};
















