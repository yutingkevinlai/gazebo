#include "robot_position_controll.h"


 SocketClient::SocketClient()
    {

    	//WSAStartup(MAKEWORD(2,1),&wsaData);
    	len = sizeof(server);
    	//strcpy(ip,"127.0.0.1");
    	server.sin_family = AF_INET;
    	server.sin_port = htons(12345);  //設定port
    	std::string IP("127.0.0.1");
    	server.sin_addr.s_addr = inet_addr(IP.c_str()/*ip*/);
    	socket1 = socket(AF_INET,SOCK_DGRAM,0);//設為UDP
    }
    void SocketClient::Send(char * m_data)
    {
    	std::string data_string(m_data);
    	//strcpy(sentdata,m_data);
    	sendto(socket1,data_string.c_str(),sizeof(data_string.c_str())*strlen(data_string.c_str()),0,(struct sockaddr*)&server,len);
    	////////////////////////////////////////↑這裡要乘上字串的長度,否則 sizeof(sentdata)就只是這指標的大小而已 :4
    }
    void SocketClient::CloseSocket()
    {
    	close(socket1);
    }


    char* SocketServer::Receive()//這是block function
    {
        std::cout<<recvfrom(socket1,buffer,sizeof(buffer),0,(struct sockaddr*)&client,&len) ;
        return buffer;//雖然是宣告靜態陣列 但回傳是指標 ,要用char*收 char[512]不行
    }
    void SocketServer::CloseSocket()
    {
        close(socket1);
    }
    SocketServer::~SocketServer()
    {
    	std::cout<<"~SocketServer()"<<std::endl;
    	CloseSocket();
    }
    SocketServer::SocketServer()
    {
        //WSAStartup(MAKEWORD(2,1),&wsaData);//初始windows socket application,並設定版本MAKEWORD(2,1)
        len = sizeof(client);
        local.sin_family = AF_INET;
        local.sin_port = htons(12345);  //設定port
        std::string IP("127.0.0.1");
        local.sin_addr.s_addr = INADDR_ANY;//inet_addr(IP.c_str());
        socket1 = socket(AF_INET,SOCK_DGRAM,0);//設定為UDP
        memset(buffer, '\0',80);
        bind(socket1,(struct sockaddr*)&local,sizeof(local));

    }


void SimpleRobotController::setJointSpacePose(float j1,float j2,float j3,float j4,float j5,float j6)
{
	char  senddata[80]="\0";
	SocketClient sc;
	memset(senddata,'\0',80);
	sprintf(senddata,"j %f %f %f %f %f %f",j1,j2,j3,j4,j5,j6);
	sc.Send(senddata);
	
	
	//memset(sentdata,"\0",80);
	//sprintf(sentdata,"end");
	//sc.Send(sentdata);	
}
void SimpleRobotController::setCartesianSpacePose(float x,float y,float z,float a,float b,float c)
{
	char  senddata[80]="\0";
	SocketClient sc;
	memset(senddata,'\0',80);
	sprintf(senddata,"c %f %f %f %f %f %f",x,y,z,a,b,c);
	sc.Send(senddata);
	
	
	//memset(sentdata,"\0",80);
	//sprintf(sentdata,"end");
	//sc.Send(sentdata);

}


