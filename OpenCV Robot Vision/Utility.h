#ifndef UTILITY_H
#define UTILITY_H

//#include <WinSock2.h>
#include "RobotVision.h"

class NetworkDebuggingOutput
{
public:
	NetworkDebuggingOutput(char*, int);

	void Send(RectangleInformation);

	~NetworkDebuggingOutput();

private:
	SOCKET Socket;
	sockaddr_in connectAddress;
};

#endif