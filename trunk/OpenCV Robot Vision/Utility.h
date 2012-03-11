#include <WinSock2.h>
#include "RobotVision.h"

class NetworkDebuggingOutput
{
public:
	NetworkDebuggingOutput(char*, int);

	void Send(RectangleInformation);

	~NetworkDebuggingOutput();

private:
	SOCKET socket;
	sockaddr_in connectAddress;
};