#include "RobotVisionNetworking.h"
#include <stdio.h>

RobotVisionNetworking::RobotVisionNetworking()
{ 
	// init winsock
	WSADATA wsaData;
	WORD version;
	int error;

	version = MAKEWORD(2,0);

	error = WSAStartup(version, &wsaData);

	// check for error
	if (error != 0)
		return;

	// check for improper winsock version
	if (LOBYTE(wsaData.wVersion) != 2 ||
		HIBYTE(wsaData.wVersion) != 0)
	{
		WSACleanup();
		return;
	}

	// set up socket
	client = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	// connect to server -- on the robot
	struct hostent* host;
	host = gethostbyname("10.1.14.2");

	memset(&connectTarget, 0, sizeof connectTarget);

	connectTarget.sin_family = AF_INET;
	connectTarget.sin_addr.s_addr = ((struct in_addr *)(host->h_addr))->s_addr;
	connectTarget.sin_port = htons(114);

	// connect to target
	if (connect(client, (sockaddr*)&connectTarget, sizeof connectTarget) == SOCKET_ERROR)
	{
		// throw connection exception
		WSACleanup();
	}
}

void RobotVisionNetworking::SendData(RobotVisionPacket packet)
{
	char* dataBuffer =  new char[30];
	dataBuffer[0] = 0;

	sprintf(&dataBuffer[0], "%f,%f,%f\r\n", packet.speed, packet.pan, packet.tilt);

	send(client, dataBuffer, strlen(dataBuffer), 0);
}

void RobotVisionNetworking::Shutdown()
{
	// close the socket
	shutdown(client, SD_BOTH);
	closesocket(client);

	// perform WSACleanup
	WSACleanup();
}