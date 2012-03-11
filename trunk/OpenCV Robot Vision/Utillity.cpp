#include "Utility.h"

NetworkDebuggingOutput::NetworkDebuggingOutput(char* ip, int port)
{
	WSAData wsaData;
	WORD version;
	int error;

	version = MAKEWORD(2,0);
	error = WSAStartup(version, &wsaData);
	
	if (error != 0)
	{
		cout << "Error: winsock initialization" << endl;
		return;
	}

	if (LOBYTE(wsaData.wVersion) != 2 ||
		HIBYTE(wsaData.wVersion) != 0)
	{
		WSACleanup();
		cout << "Error: incorrect winsock version" << endl;
		return;
	}

	// create send socket
	socket = socket(AF_INET, SOCK_DGRAM, 0);

	hostent* host;
	host = gethostbyname(ip);

	memset(&connectAddress, 0, sizeof connectAddress);

	connectAddress.sin_family = AF_INET;
	connectAddress.sin_addr.s_addr = ((in_addr*)(host->h_addr))->s_addr;
	connectAddress.sin_port = htons(port);

	// try and start connection
	if (connect(socket, (sockaddr*)&connectAddress, sizeof connectAddress) == SOCKET_ERROR)
	{
		cout << "Error: could not create datagram socket" << endl;
	}
}

void NetworkDebuggingOutput::Send(RectangleInformation rInformation)
{
	char* sendBuffer = new char[100];
	memset(sendBuffer, 0, sizeof sendBuffer);

	sprintf(sendBuffer, "%f, %f, %f, %f\r\n",
		rInformation.Distance, rInformation.AngleOffset,
		rInformation.RectangleCenter.X, rInformation.RectangleCenter.Y);

	// send the information
	send(socket, sendBuffer, strlen(sendBuffer), 0);
}

NetworkDebuggingOutput::~NetworkDebuggingOutput()
{
	// close the socket
	shutdown(socket, SD_BOTH);
	closesocket(socket);

	// shutdown winsock
	WSACleanup();
}