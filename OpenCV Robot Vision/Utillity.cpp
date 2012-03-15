#ifndef UTILITY_CPP
#define UTILITY_CPP

#include "Utility.h"

NetworkDebuggingOutput::NetworkDebuggingOutput(char* address, int port)
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
	consoleSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset(&connectAddress, 0, sizeof connectAddress);

	LPHOSTENT host;
	host = gethostbyname(address);

	// need to change back to connect socket
	connectAddress.sin_family = AF_INET;
	connectAddress.sin_addr.s_addr = ((in_addr*)(host->h_addr))->s_addr;
	connectAddress.sin_port = htons(port);

	// try and start connection
	if (connect(consoleSocket, (sockaddr*)&connectAddress, sizeof connectAddress) == SOCKET_ERROR)
	{
		int lastError = WSAGetLastError();
		WSACleanup();
		cout << "Error: " << lastError << endl;
		cout << "Error: could not create console socket!" << endl;
		return;
	}	
}

void NetworkDebuggingOutput::Send(RectangleInformation rInformation)
{
	char* sendBuffer = new char[100];
	memset(sendBuffer, 0, sizeof sendBuffer);

	sprintf(sendBuffer, "%f, %f, %f, %f\r\n",
		rInformation.Distance, rInformation.AngleOffset,
		rInformation.RectangleCenter.X, rInformation.RectangleCenter.Y);

	// format output so it prints very nice!
	bool foundEnd = false;
	for (int n = 0; (!foundEnd && n < 99) || (foundEnd && n < 100); n++)
	{
		if (sendBuffer[n] == '\r' && sendBuffer[n+1] == '\n')
			foundEnd = true;

		if (foundEnd)
			sendBuffer[n] = 0;
	}

	// send the information
	int bytesSent = 0;
	bytesSent = send(consoleSocket, sendBuffer, strlen(sendBuffer), 0);

	if (bytesSent < 0)
	{
		cout << "Winsock Error: " << WSAGetLastError() << endl;
	}
}

NetworkDebuggingOutput::~NetworkDebuggingOutput()
{
	// close the socket
	shutdown(consoleSocket, 0x02);
	closesocket(consoleSocket);

	// shutdown winsock
	WSACleanup();
}

#endif