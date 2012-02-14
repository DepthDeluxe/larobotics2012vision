#include <WinSock2.h>

struct RobotVisionPacket
{
	float speed;
	float pan;
	float tilt;
};

class RobotVisionNetworking
{
private:
	SOCKET			client;
	sockaddr_in		connectTarget;

public:
	// constructor
	RobotVisionNetworking();

	// runtime operations
	void SendData(RobotVisionPacket);

	// closing operations
	void Shutdown();
};