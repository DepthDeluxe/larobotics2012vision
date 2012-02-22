#include <fstream>
using namespace std;

struct LookupTableMember
{
	// position variables
	float	Distance;
	float	AngleOffset;

	// actual data
	float	Speed;
	float	Tilt;
	float	PanOffset;
};

class LookupTable
{
public:
	LookupTable(char* filename);

	// runtime logic
	void FindShootingParams(LookupTableMember*);

private:
	char*					m_filename;
	ifstream				file;
	LookupTableMember*		lookupTable;
	int						lookupTableSize;
};