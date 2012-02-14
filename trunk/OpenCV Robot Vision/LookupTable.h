#include <fstream>
using namespace std;

struct LookupTableMember
{
	// position variables
	int		X;
	int		Y;

	// actual data
	float	Speed;
	float	Tilt;
};

class LookupTable
{
private:
	char*			m_filename;
	ifstream		file;
	LookupTable*	table;
	int				tableSize;

public:
	LookupTable(char* filename);

	// runtime logic
	void FindShootingParams(LookupTableMember*);
};