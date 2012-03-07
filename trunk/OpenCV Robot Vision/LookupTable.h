#include <fstream>
using namespace std;

struct LookupTableInput
{
	float Distance;
	float AngleOffset;
};

struct LookupTableOutput
{
	float	Speed;
	float	Tilt;
	float	PanOffset;
};

// this has both input and output values
struct LookupTableSlot
	: public LookupTableInput, public LookupTableOutput
{};

class LookupTable
{
public:
	LookupTable(char* filename);

	// runtime logic
	LookupTableOutput* FindShootingParams(LookupTableInput*);

	int GetTableSize();

private:
	char*					m_filename;
	ifstream				file;
	LookupTableSlot*		lookupTable;
	int						lookupTableSize;
};