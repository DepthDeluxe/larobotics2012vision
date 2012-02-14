#include "LookupTable.h"
#include <iostream>
#include <string>

LookupTable::LookupTable(char* filename)
{
	// init variables
	tableSize = 0;
	m_filename = filename;

	// check to see if file exists
	file.open(filename, ios::in);
	if (!file.is_open())
	{
		cout<<"Could not open lookup table with name: " << filename << "!" << endl;
	}

	// dig through file to find number of members
	int memberCount = 0;

	for (int memberCount = 0; !file.eof(); memberCount++)
	{
	}
}

void LookupTable::FindShootingParams(LookupTableMember* member)
{
	// given the X and Y values in the member table, look for the member
	for (int n = 0; n < tableSize; n++)
	{

	}
}