#include "LookupTable.h"
#include <iostream>
#include <string>

LookupTable::LookupTable(char* filename)
{
	// init variables
	lookupTableSize = 0;
	m_filename = filename;

	// check to see if file exists
	file.open(filename, ios::in);
	if (!file.is_open())
	{
		cout<<"Could not open lookup table with name: " << filename << "!" << endl;
	}

	// read first line to get header information
	char* readBuffer = new char[100];
	file.getline(readBuffer, 100);

	char* itemBuffer = new char[10];

	for (; atoi(itemBuffer) != (INT_MAX | INT_MIN);)
		itemBuffer = strtok(readBuffer, ",");

	// convert to int
	lookupTableSize = atoi(itemBuffer);

	// allocate memory for lookup table
	lookupTable = new LookupTableSlot[lookupTableSize];

	// fill table with values
	for (int n = 0; n < lookupTableSize; n++)
	{
		file.getline(readBuffer, 100);

		itemBuffer = strtok(readBuffer, ",");
		lookupTable[n].Distance = (float)atof(itemBuffer);

		itemBuffer = strtok(readBuffer, ",");
		lookupTable[n].AngleOffset = (float)atof(itemBuffer);

		itemBuffer = strtok(readBuffer, ",");
		lookupTable[n].Speed = (float)atof(itemBuffer);

		itemBuffer = strtok(readBuffer, ",");
		lookupTable[n].Tilt = (float)atof(itemBuffer);

		itemBuffer = strtok(readBuffer, ",");
		lookupTable[n].PanOffset = (float)atof(itemBuffer);
	}
}

LookupTableOutput* LookupTable::FindShootingParams(LookupTableInput* input)
{
	// given the X and Y values in the member table, look for the member
	for (int n = 0; n < lookupTableSize; n++)
	{

	}

	return &lookupTable[0];
}

int LookupTable::GetTableSize()
{
	return lookupTableSize;
}