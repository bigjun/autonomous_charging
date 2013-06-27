#ifndef DockingStationFinder_H_
#define DockingStationFinder_H_
#include "HardCodedLine.h"
#include <vector>
#include <stdio.h>
#include <cmath>

using namespace std;

class DockingStationFinder
{
	public:
	DockingStationFinder();
	virtual ~DockingStationFinder();


	virtual vector<float> getMostLikelyLocation(vector<float> & laser_x, vector<float> & laser_y);
	private:
	vector<HardCodedLine*> model;
	float score(vector<HardCodedLine*> & model, vector<float> & x, vector<float> & y, float noise);

};

#endif
