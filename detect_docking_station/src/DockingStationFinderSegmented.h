#ifndef DockingStationFinderSegmented_H_
#define DockingStationFinderSegmented_H_
#include "HardCodedLine.h"
#include <vector>
#include <stdio.h>
#include <cmath>
#include "DockingStationFinder.h"

using namespace std;

class DockingStationFinderSegmented : public DockingStationFinder
{
	public:
	DockingStationFinderSegmented();
	~DockingStationFinderSegmented();

	vector<float> getMostLikelyLocation(vector<float> & laser_x, vector<float> & laser_y);
	private:
	vector<HardCodedLine*> model;
	//float score(vector<HardCodedLine*> & model, vector<float> & x, vector<float> & y, float noise);

};

#endif
