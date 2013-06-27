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
	float max_angle_diff;//max dot product between slopes(if larger than this then segment)
	float min_segment_size;//The minimum distance between the first and the last point of the segment
	float max_segment_size;//The maximum distance between the first and the last point of the segment
	float max_diff_from_expected;//Makes sure there is a certain difference between length of segment and distance between first and last point
	float expected_diff;//Expected difference between length of segment and distance between first and last point

	DockingStationFinderSegmented();
	~DockingStationFinderSegmented();

	//Finds likely corner of the chargeing station
	vector<float> getMostLikelyLocation(vector<float> & laser_x, vector<float> & laser_y);
};

#endif
