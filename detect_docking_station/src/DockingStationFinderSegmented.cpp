#include "DockingStationFinderSegmented.h"

DockingStationFinderSegmented::DockingStationFinderSegmented(){
	printf("segmentation based\n");

	max_angle_diff = 0.3;
	min_segment_size = 0.3;
	max_segment_size = 0.5;

	max_diff_from_expected = 0.01;
	expected_diff = 0.53-0.467;//Measured difference between arc-lenght(53 cm) and width(46.7 cm) of charger
}


DockingStationFinderSegmented::~DockingStationFinderSegmented(){}

vector<float>
DockingStationFinderSegmented::getMostLikelyLocation (vector<float> & laser_x, vector<float> & laser_y)
{
	vector<float> ret;
	ret.resize (4);
	float angles[laser_x.size()];
	float distances[laser_x.size()];
//Extract angles and distances between neighbouring points
	for (unsigned int i = 1; i+1 < laser_x.size(); i++)
	{
		float xn = laser_x.at (i-1);
		float yn = laser_y.at (i-1);
		float x = laser_x.at (i);
		float y = laser_y.at (i);
		float xp = laser_x.at (i+1);
		float yp = laser_y.at (i+1);

		float sxn = xn-x;
		float syn = yn-y;
		float ln = sqrt(sxn*sxn + syn*syn);
		sxn /= ln;
		syn /= ln;

		float sxp = xp-x;
		float syp = yp-y;
		float lp = sqrt(sxp*sxp + syp*syp);
		sxp /= lp;
		syp /= lp;

		distances[i]	= lp;
		angles[i]		= 1+sxp*sxn+syp*syn;
	}

	//Find likely segments
	for (unsigned int i = 1; i+1 < laser_x.size(); i++)
	{
		int starti = i;
		
		float start_x = laser_x.at(i);
		float start_y = laser_y.at(i);
		while(i+1 < laser_x.size()){
			if(fabs(angles[i]) > max_angle_diff){break;}
			i++;
		}

		float end_x = laser_x.at(i);
		float end_y = laser_y.at(i);

		float dx = start_x - end_x;
		float dy = start_y - end_y;	


		float segment_size = sqrt(dx*dx+dy*dy);

		dx /= segment_size;
		dy /= segment_size;

		float segment_arc_length = 0;
		for(int j = starti; j < i ; j++){segment_arc_length += distances[j];}

		float length_size_diff = segment_arc_length - segment_size;
		if(segment_size > min_segment_size && segment_size < max_segment_size && fabs(length_size_diff-expected_diff) < max_diff_from_expected){//Likely segment
			ret.at (0) = (end_x+start_x)/2;
        	ret.at (1) = (end_y+start_y)/2; 
        	ret.at (2) = acos(1*dx+0*dy);
        	ret.at (3) = 1000;
		}
	}
  return ret;
}
