#include "DockingStationFinderSegmented.h"

DockingStationFinderSegmented::DockingStationFinderSegmented(){
	printf("segmentation based\n");

	//Create a line segment where the previous line segment ended and stop it after start + offset
	float start_x = 0;
	float start_y = 0;
	float stop_x = 0;
	float stop_y = 0.058;
	model.push_back(new HardCodedLine(start_x,start_y,stop_x,stop_y));//Tiny first piece

	start_x = stop_x;
	start_y = stop_y;
	stop_x += 0.1;
	stop_y += 0.1575;
	model.push_back(new HardCodedLine(start_x,start_y,stop_x,stop_y));//First long sloped piece

	start_x = stop_x;
	start_y = stop_y;
	stop_x += 0.0;
	stop_y += 0.045;
	model.push_back(new HardCodedLine(start_x,start_y,stop_x,stop_y));//Middle piece

	start_x = stop_x;
	start_y = stop_y;
	stop_x += -0.1;
	stop_y += 0.1575;
	model.push_back(new HardCodedLine(start_x,start_y,stop_x,stop_y));//Second long sloped piece, notice how x is negative

	start_x = stop_x;
	start_y = stop_y;
	stop_x += 0;
	stop_y += 0.058;
	model.push_back(new HardCodedLine(start_x,start_y,stop_x,stop_y));//Tiny end piece

}


DockingStationFinderSegmented::~DockingStationFinderSegmented(){
	for(unsigned int i = 0; i < model.size(); i++){delete model.at(i);}
}


vector<float>
DockingStationFinderSegmented::getMostLikelyLocation (vector<float> & laser_x, vector<float> & laser_y)
{
	vector<float> ret;
	ret.resize (4);
	float angles[laser_x.size()];
	float distances[laser_x.size()];
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

		distances[i] =lp;

		float angle = sxp*sxn+syp*syn;
		//printf("%i angle: %f\n",i,1+angle);
		angles[i] = 1+angle;
	}

	for (unsigned int i = 1; i+1 < laser_x.size(); i++)
	{
		int starti = i;
		
		float start_x = laser_x.at(i);
		float start_y = laser_y.at(i);
		while(i+1 < laser_x.size()){
			if(fabs(angles[i]) > 0.3){break;}
			i++;
		}

		float end_x = laser_x.at(i);
		float end_y = laser_y.at(i);

		float dx = start_x - end_x;
		float dy = start_y - end_y;	


		float segment_size = sqrt(dx*dx+dy*dy);

		dx /= segment_size;
		dy /= segment_size;

		float segment_length = 0;
		for(int j = starti; j < i ; j++){
			segment_length += distances[j];
		}

		float length_size_diff = segment_length - segment_size;
		float expected_diff = 0.53-0.467;
		if(segment_size > 0.3 && segment_size < 0.5 && fabs(length_size_diff-expected_diff) < 0.01){
			printf("--THIS---> ");

			ret.at (0) = end_x;
        	ret.at (1) = end_y;
        	ret.at (2) = acos(1*dx+0*dy);
        	ret.at (3) = 1000;
		}
		
		printf("s: %i ",starti);
		printf("size:%f ",segment_size);
		printf("length: %f ",segment_length);
		printf("d_exp %f ",fabs(length_size_diff-expected_diff));
		printf("e: %i\n",i);

		
	}
	

/*
	float slope_x[laser_x.size()-1];
	float slope_y[laser_x.size()-1];
	for (unsigned int i = 0; i+1 < laser_x.size(); i++)
	{
		float x = laser_x.at (i);
		float y = laser_y.at (i);
		float xn = laser_x.at (i+1);
		float yn = laser_y.at (i+1);
		float sx = x - xn;
		float sy = y - yn;
		float l = sqrt(sx*sx+sy*sy);
		sx/=l;
		sy/=l;
		slope_x[i] = sx;
		slope_y[i] = sy;
	}

	for (unsigned int i = 0; i+2 < laser_x.size(); i++)
	{
		float angle = slope_x[i]*slope_x[i+1] + slope_y[i]*slope_y[i+1];
		printf("%i:angle:%f\n",i+1,angle);
	}
*/
/*
  float best = -1;
  //tx
  //ty
  //rotation
  //score

  float noise = 0.005;
  float noise_sq = noise * noise;
  for (unsigned int i = 0; i < laser_x.size (); i++)
  {
    float start_x = laser_x.at (i);
    float start_y = laser_y.at (i);
    for (float angle = 0; angle < 2 * 3.14; angle += 0.05)
    {
      for (int k = 0; k < model.size (); k++)
      {
        model.at (k)->transform (start_x, start_y, angle);
      }
      float current_score = score (model, laser_x, laser_y, noise_sq);
      if (current_score > best)
      {
        best = current_score;
        if (ret.size () == 0)
        {
          ret.resize (4);
        }
        ret.at (0) = start_x;
        ret.at (1) = start_y;
        ret.at (2) = angle;
        ret.at (3) = current_score;
      }
    }
  }
*/
  return ret;
}
/*
float
DockingStationFinderSegmented::score (vector<HardCodedLine*> & model, vector<float> & x, vector<float> & y, float noise)
{
  if (model.size () == 0)
  {
    return 0;
  }//if there is no model return 0
  float total_score = 0;//Total score initiated at 0 but will be the total contribution of all points
  for (unsigned int i = 0; i < x.size (); i++)
  {
    float x_val = x.at (i);
    float y_val = y.at (i);
    float smallest = model.at (0)->distance (x_val, y_val);
    for (unsigned int j = 1; j < model.size (); j++)
    {
      float d = model.at (j)->distance (x_val, y_val);
      if (d < smallest)
      {
        smallest = d;
      }
    }

    //if(smallest < 2*noise)
    total_score += exp (-0.5 * smallest * smallest / noise);
    //total_score += exp (-0.5 * smallest * smallest / noise);//Score is based on a gaussian using the noise and the shortest distance a line in the model
  }
  return total_score;
}
*/
