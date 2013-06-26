#include "DockingStationFinder.h"

DockingStationFinder::DockingStationFinder(){

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


DockingStationFinder::~DockingStationFinder(){
	for(unsigned int i = 0; i < model.size(); i++){delete model.at(i);}
}


vector<float> DockingStationFinder::getMostLikelyLocation(vector<float> & laser_x, vector<float> & laser_y){
	vector<float> ret;
	//tx
	//ty
	//rotation
	//score
	return ret;
}

float DockingStationFinder::score(vector<HardCodedLine*> model, vector<float> x, vector<float> y, float noise){
	if(model.size() == 0){return 0;}//if there is no model return 0
	float total_score = 0;//Total score initiated at 0 but will be the total contribution of all points	
	for(unsigned int i = 0; i < x.size(); i++){
		float x_val = x.at(i);
		float y_val = y.at(i);
		float smallest = model.at(0)->distance(x_val,y_val);
		for(unsigned int j = 1; j < model.size(); j++){
			float d = model.at(j)->distance(x_val,y_val);
			if(d < smallest){smallest = d;}
		}
		total_score += exp(-0.5*smallest*smallest/noise);//Score is based on a gaussian using the noise and the shortest distance a line in the model
	}
	return total_score;
}

