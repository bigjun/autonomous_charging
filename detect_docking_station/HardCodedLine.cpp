#include "HardCodedLine.h"
int HardCodedLine_counter = 0;

HardCodedLine::HardCodedLine(float x1, float y1, float x2, float y2)
{
	id = HardCodedLine_counter++;
	point_x1 = x1;
	point_Y1 = y1;
	point_x2 = x2;
	point_y2 = y2;

	transformed_point_x1 = x1;
	transformed_point_y1 = y1;
	transformed_point_x2 = x2;
	transformed_point_y2 = y2;
}

HardCodedLine::HardCodedLine(){id = HardCodedLine_counter++;}
HardCodedLine::~HardCodedLine(){}

//TODO
void HardCodedLine::transform(float tx, float ty, float r){}

//TODO
float HardCodedLine::distance(float x, float y){return -1;}

