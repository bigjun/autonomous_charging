#include "HardCodedLine.h"
#include "math.h"
int HardCodedLine_counter = 0;

HardCodedLine::HardCodedLine(float &x1, float &y1, float &x2, float &y2)
{
	  point_x1=x1;
	  point_y1=y1;
	  point_x2=x2;
	  point_y2=y2;
	  transformed_point_x1=point_x1;
	  transformed_point_x2=point_x2;
	  transformed_point_y1=point_y1;
	  transformed_point_y2=point_y2;
	  tx=(transformed_point_x2-transformed_point_x1);
	  ty=(transformed_point_y2-transformed_point_y1);
	  length=sqrt(tx*tx+ty*ty);
	  tx/=length;
	  ty/=length; 
	  id = HardCodedLine_counter++;
}

HardCodedLine::HardCodedLine(){
	id = HardCodedLine_counter++;
	  point_x1=0;
	  point_y1=0;
	  point_x2=1;
	  point_y2=0;
	  transformed_point_x1=point_x1;
	  transformed_point_x2=point_x2;
	  transformed_point_y1=point_y1;
	  transformed_point_y2=point_y2;
	  tx=(transformed_point_x2-transformed_point_x1);
	  ty=(transformed_point_y2-transformed_point_y1);
	  length=sqrt(tx*tx+ty*ty);
}
HardCodedLine::~HardCodedLine(){}


void HardCodedLine::transform(float & translate_x, float & translate_y, float & r){
	  transformed_point_x1=(point_x1-translate_x)*cos(r)+(point_y1-ty)*sin(r);
	  transformed_point_x2=(point_x2-translate_x)*cos(r)+(point_y2-ty)*sin(r);
	  transformed_point_y1=(point_y1-translate_y)*cos(r)-(point_x1-tx)*sin(r);
	  transformed_point_y2=(point_y2-translate_y)*cos(r)-(point_x2-tx)*sin(r);
	  tx=(transformed_point_x2-transformed_point_x1);
	  ty=(transformed_point_y2-transformed_point_y1);
	  tx/=length;
	  ty/=length;
	}


float HardCodedLine::distance(float & x, float & y){
	  float d1=sqrt((x-transformed_point_x1)*(x-transformed_point_x1)+(y-transformed_point_y1)*(y-transformed_point_y1));
	  float d2=sqrt((x-transformed_point_x2)*(x-transformed_point_x2)+(y-transformed_point_y2)*(y-transformed_point_y2));
	  if (d1>d2) d1=d2;
	  float dot=((x-transformed_point_x1)*tx+(y-transformed_point_y1)*ty);
	  if ((dot<length)&&(dot>0)){
	    float d3=(((x-transformed_point_x1)*ty-(y-transformed_point_y1)*tx));
		if (d3<0) d3=-d3;
		if (d1>d3) d1=d3;
	}	
	  return d1;
	}

