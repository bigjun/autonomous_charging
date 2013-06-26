#ifndef HardCodedLine_H_
#define HardCodedLine_H_
class HardCodedLine
{
	public:
	int id;

	//Base parameters of the line
	float point_x1;
	float point_Y1;
	float point_x2;
	float point_y2;

	//Transformed the base-points of the line, this will be used to test different transformations of the line.
	float transformed_point_x1;
	float transformed_point_y1;
	float transformed_point_x2;
	float transformed_point_y2;
	float tx;
	float ty;
	float length;
	//Input the start and the stop of the line
	HardCodedLine(float x1, float y1, float x2, float y2);

	//Standard constructor, does nothing exept give the segment an id.	
	HardCodedLine()	;
	//Standard destructor
	~HardCodedLine();

	//Transforms the points of the line so that we can test different transformations of models.
	void transform(float tx, float ty, float r);
	//Distance to line between transformed points
	  float distance(float x, float y);
};
#endif
