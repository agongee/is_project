
struct traj{

	/*
		Not only the position of the RC car (struct point), but also how the RC car reach to the position
	*/ 

	double x;
	double y;
	double th; // heading angle of RC car on the point
	double d; // v * delta_t --> local movement during delta_t
	double alpha; // Steering angle of front wheels of the RC car
};
