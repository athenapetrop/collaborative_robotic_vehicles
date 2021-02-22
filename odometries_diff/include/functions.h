// FUNCTIONS 

double square (double val);
                   
double dist(Point2f point1, Point2f point2);

int readRobotId();

bool robotRecognition(Mat img, Rect &robot);

double findOtherRobotsDist(struct Position pos_1, Rect robot, double focal_y);

