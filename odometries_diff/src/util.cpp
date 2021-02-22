
double square(double val)
{

  return val * val;
}

/**************************************************************************************************************************/

double dist(Point2f point1, Point2f point2)
{
  return sqrt(square(point1.x - point2.x) + square(point1.y - point2.y));
}
/**************************************************************************************************************************/

int readRobotId()
{
    char *robot_name;
    int robot_id;
    robot_name = (char *)wb_robot_get_name();
  
    sscanf(robot_name, "robot%d", &robot_id);
    
    return robot_id;
    
}