/*   @File: util.cpp
*    @Details: Utility functions to be used in all the scope
*/

void readMatrix(string img_path, Mat &matr)
{

  ifstream inStream(img_path);

  if (inStream)
  {
    uint16_t rows;
    uint16_t columns;

    inStream >> rows;
    inStream >> columns;

    matr = Mat(Size(columns, rows), CV_64F);

    for (int r = 0; r < rows; r++)
    {
      for (int c = 0; c < columns; c++)
      {
        double read = 0.0f;
        inStream >> read;
        matr.at<double>(r, c) = read;
      }
    }
    inStream.close();
  }
  return;
}

/**************************************************************************************************************************/

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

void initCameraParams(struct CamParam *cam){
    cam->camera = wb_robot_get_device("camera");
    wb_camera_enable(cam->camera, TIME_STEP);
    cam->width = wb_camera_get_width(cam->camera);
    cam->height = wb_camera_get_height(cam->camera);
  
    // Camera parameters
    cam->hor_FOV = wb_camera_get_fov(cam->camera);
    cam->ver_FOV = 2 * atan(tan(cam->hor_FOV / 2) * cam->height / cam->width);
  
    cam->blindspot = CAMERA_HEIGHT / tan(cam->ver_FOV / 2);
    cam->intrinsic = Mat(3, 3, CV_32FC1);
  
  
    readMatrix("camera_parameters/intrinsic", cam->intrinsic);
    readMatrix("camera_parameters/distCoeffs", cam->distCoeffs);
  
    cam->focal_x = cam->intrinsic.at<double>(0, 0);
    cam->focal_y = cam->intrinsic.at<double>(1, 1);
    cam->center_x = cam->intrinsic.at<double>(0, 2);
    cam->center_y = cam->intrinsic.at<double>(1, 2);
    
    return;
}

void initOdom(struct Odometry *odom){
    odom->left_motor = wb_robot_get_device("left wheel motor");
    odom->right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(odom->left_motor, INFINITY);
    wb_motor_set_position(odom->right_motor, INFINITY);
  
    // position sensors
    
    odom->left_ps = wb_robot_get_device("left wheel sensor");
    odom->right_ps = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(odom->left_ps, TIME_STEP);
    wb_position_sensor_enable(odom->right_ps, TIME_STEP);
    
    return;
}