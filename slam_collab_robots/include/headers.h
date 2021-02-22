#include <stdio.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>

#include <cmath>
#include <fstream> 
#include <sstream>
#include <iterator>
#include <algorithm>
#include <list>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/device.h>
#include <webots/nodes.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include <opencv2/opencv_modules.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video.hpp>

#define PI 3.14159265359

#define TIME_STEP 32
#define MAX_SPEED 5
#define CAMERA_HEIGHT 0.028

#define ROBOT_RADIUS 0.037
#define WHEEL_RADIUS 0.020
#define AXIS_LENGTH 0.057 
#define EPUCK_HEIGHT 0.0486
#define wheel_diameter_left 0.04
#define wheel_diameter_right 0.04

#define WIDTH_STEP 5
#define MAP_SZ 100
#define OCC_THRESH 10
#define FREE_THRESH -40

/* Communication signals */
#define SAW_YOU 1
#define FALSE_ALARM 2
#define GOT_IT 3
#define CONTINUE 4
#define SEND_MAP 5
#define MAP_SENT 6
#define SENDING_MERGED 7
#define SEND_POS 8
#define POS_SENT 9
#define WAIT 10
#define GO 11
#define FINISHED 12

#define GROUND_TRUTH 1

#define FRAME_WIDTH 480
#define FRAME_HEIGHT 360

#define ASTAR 0

#define BLIND 0.0746426

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


/***********************************************************************************************************/

struct Position
{
  double xPos,zPos,theta;
};

/***********************************************************************************************************/

struct Encoders{
  double left,right;
};

/***********************************************************************************************************/

//struct for the log-odd maps
struct Map
{
  Mat map = Mat::zeros(MAP_SZ, MAP_SZ, CV_64F);   //occupancy log map
  Mat map_draw = Mat::zeros(MAP_SZ, MAP_SZ, CV_8UC3);   //occupancy draw map

  Point id;
  Point global_id;
};

/***********************************************************************************************************/

struct Odometry
{
  WbDeviceTag left_motor, right_motor;
  WbDeviceTag left_ps, right_ps;
};

struct CamParam
{
  WbDeviceTag camera;
  int width, height;
  double hor_FOV, ver_FOV;
  double blindspot;
  Mat intrinsic, distCoeffs;
  float focal_x, focal_y, center_x, center_y;
};

