/*
* File:          calibration_check.cpp
* Date:          11/05/20
* Description:   Testing of camera calibration using an object of known size
* Author:        Athena Petropoulou
*/

#include <stdio.h>
#include "iostream"
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include <iterator>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/device.h>
#include <webots/nodes.h>

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
#define CAMERA_HEIGHT 0.05

#define WHEEL_RADIUS 0.020
#define AXIS_LENGTH 0.052

#define MAP_STEP 0.1

using namespace std;
using namespace cv;
//using namespace Eigen;

/***********************************************************************************************************/

struct Position
{
  double xPos, zPos, theta;
};

/***********************************************************************************************************/

// FUNCTIONS

static void compute_odometry(struct Position *pos, double Rspeed, double Lspeed);

void readMatrix(string img_path, Mat &matr);

double square(double val);

Rect findContours(Mat frame);

void thresh_callback(int, void *);
/***********************************************************************************************************/

int thresh = 100;
int max_thresh = 255;
/***********************************************************************************************************/

int main()
{

  // BASIC ROBOT INIT

  /********************************************************************************/

  wb_robot_init();
  int timestep = wb_robot_get_basic_time_step();

  /* Initialize camera */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  static int width = wb_camera_get_width(camera);
  static int height = wb_camera_get_height(camera);

  // get a handler to the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // set up the motor speeds
  wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);

  /********************************************************************************/

  // Camera parameters

  double hor_FOV = wb_camera_get_fov(camera);
  double ver_FOV = 2 * atan(tan(hor_FOV / 2) * height / width);

  double deadzone = CAMERA_HEIGHT * tan(ver_FOV / 2);

  Point2d principalPoint(width / 2, height / 2);

  double focal_length = (double)width * 0.5 / tan(hor_FOV / 2);

  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;

  readMatrix("intrinsic", intrinsic);
  readMatrix("distCoeffs", distCoeffs);

  /********************************************************************************/

  Position mypos = {0, 0, 0};

  //undistort(frame_old, frame_old, intrinsic, distCoeffs);

  Rect object;

  // object real coords
  vector<Point3f> vec3d;
  vec3d.push_back(Point3f(0, 0, 0));
  vec3d.push_back(Point3f(0, 0.1, 0));
  vec3d.push_back(Point3f(0.1, 0.1, 0));
  vec3d.push_back(Point3f(0.1, 0, 0));

  while (wb_robot_step(TIME_STEP) != -1)
  {

    // ground truth
    compute_odometry(&mypos, 0.1 * MAX_SPEED, 0.1 * MAX_SPEED);
    cout << "x=" << mypos.xPos << "   y=" << 0.3 - mypos.zPos << endl;

    Mat img = Mat(Size(width, height), CV_8UC4);

    // grab frame
    img.data = (uchar *)wb_camera_get_image(camera);

    cvtColor(img, img, COLOR_BGR2GRAY);

    namedWindow("Frame", WINDOW_NORMAL);
    imshow("Frame", img);

    createTrackbar(" Canny thresh:", "Frame", &thresh, max_thresh, thresh_callback);

    object = findContours(img);

    // object pixel coords
    vector<Point2f> vec2d;
    vec2d.push_back(Point2f(object.x, object.y));
    vec2d.push_back(Point2f(object.x, object.y + object.height));
    vec2d.push_back(Point2f(object.x + object.width, object.y + object.height));
    vec2d.push_back(Point2f(object.x + object.width, object.y));

    //The pose of the object: rvec is the rotation vector, tvec is the translation vector
    Mat rvec, tvec;
    solvePnP(vec3d, vec2d, intrinsic, distCoeffs, rvec, tvec);

    double d = sqrt(square(tvec.at<double>(0)) + square(tvec.at<double>(2)));

    cout << "Distance:" << d << endl;

    cout << "Error is : " << fabs(d - (0.3 - fabs(sqrt(square(mypos.xPos) + square(mypos.zPos))))) << endl;
    cout << " " << endl;

    waitKey(TIME_STEP);
  }

  //CLEANUP

  wb_robot_cleanup();
  return 0;
}

/**************************************************************************************************************************/

void thresh_callback(int, void *)
{
}

/**************************************************************************************************************************/

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
}

/**************************************************************************************************************************/

double square(double val)
{

  return val * val;
}

/**************************************************************************************************************************/

static void compute_odometry(struct Position *pos, double Rspeed, double Lspeed)
{

  double l = Lspeed * TIME_STEP * 0.001;
  double r = Rspeed * TIME_STEP * 0.001;

  // distance covered by left wheel in meter
  double dl = l * WHEEL_RADIUS;

  // distance covered by right wheel in meter
  double dr = r * WHEEL_RADIUS;

  // delta orientation in rad
  double deltaTheta = (dl - dr) / (AXIS_LENGTH);

  double deltaStep = (dl + dr) / 2; // Expressed in meters.

  pos->xPos += deltaStep * sin(pos->theta + deltaTheta / 2); // Expressed in meters.
  pos->zPos += deltaStep * cos(pos->theta + deltaTheta / 2); // Expressed in meters.

  pos->theta += deltaTheta; // Expressed in rad.
}

/**************************************************************************************************************************/

Rect findContours(Mat frame)
{
  Mat thresholded;
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  Mat frame2 = frame.clone();

  blur(frame, thresholded, Size(3, 3));

  Canny(thresholded, thresholded, thresh, 2 * thresh, 3);

  // Find contours
  findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Rect> boundRect(contours.size());

  /// Draw contours
  Mat drawing = Mat::zeros(thresholded.size(), CV_8UC3);
  for (size_t i = 0; i < contours.size(); i++)
  {
    boundRect[i] = boundingRect(contours[i]);
    rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 0, 255), 2);
  }

  namedWindow("contours", WINDOW_NORMAL);
  imshow("contours", drawing);

  waitKey(TIME_STEP);

  return boundRect[0];
}
