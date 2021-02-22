/** File:        camera_calibration.cpp
* Date:          20/04/20

* Description:   Calibration of camera
* Author:        Athena Petropoulou
* Modifications: Calibration of camera via chessboard images in simulation environment
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
#include <opencv2/video.hpp>

#define PI 3.14159265359

#define TIME_STEP 32
#define MAX_SPEED 5
#define CAMERA_HEIGHT 0.05

#define WHEEL_RADIUS 0.020
#define AXIS_LENGTH 0.052

using namespace std;
using namespace cv;

/***********************************************************************************************************/

struct Position
{
  double xPos, yPos, theta;
};

/***********************************************************************************************************/

static void compute_odometry(struct Position *pos, double Rspeed, double Lspeed);

static double computeReprojectionErrors(
    const vector<vector<Point3f>> &objectPoints,
    const vector<vector<Point2f>> &imagePoints,
    const vector<Mat> &rvecs, const vector<Mat> &tvecs,
    const Mat &cameraMatrix, const Mat &distCoeffs,
    vector<float> &perViewErrors);

/***********************************************************************************************************/

/***********************************************************************************************************/

int main()
{

  wb_robot_init();
  int timestep = wb_robot_get_basic_time_step();

  /* Initialize camera */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  static int width = wb_camera_get_width(camera);
  static int height = wb_camera_get_height(camera);

  //double FOV=wb_camera_get_fov(camera);
  //double FOV_dist=0;

  // get a handler to the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // set up the motor speeds
  wb_motor_set_velocity(left_motor, 0.0 * MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.0 * MAX_SPEED);

  vector<vector<Point3f>> object_points;
  vector<vector<Point2f>> image_points;

  int numBoards = 1;
  int numCornersHor = 5;
  int numCornersVer = 4;

  vector<Point3f> obj;
  int numSquares = numCornersHor * numCornersVer;
  Size board_sz = Size(numCornersVer, numCornersHor);

  for (int i = 0; i < numCornersHor; i++)
  {
    for (int j = 0; j < numCornersVer; j++)
    {
      obj.push_back(Point3f(j, i, 0.0f));
    }
  }

  Mat img = Mat(Size(width, height), CV_8UC4);
  Mat gray_image;

  wb_robot_step(TIME_STEP);
  vector<Point2f> corners;

  for (int i = 0; i < 23; i++)
  {

    string img_path = "calib_img/img" + to_string(i) + ".jpg";

    img = imread(img_path, IMREAD_COLOR);

    if (img.empty())
    {
      std::cout << "Could not read the image: " << img_path << std::endl;
      return 1;
    }

    cvtColor(img, gray_image, COLOR_BGR2GRAY);

    bool found = findChessboardCorners(gray_image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH);

    if (found)
    {

      cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.1));
      drawChessboardCorners(gray_image, board_sz, corners, found);
      imwrite("cornersFound/img" + to_string(i) + ".jpg", gray_image);

      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }

  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs;
  vector<Mat> tvecs;

  intrinsic.ptr<float>(0)[0] = 1;
  intrinsic.ptr<float>(1)[1] = 1;

  calibrateCamera(object_points, image_points, img.size(), intrinsic, distCoeffs, rvecs, tvecs);

  //Save intrinsic parameters to txt
  ofstream outStream("intrinsic");
  uint16_t rows = intrinsic.rows;
  uint16_t columns = intrinsic.cols;

  outStream << rows << endl;
  outStream << columns << endl;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < columns; c++)
    {
      double value = intrinsic.at<double>(r, c);
      outStream << value << endl;
    }
  }
  outStream.close();

  //Save coef parameters to txt
  ofstream outStreama("distCoeffs");
  uint16_t rowsa = distCoeffs.rows;
  uint16_t columnsa = distCoeffs.cols;

  outStreama << rowsa << endl;
  outStreama << columnsa << endl;

  for (int r = 0; r < rowsa; r++)
  {
    for (int c = 0; c < columnsa; c++)
    {
      double value = distCoeffs.at<double>(r, c);
      outStreama << value << endl;
    }
  }
  outStreama.close();

  cout << "Calibration finished" << endl;

  wb_robot_step(TIME_STEP);

  Mat img2 = Mat(Size(width, height), CV_8UC4);

  img2.data = (uchar *)wb_camera_get_image(camera);
  Mat imageUndistorted;

  undistort(img2, imageUndistorted, intrinsic, distCoeffs);
  imwrite("distorted.jpg", img2);
  imwrite("undistorted.jpg", imageUndistorted);

  namedWindow("win2", WINDOW_NORMAL);
  imshow("win2", imageUndistorted);

  cout << "Computing Reprojection Errors..." << endl;
  vector<float> perViewErrors;
  double repr_err = computeReprojectionErrors(object_points, image_points, rvecs, tvecs, intrinsic, distCoeffs, perViewErrors);

  cout << "Error= " << repr_err << endl;

  waitKey(TIME_STEP);

  //CLEANUP

  wb_robot_cleanup();
  return 0;
}

/***********************************************************************************************************/

static double computeReprojectionErrors(
    const vector<vector<Point3f>> &objectPoints,
    const vector<vector<Point2f>> &imagePoints,
    const vector<Mat> &rvecs, const vector<Mat> &tvecs,
    const Mat &cameraMatrix, const Mat &distCoeffs,
    vector<float> &perViewErrors)
{
  vector<Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); i++)
  {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                  cameraMatrix, distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err * err / n);

    totalErr += err * err;
    totalPoints += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

/***********************************************************************************************************/

static void compute_odometry(struct Position *pos, double Rspeed, double Lspeed)
{

  double l = Lspeed * TIME_STEP * 0.001;
  double r = Rspeed * TIME_STEP * 0.001;
  double dl = l * WHEEL_RADIUS;                  // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;                  // distance covered by right wheel in meter
  double deltaTheta = (dl - dr) / (AXIS_LENGTH); // delta orientation in rad

  double deltaStep = (dl + dr) / 2; // Expressed in meters.

  /*
  printf("estimated distance covered by left wheel: %f m.\n", dl);
  printf("estimated distance covered by right wheel: %f m.\n", dr);
  printf("estimated angle: %f rad.\n", deltaTheta);
  */

  pos->xPos += deltaStep * sin(pos->theta + deltaTheta / 2); // Expressed in meters.
  pos->yPos += deltaStep * cos(pos->theta + deltaTheta / 2); // Expressed in meters.

  pos->theta += deltaTheta; // Expressed in rad.

  //cout<<"x="<<pos->xPos <<"m  y="<<pos->yPos<<"m  angle="<<pos->theta*180/PI<<" deg"<<endl;
}
