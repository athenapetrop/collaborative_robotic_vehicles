/*
* File:          camera_trajectory.cpp
* Date:          15/05/20
* Description:   Υπολογισμός rotation και translation της κάμερας με βάση τα features (KLT)
* Author:        Athena Petropoulou
* Modifications:
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


//#include <eigen-master/Eigen/Dense>


#define PI 3.14159265359

#define TIME_STEP 32
#define MAX_SPEED 5
#define CAMERA_HEIGHT 0.025  

#define WHEEL_RADIUS 0.020 
#define AXIS_LENGTH 0.052

#define MAP_STEP 0.1


using namespace std;
using namespace cv;


/***********************************************************************************************************/

struct Position
{
  double xPos,zPos,theta;
};

/***********************************************************************************************************/

// FUNCTIONS 

vector<Point2f> shiTomasi (Mat img);

void lucasCanade (Mat img1, Mat img2, vector<Point2f> &frame1_features, vector<Point2f> &frame2_features);

void compute_traj(vector<Point2f> frame1_features,vector<Point2f> frame2_features, vector<Point2f> frame1_warp,
                  vector<Point2f> frame2_warp, struct Position *computed_pos,  Mat intrinsic);

void show_traj(struct Position *pos);

static void compute_odometry(struct Position *pos,double Rspeed,double Lspeed) ;

void readMatrix( string img_path , Mat &matr );

double square(double val);


/***********************************************************************************************************/

Mat traj = Mat::zeros(400, 400, CV_8UC3);


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
  wb_motor_set_velocity(left_motor, 0.1*MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.1*MAX_SPEED);
  
  /********************************************************************************/


  // Camera parameters

  double hor_FOV=wb_camera_get_fov(camera);
  double ver_FOV=2*atan(tan(hor_FOV/2)*height/width);
  
  double blindspot=CAMERA_HEIGHT / tan( ver_FOV/2);
 
  
  double focal_length=(double)width*0.5/tan(hor_FOV/2);  
 
  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs, homography;
     
  readMatrix("Homography",homography);
  readMatrix( "intrinsic" , intrinsic );
  readMatrix("distCoeffs", distCoeffs);
  
  float focal_x = intrinsic.at<double>(0,0);
  float focal_y = intrinsic.at<double>(1,1);
  float center_x = intrinsic.at<double>(0,2);
  float center_y = intrinsic.at<double>(1,2);

  /********************************************************************************/

  Position currpos={0,0,0};
  Position prevpos={0,0,0};
  Position computed_pos{0,0,0};

	
  vector<Point2f> frame1_features;
  vector<Point2f> frame2_features; 
	
  vector<Point2f> frame1_warp;
  vector<Point2f> frame2_warp;
   

  Rect croppedRectangle = Rect(50 , center_y-50 , 380 , 100);  
  
  Mat img1_warp, img2_warp;

  /********************************************************************************/


  // initialize frame
  
  wb_robot_step(TIME_STEP);
  Mat img1= Mat(Size(width,height),CV_8UC4);
  img1.data=(uchar *)wb_camera_get_image(camera);
  cvtColor(img1,img1,COLOR_BGR2GRAY);
  
  
  warpPerspective(img1,img1_warp,homography,img1.size());
  
  // shi tomasi
    
  frame1_features=shiTomasi(img1(croppedRectangle));
  frame1_warp=shiTomasi(img1_warp);
  compute_odometry(&currpos, 0.1*MAX_SPEED, 0.1*MAX_SPEED);
  int basic_count = 0;
 /********************************************************************************/
while(basic_count < 4){
  int counter = 0;
  
  // set up the motor speeds 
  wb_motor_set_velocity(left_motor, 0.1*MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.1*MAX_SPEED);
  while (wb_robot_step(TIME_STEP) != -1) 
  {
    

    // ground truth
    compute_odometry(&currpos, 0.1*MAX_SPEED, 0.1*MAX_SPEED); 
    cout<<"Ground Truth:    x="<<currpos.xPos<<"   z="<<currpos.zPos<<"   theta="<<currpos.theta <<endl;
    
    
    // grab frame 
    Mat img2= Mat(Size(width,height),CV_8UC4);
    img2.data=(uchar *)wb_camera_get_image(camera);
    
    // grayscale
    cvtColor(img2,img2,COLOR_BGR2GRAY);
    
  
    warpPerspective(img2,img2_warp,homography,img2.size());
    imwrite("warped.jpg",img2_warp);
    imwrite("bef_warp.jpg",img2);
    // Lucas Canade optical flow
    lucasCanade(img1(croppedRectangle),img2(croppedRectangle),frame1_features,frame2_features);
    lucasCanade(img1_warp , img2_warp, frame1_warp, frame2_warp);

    
    // compute trajectory
    compute_traj( frame1_features, frame2_features, frame1_warp, frame2_warp, &computed_pos, intrinsic);
       
    //Convert img2 to img1 for next iteration
    img1 = img2.clone();
    img1_warp = img2_warp.clone();
    
    frame1_features = frame2_features;
   // frame1_warp = frame2_warp;
    frame1_warp=shiTomasi(img2_warp);
    prevpos=currpos;
  
    //show trajectory 
    show_traj(&computed_pos); 
    counter++;	
	if(counter > 500){
        break;
      }	 
    if( frame1_features.size() < 15 || frame1_warp.size()<15 )
    {
      frame1_features = shiTomasi(img1(croppedRectangle));
      frame2_warp=shiTomasi(img2_warp);
      cout << "redetection" << endl ;
    }
		


     waitKey(TIME_STEP);
    
    
    
 
   }
       counter = 0;
         // set up the motor speeds 
  wb_motor_set_velocity(left_motor, 0.1*MAX_SPEED);
  wb_motor_set_velocity(right_motor,-0.1*MAX_SPEED);
  
  while (wb_robot_step(TIME_STEP) != -1) 
  {
    

    // ground truth
    compute_odometry(&currpos, 0.1*MAX_SPEED, -0.1*MAX_SPEED); 
    cout<<"Ground Truth:    x="<<currpos.xPos<<"   z="<<currpos.zPos<<"   theta="<<currpos.theta <<endl;
    
    
    // grab frame 
    Mat img2= Mat(Size(width,height),CV_8UC4);
    img2.data=(uchar *)wb_camera_get_image(camera);
    
    // grayscale
    cvtColor(img2,img2,COLOR_BGR2GRAY);
    
  
    warpPerspective(img2,img2_warp,homography,img2.size());
    
    // Lucas Canade optical flow
    lucasCanade(img1(croppedRectangle),img2(croppedRectangle),frame1_features,frame2_features);
    lucasCanade(img1_warp , img2_warp, frame1_warp, frame2_warp);

    
    // compute trajectory
    compute_traj( frame1_features, frame2_features, frame1_warp, frame2_warp, &computed_pos, intrinsic);
     
    //Convert img2 to img1 for next iteration
    img1 = img2.clone();
    img1_warp = img2_warp.clone();
    
    frame1_features = frame2_features;
   // frame1_warp = frame2_warp;
    frame1_warp=shiTomasi(img2_warp);
    prevpos=currpos;
  
    //show trajectory 
    show_traj(&computed_pos); 	
    if(counter>138){
        break;
      }counter++; 
    if( frame1_features.size() < 15 || frame1_warp.size()<15 )
    {
      frame1_features = shiTomasi(img1(croppedRectangle));
      frame2_warp=shiTomasi(img2_warp);
      cout << "redetection" << endl ;
    }
		


     waitKey(TIME_STEP);
    
    
    
 
   }
   
   basic_count++;
  }

  //CLEANUP
  
  wb_robot_cleanup();
  return 0;
 

}

/**************************************************************************************************************************/

vector<Point2f> shiTomasi (Mat img)
{

  vector<Point2f> frame_features;
  const int  minDistance = 10, blockSize = 15 ;
  double qualityLevel = 0.01 ; 
  const int number_of_features_tr = 30 ;
  
  goodFeaturesToTrack(img, frame_features, number_of_features_tr, qualityLevel, minDistance, noArray(), blockSize);
  
  
  return frame_features;
}

/**************************************************************************************************************************/

void lucasCanade (Mat img1, Mat img2, vector<Point2f> &frame1_features, vector<Point2f> &frame2_features)
{
  
  //Params for lucas kanade optical flow
  Size winSize = Size(18, 18) ;
  vector<uchar> status ;
  const int maxLevel = 2;
  Point2f p, q;
  Mat  err;
  int thresshold_tr = 20*20;
  
  
  
  if (!frame1_features.empty()) 
    { 
      calcOpticalFlowPyrLK(img1,img2,frame1_features,frame2_features,status,err,winSize,maxLevel,TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,10,0.03));
    }
    
    
    int indexCorrection = 0;
    for ( size_t i = 0; i < status.size(); i++) 
    {
      p.x = (int) frame1_features[i].x;
      p.y = (int) frame1_features[i].y;
      q.x = (int) frame2_features[i].x;
      q.y = (int) frame2_features[i].y;
      int feature_hypotenuse_square;
      feature_hypotenuse_square = ((p.y - q.y)*(p.y - q.y) + (p.x - q.x)*(p.x - q.x)); 
			
      //Erase bad points for KLT. Too large moton or out of camera optical field
      Point2f pt = frame2_features.at(i- indexCorrection);
      if( (feature_hypotenuse_square >= thresshold_tr) || (q.x < 0) || (q.y < 0) || (q.x > 480) || (q.y > 360) )
      {
        status.at(i) = 0;
        frame1_features.erase (frame1_features.begin() + (i - indexCorrection));
        frame2_features.erase (frame2_features.begin() + (i - indexCorrection));
        indexCorrection++;
      }
     }

   
}

/**************************************************************************************************************************/

void compute_traj(vector<Point2f> frame1_features,vector<Point2f> frame2_features, vector<Point2f> frame1_warp,vector<Point2f> frame2_warp, 
                  struct Position *computed_pos,  Mat intrinsic)
{
    
    
      
   float focal_x = intrinsic.at<double>(0,0);
   float center_x = intrinsic.at<double>(0,2);
    
    Point2f p, q;

    float prev_placement = 0 , cur_placement = 0;  
	
    float theta_point,theta_frame=0;
    
    float sumx=0, sumy=0;

    int counter = 1;

    for (size_t i = 0; i < frame2_features.size(); i++) 
    {
      p.x = frame1_features[i].x;
      p.y = frame1_features[i].y;
      q.x = frame2_features[i].x;
      q.y = frame2_features[i].y;
			
      counter += 1 ;
			
      //Convert to Cylindrical Coordinate System
      prev_placement = atan2( (p.x - center_x), focal_x);
      cur_placement = atan2( (q.x - center_x), focal_x);
      theta_point =  prev_placement - cur_placement;
		
      theta_frame +=  theta_point;
     }
       
     if(counter >1)
     {
       counter -= 1;
     }
       
    
       
     theta_frame = theta_frame ;
     theta_frame = theta_frame / counter;
		
     computed_pos->theta += theta_frame;
	
     
     
     counter = 1;
     for (size_t i = 0; i < frame2_warp.size(); i++) 
     {
        p.x = frame1_warp[i].x;
        p.y = frame1_warp[i].y;
        q.x = frame2_warp[i].x;
        q.y = frame2_warp[i].y;
        
        if((q.y - p.y)>0){
          sumx += (q.x - p.x);
          sumy += (q.y - p.y);	
          counter += 1 ;
        }		

      
     }
     
     if(counter >1)
     {
       counter -= 1;
     }
     
     sumx = sumx / (1760 * counter);  
     sumy = sumy / (1760 * counter);        
     
     float scale = 0;

     scale = sqrt(square(sumx) + square(sumy));

      
            
     computed_pos->xPos +=  scale*sin(computed_pos->theta);
     computed_pos->zPos +=  scale*cos(computed_pos->theta); 
     
     if(computed_pos->theta > PI && computed_pos->theta < 2*PI)
       computed_pos->theta -= 2*PI;
     else if(computed_pos->theta < -PI && computed_pos->theta > -2*PI)
       computed_pos->theta += 2*PI;
 
     cout<<"Visual Odometry:   x="<<computed_pos->xPos<<"   z="<<computed_pos->zPos<<"   theta= " << computed_pos->theta *180/PI <<endl;
     cout<<""<<endl;
     	
  	
     
}

/**************************************************************************************************************************/

void show_traj(struct Position *pos)
{
    char text[100];
    int fontFace = FONT_HERSHEY_PLAIN;
    double fontScale = 0.75;
    int thickness = 1;  
    Point textOrg(10, 50);
  
    int x = int(pos->xPos*400) + 150;
    int y =300- int(pos->zPos*400) ;
    circle(traj, Point(x, y) ,2, Scalar(255,0,0));

    rectangle( traj, Point(20, 30), Point(380, 80), CV_RGB(0,0,0), FILLED);
    sprintf(text, "Coords: x=%02fm  z=%02fm th=%02fdeg",pos->xPos, pos->zPos, pos->theta*180/PI);
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
    
    namedWindow( "Trajectory", WINDOW_NORMAL );
    imshow( "Trajectory", traj );
    imwrite("visual_odom.jpg", traj);
    waitKey(TIME_STEP);

}
/**************************************************************************************************************************/

void readMatrix( string img_path , Mat &matr )
{

   ifstream inStream( img_path );
   
   if(inStream){
     uint16_t rows;
     uint16_t columns;
    
     inStream >> rows;
     inStream >> columns;
    
     matr = Mat(Size(columns, rows), CV_64F);
    
     for(int r =0; r<rows; r++){
       for(int c =0; c<columns; c++){
         double read = 0.0f;
         inStream >> read;
         matr.at<double>(r,c) = read;
         //cout << matr.at<double>(r,c) << "\n" ;
       }
     }
     inStream.close();
   } 
  
} 

/**************************************************************************************************************************/

double square(double val)
{
  
    return val*val;
}

/**************************************************************************************************************************/

static void compute_odometry(struct Position *pos,double Rspeed,double Lspeed) 
{

  double l = Lspeed*TIME_STEP*0.001;
  double r = Rspeed*TIME_STEP*0.001;
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  double deltaTheta = (dl - dr) / (AXIS_LENGTH ) ;  // delta orientation in rad
  
  double deltaStep = (dl + dr)/2; // Expressed in meters.
  
  
  pos->xPos += deltaStep*sin(pos->theta + deltaTheta/2); // Expressed in meters.
  pos->zPos += deltaStep*cos(pos->theta + deltaTheta/2); // Expressed in meters.

  pos->theta += deltaTheta;
  

}  