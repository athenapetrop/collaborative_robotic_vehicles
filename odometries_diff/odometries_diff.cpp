// File:          odometries_diff.cpp
// Date:          01/02/2021
// Description:   Differencies between wheel odometry, visual odometry and ground truth
// Author:        Athena Petropoulou
// Modifications:

#include <include/headers.h>
#include <include/functions.h>
#include <src/odometry.cpp>
#include <src/cameraParams.cpp>
#include <src/util.cpp>


/***********************************************************************************************************/

int main()
{
  
  // BASIC ROBOT INIT
  
  Odometry odom;
  odom.init();

  CameraParams camera;
  camera.init();
  
  vector<Point2f> frame1_features;
  vector<Point2f> frame2_features; 
  vector<uchar> status ;

  int basic_count = 0;
  odom.setSpeed(0, 0);
  wb_robot_step(TIME_STEP);
   
  camera.grabFrame();
  frame1_features=camera.shiTomasi();
  camera.prevImg();
  while(basic_count < 4){
    int counter = 0;
    odom.setSpeed(0.1 * MAX_SPEED, 0.1 * MAX_SPEED);
    while (wb_robot_step(TIME_STEP) != -1)
    {
      
      odom.computePos(false);
      
      odom.updateTrajectory();
      
      camera.grabFrame();
      status=camera.lucasCanade( frame1_features, frame2_features );
      camera.compute_traj(frame1_features, frame2_features, odom);
      frame1_features = frame2_features;
      
      odom.prevPosSet();
      counter++;
      if(counter > 500){
        break;
      }
      
      if( status.size() < 15 )
      {
      
            
        frame1_features = camera.shiTomasi();
        cout << "Redetection of feature points" << endl ;
     
      }
      
    }
    counter = 0;
    odom.setSpeed(0.1 * MAX_SPEED, -0.1 * MAX_SPEED);
    while (wb_robot_step(TIME_STEP) != -1)
    {
      
      odom.computePos(true);
      
      odom.updateTrajectory();
      
      camera.grabFrame();
      status=camera.lucasCanade( frame1_features, frame2_features );
      camera.compute_traj(frame1_features, frame2_features, odom);
      frame1_features = frame2_features;
      odom.prevPosSet();
      if(counter>138){
        break;
      }counter++;
      
      if( status.size() < 15 )
      {
      
            
        frame1_features = camera.shiTomasi();
        cout << "Redetection of feature points" << endl ;
     
      }
      
    }
     basic_count++; 
  }

   
 
  //CLEAN

  wb_robot_cleanup();
  return 0;
}

/**************************************************************************************************************************/
