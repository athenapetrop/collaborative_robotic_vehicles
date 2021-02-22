/*   @File: cameraParams.cpp
*    @Details: Functions regarding image processing
*/



Mat find_contour_down(Mat frame)
{

  Mat canny;
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  int width = 480;

  blur(frame, frame, Size(3, 3));

  Canny(frame, canny, 100, 240, 3); //gia na min vgazei eswterika simeia sta koutia
  //Canny(frame, canny, 100, 200, 3);
  // Find contours
  findContours(canny, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));

  /// Draw contours
  Mat drawing = Mat::zeros(canny.size(), CV_8UC3);
  Scalar color1 = Scalar(0, 0, 255);

  for (size_t i = 0; i < contours.size(); i++)
  {
    drawContours(drawing, contours, i, color1, 2, 8, hierarchy, 0, Point());
  }

  Mat low_pts = Mat::zeros(width / WIDTH_STEP, 2, CV_32FC1);
  for (size_t j = 0; j < contours.size(); j++)
  {
    for (size_t i = 0; i < contours[j].size(); i++)
    {

      int x = div(contours[j][i].x, WIDTH_STEP).quot;

      circle(drawing, Point(contours[j][i].x, contours[j][i].y), 1, Scalar(0, 255, 0), -1);

      if (contours[j][i].y > low_pts.at<float>(x, 1))
      {
        low_pts.at<float>(x, 0) = contours[j][i].x;
        low_pts.at<float>(x, 1) = contours[j][i].y;
      }
    }
  }
  /* namedWindow("draw", WINDOW_NORMAL);
  imshow("draw",canny);
  waitKey(TIME_STEP);*/

  return low_pts;
}
/**************************************************************************************************************************/

vector<Point2f> shiTomasi(Mat img)
{

  vector<Point2f> frame_features;
  const int minDistance = 5, blockSize = 5;
  double qualityLevel = 0.01;
  const int number_of_features = 100;

  goodFeaturesToTrack(img, frame_features, number_of_features, qualityLevel, minDistance, noArray(), blockSize);
  /*
  for (auto vec: frame_features)
    circle(img, vec, 3, Scalar(0,0,0));
  
   namedWindow("draw", WINDOW_NORMAL);
  imshow("draw",img);
  waitKey(TIME_STEP);
*/
  return frame_features;
}

/**************************************************************************************************************************/

void calculate_depth(Mat img, vector<double> &depth, vector<double> &angle, struct CamParam *cam)
{
  Mat down_edges;
  vector<Point2f> features_for_mapping;
  double angle_x;
  double d;

  down_edges = find_contour_down(img);
  features_for_mapping = shiTomasi(img);
  for (size_t i = 0; i < features_for_mapping.size(); i++)
  {
    int angular_bin = div(features_for_mapping[i].x, WIDTH_STEP).quot;

    //depth
    if (down_edges.at<float>(angular_bin, 1) != 0 && down_edges.at<float>(angular_bin, 1) != 360)
    {
      d = CAMERA_HEIGHT * cam->focal_y / fabs(cam->center_y - down_edges.at<float>(angular_bin, 1)) + cam->blindspot;

      if (d < 2 && d > -2) //for filtering out errors in depth
      {
        depth.push_back(d);

        //angles
        angle_x = atan2((features_for_mapping[i].x - cam->center_x), cam->focal_x);
        angle.push_back(angle_x);
      }
    }
  }
  return;
}

/**************************************************************************************************************************/
  Rect getBoundingRect(Mat mask){
     Rect ROI;
    
     ROI = boundingRect(mask);

    /* rectangle(img, ROI , Scalar(0, 0, 255), 4, 8 , 0);
     namedWindow("Bounding Rect", WINDOW_NORMAL);            
     imshow("Bounding Rect", img);
     waitKey(TIME_STEP);*/
     return ROI;
  }

/**************************************************************************************************************************/  

  bool robotRecognition(Rect &robot, Mat img){
      
      Mat hsv;
      cvtColor(img, hsv, COLOR_BGR2HSV);
      Mat mask1,mask2;
      
       // Creating masks to detect the upper and lower color.
      if(readRobotId() == 1){                                                 // robot 1 searches for green color
        inRange(hsv, Scalar(44,54,63), Scalar(71,255,255), mask1);
        inRange(hsv, Scalar(), Scalar(), mask2);
        
      }else if (readRobotId() == 2){                                          // robot 2 searches for red color
        inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255), mask2);
        mask1 = mask1 + mask2;
      }
      
      // Generating the final mask
      robot = getBoundingRect(mask1);
      
      if(robot.height * robot.width > 10 && (robot.x + robot.width / 2)<(FRAME_WIDTH/2 + 5) && 
         (robot.x + robot.width / 2)>(FRAME_WIDTH/2 - 5) )
          return true;
      
      /*
      namedWindow("Color Filtering", WINDOW_NORMAL);            
      imshow("Color Filtering", mask1);
      waitKey(TIME_STEP);
      */
      
      return false;
      
  }