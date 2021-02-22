class CameraParams{

  private:
    WbDeviceTag camera;
    int width;
    int height;
    double hor_FOV;
    double ver_FOV;
    double blindspot;
    
    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    
    float focal_x;
    
    float center_x;
    float center_y;
    
    Mat img;
    
    // for visual odometry
    Mat visual_odom = Mat::zeros(MAP_SZ * 4, MAP_SZ * 4, CV_8UC3);
    int count = 0;
    
    Mat img1;
    
    Position *visual_pos = nullptr;
    Mat R_f,t_f;
    
  /*-------------------------------------------------------------------------------------------*/
 
  public:
     float focal_y;
     
    void init(){
      camera = wb_robot_get_device("camera");
      wb_camera_enable(camera, TIME_STEP);
      width = wb_camera_get_width(camera);
      height = wb_camera_get_height(camera);
    
      // Camera parameters
    
      hor_FOV = wb_camera_get_fov(camera);
      ver_FOV = 2 * atan(tan(hor_FOV / 2) * height / width);
    
      blindspot = CAMERA_HEIGHT / tan(ver_FOV / 2);
      
    
      readMatrix("camera_parameters/intrinsic", intrinsic);
      readMatrix("camera_parameters/distCoeffs", distCoeffs);
    
      focal_x = intrinsic.at<double>(0, 0);
      focal_y = intrinsic.at<double>(1, 1);
      center_x = intrinsic.at<double>(0, 2);
      center_y = intrinsic.at<double>(1, 2);
      
      img = Mat(Size(width, height), CV_8UC4);
      
      visual_pos = new Position{0,0,0};
      
     
      
    }
    
    void prevImg(){
      img1 = grayscale();
      return;
    }
    
    void show_traj()
      {
          char text[100];
          int fontFace = FONT_HERSHEY_PLAIN;
          double fontScale = 0.85;
          int thickness = 1;  
          Point textOrg(10, 50);
           
          
          int x = MAP_SZ * 4 * visual_pos->xPos  + 150; 
         // int x = MAP_SZ * 4 * visual_pos->xPos  ;  // for ransac

          int y = MAP_SZ * 4 - 100 - MAP_SZ * 4 * visual_pos->zPos;
          circle(visual_odom, Point(x,y), 2, Scalar(0,0,255));
            
          rectangle(visual_odom, Point(20, 30), Point(380, 80), CV_RGB(0,0,0), FILLED);
          sprintf(text, "x = %02fm  y = %02fm  th = %02fdeg", visual_pos->xPos, visual_pos->zPos, visual_pos->theta* 180/PI);
          putText(visual_odom, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
            
          namedWindow( "Visual Odometry", WINDOW_NORMAL );
          imshow( "Visual Odometry", visual_odom );
          imwrite("visual_odom.jpg", visual_odom);
          
          waitKey(TIME_STEP);
      
      }
    /*----------------------------------------------------------*/
  
    void readMatrix(string img_path, Mat &matr){
    
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
    
    /*----------------------------------------------------------*/
    
    void grabFrame(){
      img.data = (uchar *)wb_camera_get_image(camera);
    //  i++;
    //  imwrite("data_img/"+to_string(i)+".jpg",img);
      return;
    }
    
    /*----------------------------------------------------------*/
    
    Mat grayscale(){
      Mat gray;
      cvtColor(img, gray, COLOR_BGR2GRAY);
      return gray;
    }
    

    /*----------------------------------------------------------*/
    
    vector<Point2f> shiTomasi()
    {
      vector<Point2f> frame_features;
      const int  minDistance = 5, blockSize = 5 ;
      double qualityLevel = 0.01 ; 
      const int number_of_features_tr = 100 ;
      
      goodFeaturesToTrack(grayscale(), frame_features, number_of_features_tr, qualityLevel, minDistance, noArray(), blockSize);
      
      return frame_features;
    }
    
    /*----------------------------------------------------------*/
  
  vector<uchar> lucasCanade ( vector<Point2f> &frame1_features, vector<Point2f> &frame2_features )
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
        calcOpticalFlowPyrLK(img1,grayscale(),frame1_features,frame2_features,status,err,winSize,maxLevel,
                            TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,10,0.03));
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
       // Point2f pt = frame2_features.at(i- indexCorrection);
        if( (feature_hypotenuse_square >= thresshold_tr) || (q.x < 0) || (q.y < 0) || (q.x > 480) || (q.y > 360) )
        {
          status.at(i) = 0;
          frame1_features.erase (frame1_features.begin() + (i - indexCorrection));
          frame2_features.erase (frame2_features.begin() + (i - indexCorrection));
          
          
          indexCorrection++;
        }
       }
  
      
   img1 = grayscale().clone();
   
    
   return status;
}

/**************************************************************************************************************************/

void compute_traj(vector<Point2f> frame1_features,vector<Point2f> frame2_features, Odometry& od)
{
    
    Point2f p, q;
    float prev_placement = 0 , cur_placement = 0;  
    float theta_point,theta_frame=0;
    int counter = 1;

    for (size_t i = 0; i < frame2_features.size(); i++) 
    {
      p.x = frame1_features[i].x;
      p.y = frame1_features[i].y;
      q.x = frame2_features[i].x;
      q.y = frame2_features[i].y;
			
      counter++ ;

      
      		
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
       
     //Find mean theta 
     theta_frame = theta_frame ;
     theta_frame = theta_frame / counter;
		
     //Result  
     visual_pos->theta += theta_frame;
  
     visual_pos->xPos +=  fabs(od.getScale())*sin(visual_pos->theta);
     visual_pos->zPos +=  fabs(od.getScale())*cos(visual_pos->theta); 

     if(visual_pos->theta > PI && visual_pos->theta < 2*PI)
       visual_pos->theta -= 2*PI;
     else if(visual_pos->theta < -PI && visual_pos->theta > -2*PI)
       visual_pos->theta += 2*PI;
 
     cout<<"Visual Odometry:   x="<<visual_pos->xPos<<"   z="<<visual_pos->zPos<<"   theta= " << visual_pos->theta * 180/PI<<" degrees"<<endl;
     show_traj();
     cout<<" ---------------- "<<endl;
     
     return;	

}


};