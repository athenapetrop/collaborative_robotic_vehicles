class Odometry{

  private:
    WbDeviceTag left_motor;
    WbDeviceTag right_motor;
    
  
    // position sensors
    WbDeviceTag left_ps, right_ps;
    double left_enc = 0; 
    double right_enc = 0;
 
    struct Encoders
    {
      double left,right;
    };
    
    Position *pos = nullptr;
    Position *wheel_pos = nullptr;
    Position *wheel_pos_2 = nullptr;
    Position *prevPos = nullptr;
    
    Encoders *enc = nullptr;
    Position *prev_pos = nullptr;
    
    Position *originPos = nullptr;
    
    WbNodeRef robot_node;

    WbFieldRef trans_field;
    WbFieldRef rot_field;

    double delta_theta = 0;
    double prev_theta = 0;
    
    double speed_right = 0;
    double speed_left = 0;
    
    double scale = 0;
    
    Mat ground_truth = Mat::zeros(MAP_SZ * 4, MAP_SZ * 4, CV_8UC3);
    Mat wheel_odom = Mat::zeros(MAP_SZ * 4, MAP_SZ * 4, CV_8UC3);
    
/*---------------------------------------------------------------------------------------------*/

  public:
  
    void init()
    {
      wb_robot_init();
      left_motor = wb_robot_get_device("left wheel motor");
      right_motor = wb_robot_get_device("right wheel motor");
      left_ps = wb_robot_get_device("left wheel sensor");
      right_ps = wb_robot_get_device("right wheel sensor");
      
      wb_motor_set_position(left_motor, INFINITY);
      wb_motor_set_position(right_motor, INFINITY);
      
      wb_position_sensor_enable(left_ps, TIME_STEP);
      wb_position_sensor_enable(right_ps, TIME_STEP);
      
      
      pos = new Position{0, 0, 0};
      wheel_pos = new Position{0, 0, 0};
      wheel_pos_2 = new Position{0, 0, 0};
      enc = new Encoders{0, 0};
      prev_pos = new Position{0, 0, 0};
      prevPos = new Position{0, 0, 0};
      
      robot_node = wb_supervisor_node_get_from_def((char *)wb_robot_get_name());

      trans_field = wb_supervisor_node_get_field(robot_node, "translation");
      rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
      
      set_init_prev_pos();
      
      
      return;
    }
    /*----------------------------------------------------------*/
    
    double getXdiff(){
      return wheel_pos->xPos - prevPos->xPos;
    }
    
    double getZdiff(){
      return wheel_pos->zPos - prevPos->zPos;
    }
    
    double getScale(){
      return scale;
    }
    /*----------------------------------------------------------*/
    
    void setSpeed(double left_sp, double right_sp)
    {
      wb_motor_set_velocity(left_motor, left_sp);
      wb_motor_set_velocity(right_motor, right_sp);
      
      speed_right = right_sp;
      speed_left = left_sp;
      
      return;
    }
    /*----------------------------------------------------------*/
    
    Point2f getSpeed(){
      return Point2f(speed_left, speed_right);
    }
    
   /*----------------------------------------------------------*/
    
    void updateEncoderVal()
    {
      left_enc = wb_position_sensor_get_value(left_ps);
      right_enc = wb_position_sensor_get_value(right_ps);
      
      return;
    }
    
    /*----------------------------------------------------------*/
    
    void set_init_prev_pos(){
    
        const double *values_trans = wb_supervisor_field_get_sf_vec3f(trans_field);
        const double *values_rot = wb_supervisor_field_get_sf_rotation(rot_field);
        
        double x = values_trans[0];
        double z = values_trans[2];
        double th = values_rot[3];
        
        prev_pos->xPos = x;
        prev_pos->zPos = z;
        prev_pos->theta = th;
        
        return;
    }
    /*----------------------------------------------------------*/
    
    void computePos(bool strive)
    {
       
     
        updateEncoderVal();
      
        double dl = (left_enc - enc->left) * WHEEL_RADIUS;   // distance covered by left wheel in meters
        double dr = (right_enc - enc->right) * WHEEL_RADIUS; // distance covered by right wheel in meters
        double deltaTheta = (dl - dr) / (AXIS_LENGTH);      // delta orientation in rad
      
        double deltaStep = (dl + dr) / 2; // Expressed in meters.
        scale = deltaStep;
        
        
        wheel_pos->xPos += deltaStep * sin(wheel_pos->theta + deltaTheta / 2); // Expressed in meters.
        wheel_pos->zPos += deltaStep * cos(wheel_pos->theta + deltaTheta / 2); // Expressed in meters.
        
   /*     wheel_pos_2->xPos += deltaStep * sin(wheel_pos_2->theta + deltaTheta / 2); // Expressed in meters.
        wheel_pos_2->zPos += deltaStep * cos(wheel_pos_2->theta + deltaTheta / 2); // Expressed in meters.
    */    
        wheel_pos->theta += deltaTheta;
        
    /*    if(!strive)
          wheel_pos_2->theta += deltaTheta;
        else
          wheel_pos_2->theta += (deltaTheta - 0.00111644396293202679906539915292);
    */     
          if(wheel_pos->theta > PI && wheel_pos->theta < 2*PI)
            wheel_pos->theta -= 2*PI;
          else if(wheel_pos->theta < -PI && wheel_pos->theta > -2*PI)
            wheel_pos->theta += 2*PI;
      /*      
            if(wheel_pos_2->theta > PI && wheel_pos_2->theta < 2*PI)
            wheel_pos_2->theta -= 2*PI;
          else if(wheel_pos_2->theta < -PI && wheel_pos_2->theta > -2*PI)
            wheel_pos_2->theta += 2*PI;
      */    
        cout<<"Wheel Odometry -> " << "x=" << wheel_pos->xPos << "m  z=" << wheel_pos->zPos << "m  angle=" << wheel_pos->theta*180/PI << " degrees" << endl;
     //   cout<<"Wheel Odometry Fixed -> " << "x=" << wheel_pos_2->xPos << "m  z=" << wheel_pos_2->zPos << "m  angle=" << wheel_pos_2->theta*180/PI << " degrees" << endl;

        
        enc->left = left_enc;
        enc->right = right_enc;
      
        const double *values_trans = wb_supervisor_field_get_sf_vec3f(trans_field);
        const double *values_rot = wb_supervisor_field_get_sf_rotation(rot_field);
        
        double x = values_trans[0];
        double z = values_trans[2];
        double th = values_rot[3];
        
         deltaStep = sqrt(square(prev_pos->xPos - x)+square(prev_pos->zPos - z));
       
        
        double diff;
        if(th * prev_pos->theta > 0){
         diff = abs(th - prev_pos->theta);
        
        }else{
        
         diff = abs(th + prev_pos->theta);
        }
        if(speed_left > speed_right){
          pos->theta += diff;
        }else if(speed_left < speed_right){
          pos->theta -= diff;
        }else{
        
          pos->theta += 0;
        }
        
        if(pos->theta > PI && pos->theta < 2*PI)
            pos->theta -= 2*PI;
        else if(pos->theta < -PI && pos->theta < -2*PI)
            pos->theta += 2*PI;
        
        
        pos->xPos += deltaStep * sin(pos->theta); // Expressed in meters.
        pos->zPos += deltaStep * cos(pos->theta); // Expressed in meters.
        
        
        cout<<"Ground Truth -> " << "x=" << pos->xPos << "m  z=" << pos->zPos << "m  angle=" << pos->theta*180/PI << " deg" << endl;
        
        
        prev_pos->xPos = x;
        prev_pos->zPos = z;
        prev_pos->theta = th;
        
     
      return;
    }
    
    /*----------------------------------------------------------*/
    
    void prevPosSet(){
      prevPos->xPos = wheel_pos->xPos;
       prevPos->zPos = wheel_pos->zPos;
       prevPos->theta = wheel_pos->theta;
      
    }
    
    void setPrevTheta(){
      prev_theta = pos->theta;
      return;
    }
    
      /*----------------------------------------------------------*/ 
     bool thetaDiff(double angle){
       double diff;
       if(pos->theta * prev_theta > 0)
         diff = abs(pos->theta - prev_theta);
       else
         diff = abs(pos->theta + prev_theta);
       delta_theta += diff;

       if(delta_theta >= angle){
         prev_theta = 0;
         delta_theta = 0;
         return true;
       }
         
       else{
         prev_theta = pos->theta;
         return false; 
       }
     }
    
   
    /*----------------------------------------------------------*/

    
    double getTheta()
    {
      return pos->theta;
    }
    
    /*----------------------------------------------------------*/
    
    Position* getPos(){
      return pos;
    }
    
    /*----------------------------------------------------------*/
    
    void updateTrajectory(){
    
       char text[100];
       int fontFace = FONT_HERSHEY_PLAIN;
       double fontScale = 0.85;
       int thickness = 1;  
       Point textOrg(10, 50);
       Point textOrg2(10, 70);
       
      
        int x = MAP_SZ * 4 * pos->xPos  + 150;
        int y = MAP_SZ * 4 - 100 - MAP_SZ * 4 * pos->zPos;
        
        circle(ground_truth, Point(x,y), 2, Scalar(0,0,255));
        
        rectangle(ground_truth, Point(20, 30), Point(380, 80), CV_RGB(0,0,0), FILLED);
        sprintf(text, "x = %02fm  y = %02fm  th = %02fdeg", pos->xPos, pos->zPos, pos->theta* 180/PI);
        putText(ground_truth, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
        
       
     /*   x = MAP_SZ * 4 * wheel_pos_2->xPos  + 150;
        y = MAP_SZ * 4 - 100 - MAP_SZ * 4 * wheel_pos_2->zPos;
        circle(wheel_odom, Point(x,y), 2 , Scalar(0,255,255));*/
        x = MAP_SZ * 4 * wheel_pos->xPos  + 150;
        y = MAP_SZ * 4 - 100 - MAP_SZ * 4 * wheel_pos->zPos;
        circle(wheel_odom, Point(x,y), 2 , Scalar(0,255,0));
        
        rectangle(wheel_odom, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), FILLED);
        sprintf(text, "x = %02fm  y = %02fm  th = %02fdeg", wheel_pos->xPos, wheel_pos->zPos, wheel_pos->theta* 180/PI);
        putText(wheel_odom, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
        
       /* rectangle(wheel_odom, Point(10, 51), Point(550, 71), CV_RGB(0,0,0), FILLED);
        sprintf(text, "x = %02fm  y = %02fm  th = %02fdeg", wheel_pos_2->xPos, wheel_pos_2->zPos, wheel_pos_2->theta* 180/PI);
        putText(wheel_odom, text, textOrg2, fontFace, fontScale, Scalar::all(255), thickness, 8);
       */ 
        
        
         namedWindow("Ground Truth", WINDOW_NORMAL);
         imshow("Ground Truth",ground_truth);
         
         namedWindow("Wheel Odometry", WINDOW_NORMAL);
         imshow("Wheel Odometry",wheel_odom);
         
         
         imwrite("ground_truth.jpg", ground_truth);
         imwrite("wheel_odom.jpg", wheel_odom);

         
         waitKey(TIME_STEP);
    }
      
};