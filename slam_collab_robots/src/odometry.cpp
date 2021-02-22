/*   @File: util.cpp
*    @Details: Functions about odometry
*/

void setSpeed(double speed_l, double speed_r, struct Odometry *od){
    wb_motor_set_velocity( od->left_motor, speed_l);
    wb_motor_set_velocity( od->right_motor, speed_r);
    
    return;
 
}
/*--------------------------------------------------*/
void robot_controller(Point2f mypos, double theta, int pos_count, vector<Point2f> path, bool &atgoal, Point2f &speeds) {
  /* get current position */
  double curPos[3] = {mypos.x, mypos.y, theta};
  
  double goalPos[3] = {path[pos_count].x, path[pos_count].y, atan2(path.at(pos_count).x - mypos.x, -(path.at(pos_count).y - mypos.y))};

  double dx = goalPos[0] - curPos[0];
  double dy = -goalPos[1] + curPos[1];

  /* controller parameters, an initial choice for the values is given but might be changed */
  
  double k_rho = 1.2;
  double k_alpha = 1.5;
  

  /* calculate current distance and angles to goal position */
  double rho_c = sqrt(dx * dx + dy * dy);  
  double alpha_c = atan2(dx, dy) - curPos[2];  
  
  while (alpha_c > PI)  /* to prevent alpha from getting too big */
    alpha_c = alpha_c - 2 * PI;
  while (alpha_c < -PI)
    alpha_c = alpha_c + 2 * PI;



  /* control law */
  double v_c = k_rho * rho_c * cos(alpha_c);

  double omega_c = k_alpha * alpha_c * (1 + square(rho_c)) - 0.5 * k_rho * sin(2 * alpha_c);

  double v_e = v_c;
  double omega_e = omega_c;

  /* finally record motor speed */
  speeds.x = (v_e + omega_e / 2);  /* left */
  speeds.y = (v_e - omega_e / 2);  /* right */

  /* Don't set speeds < MIN_SPEED (for accuracy reasons) */
  if (abs(speeds.x) < 1.0)
    speeds.x  = 0;
  if (abs(speeds.y) < 1.0)
    speeds.y  = 0;
    
  if (abs(speeds.x) > 0.5 * MAX_SPEED)
    speeds.x =  0.5 * MAX_SPEED * speeds.x / abs(speeds.x);
  if (abs(speeds.y) >  0.5 * MAX_SPEED)
    speeds.y =  0.5 * MAX_SPEED * speeds.y / abs(speeds.y);
   

  /* Termination condition */
  if ((speeds.x  == 0 && speeds.y  == 0) || rho_c < 2)
    atgoal = true;

  return;
}

/*--------------------------------------------------*/


static void compute_pos(struct Position *pos, struct Encoders *enc, struct Odometry *od)
{
  double left_enc = wb_position_sensor_get_value(od->left_ps);
  double right_enc = wb_position_sensor_get_value(od->right_ps);
  
  double dl = (left_enc - enc->left) * WHEEL_RADIUS;   /* distance covered by left wheel in meters */
  double dr = (right_enc - enc->right) * WHEEL_RADIUS; /* distance covered by right wheel in meters */
  double deltaTheta = (dl - dr) / (AXIS_LENGTH);       /* delta orientation in rad */

  double deltaStep = (dl + dr) / 2; /* Expressed in meters. */
  

  pos->xPos += deltaStep * sin(pos->theta + deltaTheta / 2); /* Expressed in meters. */
  pos->zPos += deltaStep * cos(pos->theta + deltaTheta / 2); /* Expressed in meters. */

  pos->theta += deltaTheta;


  if(pos->theta > PI && pos->theta < 2*PI)
    pos->theta -= 2*PI;
  else if(pos->theta < -PI && pos->theta > -2*PI)
    pos->theta += 2*PI;
 
  //cout<<"Robot "<<readRobotId()<<" -> " << "x=" << pos->xPos << "m  z=" << pos->zPos << "m  angle=" << pos->theta*180/PI << " degrees" << endl;

  enc->left = left_enc;
  enc->right = right_enc;
  
  return;
}

/*--------------------------------------------------*/

bool isFullRotation(double currTheta, double &prevTheta, double &deltaTheta){

  double diff;
  if(currTheta * prevTheta > 0)
    diff = abs(currTheta - prevTheta);
  else
    diff = abs(currTheta + prevTheta);
  deltaTheta += diff;

  if(deltaTheta >= 2*PI){
    prevTheta = 0;
    deltaTheta = 0;
    return true;
    
  }else{
    prevTheta = currTheta;
    return false; 
  }
}

/*--------------------------------------------------*/
     
Point3f posInNewCoordinateSystem(struct Position *received_pos, float received_distance, struct Position *pos, struct Position &originPos){
      
    cout<<"Robot "<<readRobotId()<<" -> Received Pos x: "<<received_pos->xPos<<"  Received Pos z: "<<received_pos->zPos<<"  Received Pos th: "<<received_pos->theta*180/PI<<" degrees"<<endl;
    double x,z,th;
    x = received_pos->xPos + (received_distance + 2*ROBOT_RADIUS) * sin(received_pos->theta);
    z = received_pos->zPos + (received_distance + 2*ROBOT_RADIUS) * cos(received_pos->theta);
                    
    th  = received_pos->theta - PI;
            
    if(th > PI && th < 2*PI)
       th -= 2*PI;
    else if(th < -PI && th > -2*PI)
       th += 2*PI;
            
    originPos = findInitialPos(x, z, th, pos);
    cout<<"Robot "<<readRobotId()<<" -> Origin Pos x: "<<originPos.xPos<<"  Received Pos z: "<<originPos.zPos<<"  Received Pos th: "<<originPos.theta*180/PI<<" degrees"<<endl;
       
    Point3f diff;
    diff.x = x - pos->xPos;
    diff.y = z - pos->zPos;
    diff.z = th - pos->theta;
            
    pos->xPos = x;
    pos->zPos = z;
    pos->theta = th;
            
            
    return diff;
}
 
/*----------------------------------------------------------*/
     
Position findInitialPos(double x_new, double z_new, double th_new, struct Position *pos){
        
    double x_origin, z_origin, th_origin, dist;
    dist = sqrt(square(pos->xPos) + square(pos->zPos)) + 2*ROBOT_RADIUS ;
            
    x_origin = x_new - pos->xPos;
    z_origin = z_new - pos->zPos;
    
    
    th_origin = th_new - pos->theta;
    if(th_origin > 2*PI)
        th_origin -= 2*PI;
    if(th_origin > PI && th_origin < 2*PI)
        th_origin -= 2*PI;
    else if(th_origin < -PI && th_origin > -2*PI)
        th_origin += 2*PI;
            
    Position originPos = {x_origin, z_origin, th_origin};

    return originPos;
}

/*----------------------------------------------------------*/

double findOtherRobotsDist(Rect robot, double focal_y)
{
  //double d = CAMERA_HEIGHT * focal_y / robot.height + blindspot;
  double d = focal_y * EPUCK_HEIGHT / robot.height;
  
  return d;
}