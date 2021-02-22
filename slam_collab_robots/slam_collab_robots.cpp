/*
* File:            cooperative_robots_final.cpp
* Date:            11/02/21
* Description:     Cooperative robots mapping
* Author:          Athena Petropoulou
* Modifications: 
*/

#include <include/headers.h>
#include <include/functions.h>
#include <src/util.cpp>
#include <src/odometry.cpp>
#include <src/pathPlanning.cpp>
#include <src/mapping.cpp>
#include <src/cameraParams.cpp>
#include <src/communication.cpp>
#include <src/a_star.cpp>
/***********************************************************************************************************/

int main()
{
  
  // BASIC ROBOT INIT
  wb_robot_init();

  Odometry od;
  initOdom(&od);

  /* Initialize camera */
  CamParam cam;
  initCameraParams(&cam);
  
  /* Initialize communication */
  Communication comm;
  comm.init();
  
  AstarAlgorithm path;
  
  Position pos = {0, 0, 0};
  Encoders enc = {0, 0};
  Position received_pos = {0, 0, 0};
  Position originPos = {0, 0, 0};
  vector<Map> maps;
  
  /* create first map (central map) */
  Map map0;
  map0.id = Point(0, 0);
  maps.push_back(map0);

  Mat global_map, global_map_draw;
  vector<Point2f> visited_targets;
  vector<Map> receivedMap;
  bool gotMap = false;
  Point3f diff = Point3f(0,0,0);

  /********************************************************************************/

  while (wb_robot_step(TIME_STEP) != -1)
  {
    cout << "Robot "<<readRobotId()<<" -> Started mapping" << endl;
    
    /* set up the motor speeds */
    setSpeed(0.1 * MAX_SPEED, -0.1 * MAX_SPEED, &od);

    double prevth = pos.theta;
    double thDiff = 0;
    Rect robot(0,0,0,0);
    
    while (!isFullRotation(pos.theta, prevth, thDiff))

    {
      comm.receive(&od, maps, &pos);
      wb_robot_step(TIME_STEP);
      compute_pos(&pos, &enc, &od);

      
      if(comm.mapAlert){
        comm.mapAlert = false;
        receivedMap = comm.getReceivedMaps();
        cout<<"Robot "<<readRobotId()<<" -> I received "<<receivedMap.size()<<" local maps from the other robot"<<endl;
        gotMap = true;
        
      }
      /* grab frame */
      Mat img = Mat(Size(cam.width, cam.height), CV_8UC4);
      img.data = (uchar *)wb_camera_get_image(cam.camera);
      
      /* grayscale */
      Mat gray;
      cvtColor(img, gray, COLOR_BGR2GRAY);
      
      Point sz;
      /* if received message of been seen, rotate until you find the other robot too */
      if(comm.adjustAngle()){
        cout<<"Robot "<<readRobotId()<<" -> Adjusting angle"<<endl;
        setSpeed(0.1 * MAX_SPEED, -0.1 * MAX_SPEED, &od);
       
        while(!robotRecognition(robot, img)){
         
          wb_robot_step(TIME_STEP);
          compute_pos(&pos, &enc, &od);
          Mat img = Mat(Size(cam.width, cam.height), CV_8UC4);
          img.data = (uchar *)wb_camera_get_image(cam.camera);
        }
          cout<<"Robot "<<readRobotId()<<" -> Saw the other robot too"<<endl;
          comm.adjustAngleDone();
          setSpeed(0, 0, &od);
          wb_robot_step(TIME_STEP);
          
          /* master's position and calculated distance to me */
          float received_distance = comm.getReceivedDist();
          received_pos = comm.getReceivedPos();
          diff = posInNewCoordinateSystem(&received_pos, received_distance, &pos, originPos);

          comm.robot_found = true;
          
          /* make global map and local maps at this point in time */
          drawMaps(maps, readRobotId());
          sz = findFinalMapSZ(maps);
          updateGlobalId(maps);
          makeFinalMap(maps, sz, readRobotId(), global_map, global_map_draw);
          
          /* adjust current map to other robot's coordinate system */
          imwrite("glob.jpg",global_map_draw);
         
          translate(diff.x, diff.y, global_map);
          //rotate(diff.z, global_map, maps, diff.x,diff.y);
       //   transform_map(diff.z, diff.x, diff.y, global_map, maps);
          
                   
          
          /* Ask for leader's map */
          wb_robot_step(TIME_STEP);
          comm.emit({SEND_MAP});
       }   
        if(gotMap){
          gotMap = false;
          //updateGlobalId(receivedMap);
          
          if(comm.firstMerge){
            adjustMap(diff, global_map, maps, sz, originPos, receivedMap);
            cout<<"Robot "<<readRobotId()<<" -> Made adjusted map" << endl;
            comm.firstMerge = false;
          }
          
                  
          mergeMaps(maps, receivedMap);
         


          drawMaps(maps, readRobotId());
          sz = findFinalMapSZ(maps);
          updateGlobalId(maps);
          makeFinalMap(maps, sz, readRobotId(), global_map, global_map_draw);
          
          wb_robot_step(TIME_STEP);
          comm.sendMap(maps, (float)SENDING_MERGED);
          
          wb_robot_step(TIME_STEP);
          comm.emit({CONTINUE});
          setSpeed(0.1*MAX_SPEED, -0.1*MAX_SPEED, &od);
          cout<<"Robot "<<readRobotId()<<" -> Continue with mapping" << endl;
        }
      
      
      /* if other robot has not been yet recognized */
     /* if( !comm.robot_found ){
      
        if( robotRecognition(robot, img) ){
          cout<<"Robot "<<readRobotId()<<" -> Found the other robot!" << endl;

          setSpeed(0, 0, &od);
          comm.robot_found = true;
          
          /* make global map at this point in time */
    /*    drawMaps(maps, readRobotId());
          Point sz = findFinalMapSZ(maps);
          updateGlobalId(maps);
          makeFinalMap(maps, sz, readRobotId(), global_map, global_map_draw);

          /* calculate distance to the other robot and send message of dist and my position */
      /*    double d = findOtherRobotsDist(robot, cam.focal_y);
          cout<<"Robot "<<readRobotId()<<" -> Distance: "<<d<<endl;
          comm.emit({SAW_YOU, (float)d, (float)pos.xPos, (float)pos.zPos, (float)pos.theta});
              
          wb_robot_step(TIME_STEP);
       
        }
    
      }*/
      
      /* Mapping if no communication is ongoing */
      if(!comm.commStatus()){
        vector<double> depth;
        vector<double> angle;
        Rect checkRobot = checkIfRobotInFrame(img);
        calculate_depth(gray, depth, angle, &cam, checkRobot);
        if(!comm.robot_found){
          mapping(&pos, depth, angle, maps, Point2f(0,0));
        }else{
          mapping(&pos, depth, angle, maps, Point2f(diff.x,diff.y));
        }
      //  drawMaps(maps, readRobotId());
      //  Point sz = findFinalMapSZ(maps);
      //  updateGlobalId(maps);
      //  makeFinalMap(maps, sz, readRobotId(), global_map, global_map_draw);

      }

    }    /* Mapping finished */
    
    cout << "Robot "<<readRobotId()<<" -> Finished mapping" << endl;
    cout << "Robot "<<readRobotId()<<" -> Drawing the maps now..." << endl;
    
    /* Stop robot's movement */
    setSpeed(0, 0, &od);
    wb_robot_step(TIME_STEP);


    /* Wait for other robot to continue sending your map */
    if(comm.robot_found){
      comm.receive(&od, maps, &pos);
      while(comm.waitingStatus){
      
        wb_robot_step(TIME_STEP);
        comm.receive(&od, maps, &pos);
      }
    }
    /* Make global map after the mapping */
    drawMaps(maps, readRobotId());
    Point sz = findFinalMapSZ(maps);
    updateGlobalId(maps);
    makeFinalMap(maps, sz, readRobotId(), global_map, global_map_draw);
    wb_robot_step(TIME_STEP);
    
    if(comm.robot_found){
    
      comm.sendMap(maps, (float)MAP_SENT);
      wb_robot_step(TIME_STEP);
    }
    /* Find obstacles' position in global map */
    vector<Point2f> obst = obstacles(maps);
      
      
    /* Send message to the other robot to stop and send its position - collision avoidance */
    if(comm.robot_found && !comm.otherFinished){
      cout << "Robot "<<readRobotId()<<" -> Asking for other robot's position"<<endl; 
      comm.emit({(float)SEND_POS});
      wb_robot_step(TIME_STEP);
       
      while(!comm.gotPos){                       /* Waiting until the other robot sends position */
        comm.receive(&od, maps, &pos);
        wb_robot_step(TIME_STEP);

      }
        
      drawMaps(maps, readRobotId());
      sz = findFinalMapSZ(maps);
      updateGlobalId(maps);
      makeFinalMap(maps, sz, readRobotId(), global_map, global_map_draw);
      
      received_pos = comm.getReceivedPos();
      Point2f receivedInMap = globalRobotPos(&received_pos, maps);

      for(int x = -4; x < 4; x++){
        for(int y = -4; y < 4; y++){
          obst.push_back(Point2f( receivedInMap.x + x, receivedInMap.y + y ));
        }
      }
       
      comm.gotPos = false;
        
    }
    wb_robot_step(TIME_STEP);
    drawMaps(maps, readRobotId());
    sz = findFinalMapSZ(maps);
    updateGlobalId(maps);
    makeFinalMap(maps, sz, readRobotId(), global_map, global_map_draw);
 
    /* Find next target point */
    Point2f target_pt = targetPoint(&pos, maps, visited_targets);
    visited_targets.push_back(target_pt);
    if(target_pt != Point2f(0,0)){
     // drawObstacles(global_map_draw, obst); 
      /* Find trajectory from mypos to the target point via APF or A* */
      vector<Point2f> traj;
      if(ASTAR == 1){
  
        bool a = path.MapCreate(obst, target_pt, globalRobotPos(&pos, maps), global_map.cols, global_map.rows);
        bool b = path.Solve_AStar();
        path.make_path();
        traj = path.path_points();
      }else{
        traj = artificial_potential_field(&pos, target_pt, obst, maps);
      } 
      drawTrajectory(traj, global_map_draw, target_pt);
      
  
      /***************************************************************/
  
      int count = 0;
      Point2f speeds;
      bool atgoal;
    
      cout << "Robot "<<readRobotId()<<" -> Going to point " << target_pt << " from position "<<globalRobotPos(&pos, maps)<< endl;
      while (wb_robot_step(TIME_STEP) != -1)
      {
        if(traj.size() == 0)
          break;
        atgoal=false;
        while(!atgoal){
          
          /* calculate speed of each motor to get to the next spot in trajectory */
          robot_controller(globalRobotPos(&pos, maps), pos.theta, count, traj, atgoal, speeds);
          setSpeed(speeds.x, speeds.y, &od);
          compute_pos(&pos, &enc, &od);
          wb_robot_step(TIME_STEP);
        }
        
        count++;
        if ((int)traj.size() == count ){
          cout << "Robot "<<readRobotId()<<" -> Went to point " << target_pt << endl; 
          break;
        }

       

        waitKey(TIME_STEP);
      }
    }
    
    /* if no target point found then the whole area is explored */
    else
    {
      cout << "Robot "<<readRobotId()<<" -> Exiting" << endl;
      break;
    }
    
    setSpeed(0, 0, &od);
    if(comm.robot_found){
      comm.emit({(float)GO});
    }
    wb_robot_step(TIME_STEP);

    //cout << "Robot "<<readRobotId()<<" -> I'm allowed to continue" << endl;
    
   
  }
    setSpeed(0, 0, &od);
    if(comm.robot_found){
      comm.emit({(float)FINISHED});
    }
    wb_robot_step(TIME_STEP);
  
  //CLEAN

  wb_robot_cleanup();
  return 0;
}



/**************************************************************************************************************************/
/*
void discardWrongMaps(vector<Map> &maps)
{
  for (size_t i = 0; i < maps.size(); i++)
  {

    Mat hsv, thresholded;
    cvtColor(maps[i].map_draw, hsv, COLOR_BGR2HSV);

    inRange(hsv, Scalar(0, 0, 0, 0), Scalar(180, 255, 30, 0), thresholded);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    // Find contours
    findContours(thresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));

    /// Draw contours
    Mat drawing = Mat::zeros(thresholded.size(), CV_8UC3);
    Scalar color1 = Scalar(0, 0, 255);

    for (size_t i = 0; i < contours.size(); i++)
    {
      drawContours(drawing, contours, i, color1, 2, 8, hierarchy, 0, Point());
    }

    if (contours.size() == 0)
    {
      maps.erase(maps.begin() + i);
    }
    else if (contours.size() == 1 && contourArea(contours[0]) > 0.97 * square(MAP_SZ))
    {
      // cout<<"Erasing map "<<i<<endl;
      maps.erase(maps.begin() + i);
    }

    // imwrite("todiscard/"+to_string(i)+".jpg" ,drawing);
  }
  return;
}*/
