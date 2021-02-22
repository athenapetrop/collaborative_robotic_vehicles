//File:           cooperative_robots_final.cpp
// Date:          11/02/21
// Description:   2 robot mapping
// Author:        Athena Petropoulou
// Modifications: 

#include <include/headers.h>
#include <include/functions.h>
#include <src/util.cpp>
#include <src/odometry.cpp>
#include <src/pathPlanning.cpp>
#include <src/mapping.cpp>
#include <src/cameraParams.cpp>
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
  
  
  AstarAlgorithm path;
 
  /********************************************************************************/

  Position pos = {0, 0, 0};
  Encoders enc = {0, 0};

  vector<Map> maps;
  
  //create first map (central map)
  Map map0;
  map0.id = Point(0, 0);
  maps.push_back(map0);

  Mat global_map, global_map_draw;
  /********************************************************************************/
  vector<Point2f> visited_targets;

  while (wb_robot_step(TIME_STEP) != -1)
  {
    cout << "Robot "<<readRobotId()<<" -> Started mapping" << endl;
    
    // set up the motor speeds
    setSpeed(0.1 * MAX_SPEED, -0.1 * MAX_SPEED, &od);

    double prevth = pos.theta;
    double thDiff = 0;
    while (!isFullRotation(pos.theta, prevth, thDiff))

    {

      wb_robot_step(TIME_STEP);
      compute_pos(&pos, &enc, &od);

      // grab frame
      Mat img = Mat(Size(cam.width, cam.height), CV_8UC4);
      img.data = (uchar *)wb_camera_get_image(cam.camera);
      
      // grayscale
      cvtColor(img, img, COLOR_BGR2GRAY);

      // mapping
      vector<double> depth;
      vector<double> angle;
      calculate_depth(img, depth, angle, &cam);
      mapping(&pos, depth, angle, maps);

    }
    cout << "Robot "<<readRobotId()<<" ->Finished mapping" << endl;
    cout << "Robot "<<readRobotId()<<" ->Drawing the maps now..." << endl;
    
    
    for (size_t i = 0; i < maps.size(); i++)
    {
      show_maps(maps, to_string(i), (int)i, readRobotId());
    }

    setSpeed(0, 0, &od);
    wb_robot_step(TIME_STEP);


    Point sz = findFinalMapSZ(maps);
    updateGlobalId(maps);
    makeFinalMap(maps, sz, readRobotId(), global_map, global_map_draw);

    namedWindow("Map", WINDOW_NORMAL);
    imshow("Map",global_map_draw);
    waitKey(TIME_STEP);

    vector<Point2f> obst = obstacles(maps);
    Point2f target_pt = targetPoint(&pos, maps, visited_targets);
    
    if (target_pt != Point2f(0, 0))
    {
      visited_targets.push_back(target_pt);
      
      vector<Point2f> traj;
      if(ASTAR == 1){

        bool a = path.MapCreate(obst, target_pt, globalRobotPos(&pos, maps), global_map.cols, global_map.rows);
        bool b = path.Solve_AStar();
        path.make_path();
        traj = path.path_points();
      }else{
        traj = artificial_potential_field(&pos, target_pt, obst, maps);
        for(auto vec:traj){
          circle(global_map_draw, vec, 2, Scalar(0,255,0), FILLED );
        }
         namedWindow("Path", WINDOW_NORMAL);
        imshow("Path",global_map_draw);
        waitKey(TIME_STEP);
        imwrite("traj.jpg",global_map_draw);
      }
      /***************************************************************/
  
      int count = 0;
      Point2f speeds;
      bool atgoal;
    
      cout << "Robot "<<readRobotId()<<" ->Going to point " << target_pt << endl;
    
      while (wb_robot_step(TIME_STEP) != -1)
      {
        atgoal=false;
        while(!atgoal){
          robot_controller(globalRobotPos(&pos, maps), pos.theta, count, traj, atgoal, speeds);

          setSpeed(speeds.x, speeds.y, &od);
          compute_pos(&pos, &enc, &od);
          wb_robot_step(TIME_STEP);
        }
        
        count++;
        if ((int)traj.size() == count )
          break;

       

        waitKey(TIME_STEP);
      }
    }

    else
    {
      cout << "Robot "<<readRobotId()<<" ->Finished Mapping" << endl;
      break;
    }
  }
  //CLEAN

  wb_robot_cleanup();
  return 0;
}
