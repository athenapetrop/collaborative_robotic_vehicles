/*   @File: pathPlanning.cpp
*    @Details: Functions about pathPlanning
*/


vector<Point2f> artificial_potential_field(struct Position *pos, Point2f target_pt, vector<Point2f> obst, vector<Map> maps)
{
  
  int count = 0;
  double Katt = 1, Krep = 250, a = 50, step = 1;
  //double Katt = 30, Krep = 150, a = 1, step = 1;

  double datt, drep;
  double theta, DVx, DVy, DVatty, DVattx;
  double DVrepx, DVrepy, DVrepxS, DVrepyS;

  vector<Point2f> traj;

  Point2f robot_pos = globalRobotPos(pos, maps);

  while (count < 100)
  {

    datt = dist(robot_pos, target_pt);
   // Vatt = -Katt * square(datt) / 2;

    DVattx = Katt * (robot_pos.x - target_pt.x);
    DVatty = Katt * (robot_pos.y - target_pt.y);

    DVrepxS = 0;
    DVrepyS = 0;

    for (size_t i = 0; i < obst.size(); i++)
    {
      drep = dist(robot_pos, obst[i]);
      if (drep < a)
      {
        DVrepx = -Krep * (((1 / drep) - (1 / a)) * (1 / pow(drep, 3))) * (robot_pos.x - obst[i].x);
        DVrepy = -Krep * (((1 / drep) - (1 / a)) * (1 / pow(drep, 3))) * (robot_pos.y - obst[i].y);
      }
      else
      {
        DVrepx = 0;
        DVrepy = 0;
      }

      DVrepxS += DVrepx;
      DVrepyS += DVrepy;
    }

    DVx = DVattx + DVrepxS;
    DVy = DVatty + DVrepyS;

    theta = PI + atan2(DVy, DVx);
    robot_pos.x += step * cos(theta);
    robot_pos.y += step * sin(theta);

    double dist_targ = dist(robot_pos, target_pt);
   
    // cout << dist(robot_pos, target_pt) << "    " << robot_pos << "     " << target_pt << endl;
    if (dist_targ < 2 * step || (dist_targ > datt && count>10))
      break;
    traj.push_back(robot_pos);
    count++;
  }

  return traj;
}

/*--------------------------------------------------*/


vector<Point2f> obstacles(vector<Map> maps)
{

  Mat hsv, obstacles;
  vector<Point2f> obst;
  for (size_t j = 0; j < maps.size(); j++)
  {

    Point decideMap = maps[j].global_id;

    cvtColor(maps[j].map_draw, hsv, COLOR_BGR2HSV);

    Mat mask1, mask2;
    // Creating masks to detect the upper and lower red color.
    inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), mask1);
    inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255), mask2);
    obstacles = mask1 + mask2;

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // Find contours
    findContours(obstacles, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));

    Mat drawing = Mat::zeros(obstacles.size(), CV_8UC3);

    for (size_t i = 0; i < contours.size(); i++)
    {
      drawContours(drawing, contours, i, Scalar(255, 255, 0), 1, 8, hierarchy, 0, Point());
      for (size_t j = 0; j < contours[i].size(); j++)

        obst.push_back(toGlobalMap(contours[i][j], decideMap));
    }
    //  imwrite("imgs/traj" + to_string(j) + ".jpg", drawing);
  }
  return obst;
}

/*--------------------------------------------------*/


Point2f targetPoint(struct Position *pos, vector<Map> maps, vector<Point2f> visited_targets)
{

  double d;
  double min_d = 1000;
  int map_key = 0;

  Point targ, targ_final, targ_local;

  for (size_t key = 0; key < maps.size(); key++)
  {
    Mat hsv, unexplored, free;
    cvtColor(maps[key].map_draw, hsv, COLOR_BGR2HSV);

    inRange(hsv, Scalar(0, 0, 230), Scalar(180, 25, 255), free);

    vector<vector<Point>> contours_free;
    vector<Vec4i> hierarchy;

    // Find contours
    findContours(free, contours_free, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    Point2f map_pos = globalRobotPos(pos, maps);

    /// Draw contours
    Mat drawing_free = Mat::zeros(free.size(), CV_8UC3);

    Scalar red = Scalar(0, 0, 255);

    for (size_t i = 0; i < contours_free.size(); i++)
      drawContours(drawing_free, contours_free, (int)i, red, 1, LINE_8, hierarchy, 0);

    imwrite("free_contours/" + to_string(key) + ".jpg", drawing_free);

    for (size_t i = 0; i < contours_free.size(); i++)
      for (size_t j = 0; j < contours_free[i].size(); j++)
      {

        bool check = true;

        for (int valx = -4; valx < 5; valx++)
          for (int valy = -4; valy < 5; valy++)
          {

            if ((contours_free[i][j].y + valy) < 0 || (contours_free[i][j].x + valx) < 0 || (contours_free[i][j].y + valy) > 99 || (contours_free[i][j].x + valx) > 99 ||
                maps[key].map.at<double>(contours_free[i][j].y + valy, contours_free[i][j].x + valx) > OCC_THRESH)
            {
              check = false;
            }
          }

        if (check)
        {
          Point2f global_targ = toGlobalMap(contours_free[i][j], maps[key].global_id);
          d = dist(global_targ, map_pos);
          if (d < min_d)
          {

            min_d = d;
            targ = global_targ;
            map_key = key;
            targ_local = contours_free[i][j];
          }
        }
      }
  }

  double d2 = 0, max_d2 = 0;
  for (int k = -5; k < 6; k++)
    for (int l = -5; l < 6; l++)
    {
      if ((targ_local.y + l) > 0 && (targ_local.x + k) > 0 && (targ_local.y + l) < 100 && (targ_local.x + k) < 100 &&
          maps[map_key].map.at<double>(targ_local.y + l, targ_local.x + k) < FREE_THRESH)
      {

        Point2f new_pt = Point2f(targ_local.x + k, targ_local.y + l);
        Point2f new_global_targ = toGlobalMap(new_pt, maps[map_key].global_id);
        bool check2 = true;
        for (auto vec:visited_targets){
          if(dist(vec, new_global_targ) < 5)
            check2 = false;
        }
       // if (!(find(visited_targets.begin(), visited_targets.end(), new_global_targ) != visited_targets.end()))
       if(check2)
        {

          d2 = dist(new_global_targ, targ);
          if (d2 > max_d2)
          {
            max_d2 = d2;
            targ_final = new_global_targ;
          }
        }
      }
    }
    
  return Point2f(targ_final.x, targ_final.y);
}
/*--------------------------------------------------*/

Point2f globalRobotPos(struct Position *pos, vector<Map> maps)
{
  Point decide_map = Point(floor((pos->xPos + 0.5) / MAP_SZ), floor((0.5 - pos->zPos) / MAP_SZ));

  double xmap = 0, ymap = 0;

  for (auto map : maps)
  {
    if (decide_map == map.id)
    {

      xmap = MAP_SZ / 2 + (pos->xPos * MAP_SZ) + MAP_SZ * fabs(decide_map.x) + MAP_SZ * map.global_id.x;
      ymap = MAP_SZ / 2 - (pos->zPos * MAP_SZ) + MAP_SZ * fabs(decide_map.y) + MAP_SZ * map.global_id.y;

      break;
    }
  }
  return Point2f(xmap, ymap);
}
/*--------------------------------------------------*/

Point2f toGlobalMap(Point pt, Point decide_map)
{
  double xpos, ypos;

  xpos = pt.x + decide_map.x * MAP_SZ;
  ypos = pt.y + decide_map.y * MAP_SZ;

  return Point2f(xpos, ypos);
}

/*--------------------------------------------------*/
void drawTrajectory( vector<Point2f> traj, Mat map ){

  for (auto vec : traj)
  {
    circle(map, vec, 1, Scalar(255, 0, 0));
  }
  imwrite("path/path"+ to_string(readRobotId()) +".jpg", map);
  
  return;
}