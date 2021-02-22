/*   @File: mapping.cpp
*    @Details: Functions about mapping
*/


void mapping(struct Position * pos, vector < double > depth, vector < double > angle, vector<Map> & maps, Point2f diff)
{

    /* 100 pixels -> 1 m , ara akribeia 0.01m gia kathe pixel */

    /* odds for log-map */
    double log_odd_occ = 0.9;
    double log_odd_free = 0.5;
    double log_free, log_occ;

    /* robot position in map */
    
    Point2f robot_pos = robotPosInMap(pos,diff);

    double x_pt, y_pt;
    double dist;
    int map_id;
    Point decide_map;
    Point2f radius_displacement;

    for(size_t i = 0; i<depth.size(); i++)
    {
      dist = (fabs(depth[i]) - ROBOT_RADIUS) * cos(angle[i]);
      
    
      x_pt = robot_pos.x + dist * sin(angle[i] + pos -> theta) * MAP_SZ;
      y_pt = robot_pos.y - dist * cos(angle[i] + pos -> theta) * MAP_SZ;
    
      /*decide in which map */
      decide_map = Point(floor(x_pt / MAP_SZ), floor(y_pt / MAP_SZ));

      log_occ = log_odd_occ / dist;
      log_free = log_odd_free / dist;
    
      /* occupied */
    
      map_id = find_map_id(decide_map, maps);
    
      maps[map_id].map.at<double>(-MAP_SZ * decide_map.y + y_pt, x_pt - MAP_SZ * decide_map.x) += log_occ;
    
      /* free */
      double d = 0;
      while (d < (fabs(depth[i]) - ROBOT_RADIUS)) {
        x_pt = robot_pos.x + d * cos(angle[i]) * sin(angle[i] + pos -> theta) * MAP_SZ;
        y_pt = robot_pos.y - d * cos(angle[i]) * cos(angle[i] + pos -> theta) * MAP_SZ;
    
        decide_map = Point(floor(x_pt / MAP_SZ), floor(y_pt / MAP_SZ));
    

        map_id = find_map_id(decide_map, maps);
    
        maps[map_id].map.at<double>(-MAP_SZ * decide_map.y + y_pt, x_pt - MAP_SZ * decide_map.x) -= log_free;
    
        d += 0.01;
      }
    }
    return;
}

/*--------------------------------------------------*/


void drawMaps(vector < Map > maps, int robot_id)
{
  Scalar white = Scalar(255, 255, 255);
  Scalar red = Scalar(0, 0, 255);
  
  for (size_t id = 0; id < maps.size(); id++)
  {

  for (int i = 0; i < MAP_SZ; i++)
    for (int j = 0; j < MAP_SZ; j++)
    {
      if (maps[id].map.at<double>(i, j) > OCC_THRESH) {
        circle(maps[id].map_draw, Point(j, i), 1, red);
      }
  
      else if (maps[id].map.at<double>(i, j) < FREE_THRESH) {
        circle(maps[id].map_draw, Point(j, i), 1, white);
      }
    }
  
  imwrite("maps/robot" + to_string(robot_id) + "/" + to_string(maps[id].id.x) + to_string(maps[id].id.y) + ".jpg", maps[id].map_draw);

  ofstream outStream("maps_data/robot" + to_string(robot_id) + "/" + "map" + to_string(id) + ".csv");
  uint16_t rows = MAP_SZ;
  uint16_t columns = MAP_SZ;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < columns; c++)
    {
      double value = maps[id].map.at<double>(r, c);
      outStream << value << ",";
    }
    outStream << endl;
  }
  outStream.close();
  }
  return;
}

/*--------------------------------------------------*/


int find_map_id(Point decide_map, vector<Map> & maps)
{

    /*           map ids
    
       (-1,-1)  (0,-1)  (1,-1)
        (-1,0)  (0,0)   (1,0) 
        (-1,1)  (0,1)   (1,1)
    
    */

    for(size_t i = 0; i<maps.size(); i++)
    {
      if (maps[i].id == decide_map) {
        return i;
      }
    }
    
    Map map_new;
    map_new.id = decide_map;
    maps.push_back(map_new);
    
    return (maps.size() - 1);
}

/*--------------------------------------------------*/

Point2f robotPosInMap(struct Position * pos, Point2f diff)
{
  Point2f decide_map = Point2f(floor((pos -> xPos + 0.5-diff.x) / MAP_SZ), floor((0.5+diff.y - pos -> zPos) / MAP_SZ));

  double xmap, ymap;
  xmap = MAP_SZ / 2 + (pos -> xPos * MAP_SZ) + MAP_SZ * decide_map.x + MAP_SZ * fabs(decide_map.x);
  ymap = MAP_SZ / 2 - (pos -> zPos * MAP_SZ) + MAP_SZ * decide_map.y + MAP_SZ * fabs(decide_map.y);

  return Point2f(xmap, ymap);
}

/*--------------------------------------------------*/

void makeFinalMap(vector < Map > maps, Point sz, int robot_id, Mat &globalMap, Mat &globalMapDraw)
{

  int x, y;
  globalMap = Mat:: zeros(sz.y * MAP_SZ, sz.x * MAP_SZ, maps[0].map.type());
  globalMapDraw = Mat:: zeros(sz.y * MAP_SZ, sz.x * MAP_SZ, maps[0].map_draw.type());

  for (size_t i = 0; i < maps.size(); i++)
  {

    x = maps[i].global_id.x * MAP_SZ;
    y = maps[i].global_id.y * MAP_SZ;

    maps[i].map.copyTo(globalMap(Rect(x, y, MAP_SZ, MAP_SZ)));
    maps[i].map_draw.copyTo(globalMapDraw(Rect(x, y, MAP_SZ, MAP_SZ)));

  }

  imwrite("global_maps/final_map" + to_string(robot_id) + ".jpg", globalMapDraw);
  
 /* namedWindow("Map of Robot "+to_string(readRobotId()), WINDOW_NORMAL);            
  imshow("Map of Robot "+to_string(readRobotId()), globalMapDraw);
  waitKey(TIME_STEP);
*/
  return;
}

/*--------------------------------------------------*/


Point findFinalMapSZ(vector < Map > maps)
{

  int x_sz = 1, y_sz = 1;
  vector < int > xkeys;
  vector < int > ykeys;

  xkeys.push_back(maps[0].id.x);
  ykeys.push_back(maps[0].id.y);

  for (size_t i = 1; i < maps.size(); i++)
  {
    if (!(find(xkeys.begin(), xkeys.end(), maps[i].id.x) != xkeys.end())) {
      x_sz++;
      xkeys.push_back(maps[i].id.x);
    }


    if (!(find(ykeys.begin(), ykeys.end(), maps[i].id.y) != ykeys.end())) {
      y_sz++;
      ykeys.push_back(maps[i].id.y);
    }

  }

  return Point(x_sz, y_sz);
}

/*--------------------------------------------------*/


void updateGlobalId(vector<Map> & maps)
{

    vector<Map> xkeys_sorted;
    vector<Map> ykeys_sorted;

    for(size_t i = 0; i<maps.size(); i++)
    {
      xkeys_sorted.push_back(maps[i]);
      ykeys_sorted.push_back(maps[i]);
    }
    
    sort(xkeys_sorted.begin(), xkeys_sorted.end(), [](Map a, Map b) {
      return a.id.x < b.id.x;
    });
    sort(ykeys_sorted.begin(), ykeys_sorted.end(), [](Map a, Map b) {
      return a.id.y < b.id.y;
    });
    
    int x = 0, x_bef = 0;
    int i = 100000;
    
    vector<Map>:: iterator it;
    
    for (auto xsort : xkeys_sorted)
    {
      for (it = maps.begin(); it != maps.end(); it++) {
    
        if (xsort.id == it -> id && i != it -> id.x) {
          x_bef = x;
          i = it -> id.x;
          it -> global_id.x = x;
          x++;
        }
        else if (xsort.id == it -> id) {
          i = it -> id.x;
          it -> global_id.x = x_bef;
        }
      }
    }
    
    int y = 0, y_bef = 0;
    i = 100000;
    
    for (auto ysort : ykeys_sorted)
    {
      for (it = maps.begin(); it != maps.end(); it++) {
    
        if (ysort.id == it -> id && i != it -> id.y) {
          y_bef = y;
          i = it -> id.y;
          it -> global_id.y = y;
          y++;
        }
        else if (ysort.id == it -> id) {
          i = it -> id.y;
          it -> global_id.y = y_bef;
        }
      }
    }
    return;
}

/*--------------------------------------------------*/

void translate(double x, double y, Mat &globalMap){
  Mat src = globalMap.clone(); 
  cout<<"Robot "<<readRobotId()<<" -> Translating Map by "<<x<<" m in x and by "<<y<<" m in y axis"<<endl;

  double tx, ty, szx, szy;
  
  if(x > 0){
    szx = x * MAP_SZ;
  }else{
    szx = 0;
  }
  if(y > 0){
    szy = 0;
  }else{
    szy = fabs(y) * MAP_SZ;
  }
  tx = x * MAP_SZ;
  ty = -y * MAP_SZ;
  
  
  Size dsize = Size(src.size().width + szx , src.size().height + szy);
  Mat dst = Mat::zeros(dsize, CV_64FC1);
  
  double data[6] = { 1, 0, tx, 0, 1, ty };
  Mat M = Mat(2, 3, CV_64FC1, data);

  warpAffine(src, dst, M, dsize, INTER_LINEAR, BORDER_CONSTANT);
  //circle(dst, Point2f(100,100), 5 , Scalar(0,0,255),-1);
  globalMap = dst.clone();
  //showmap(dst,"translation");
  
  return;

}

/*--------------------------------------------------*/

void rotate(double angle, Mat &globalMap, vector<Map> maps, double xdiff, double ydiff) {
  Mat dst;
  Mat src = globalMap.clone();
  cout<<"Robot "<<readRobotId()<<" -> Rotating Map by "<< -angle*180/PI<<" degrees"<<endl;

  
    Point2f center;
    for(auto vec:maps){
      if(vec.id == Point(0,0)){
        center = Point2f((vec.global_id.x + 0.5) * MAP_SZ , (vec.global_id.y + 0.5) * MAP_SZ);
      }
    }
    Point2f center2 = Point2f(src.cols/2, src.rows/2);
    center2.x -= xdiff * MAP_SZ;
    center2.y += ydiff * MAP_SZ;
  Mat r = getRotationMatrix2D(center2, -angle * 180 / PI, 1.0);

  warpAffine(src, dst, r, Size(src.cols, src.rows));
  globalMap = dst.clone();
 Point2f rot_diff = Point2f(dst.size().width - src.size().width , dst.size().height - src.size().height);
 //cout<<rot_diff<<endl;
showmap(dst,"rot");
  return;

}

/*--------------------------------------------------*/

void transform_map(double angle, double x, double y, Mat &globalMap, vector<Map> maps){

  Mat src = globalMap.clone(); 
  cout<<"Robot "<<readRobotId()<<" -> Translating Map by "<<x<<" m in x and by "<<y<<" m in y axis"<<endl;
  cout<<"Robot "<<readRobotId()<<" -> Rotating Map by "<< -angle*180/PI<<" degrees"<<endl;

  double tx, ty, szx, szy;
  
  if(x > 0){
    szx = x * MAP_SZ;
  }else{
    szx = 0;
  }
  if(y > 0){
    szy = 0;
  }else{
    szy = fabs(y) * MAP_SZ;
  }
  tx = x * MAP_SZ;
  ty = -y * MAP_SZ;
//szx=0;szy=0;
  Size dsize = Size(src.size().width + szx , src.size().height + szy);
  Mat dst = Mat::zeros(dsize, CV_64FC1);
  
 /* double data[9] = { cos(angle), -sin(angle), tx, sin(angle), cos(angle), ty, 0, 0, 1 };
  Mat M = Mat(3, 3, CV_64FC1, data);*/
    double trans_data[9] = {1, 0, tx, 0, 1, ty, 0, 0, 1 };
  Mat T = Mat(3, 3, CV_64FC1, trans_data);
  
  double rot_data[9] = { cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1 };
  Mat R = Mat(3, 3, CV_64FC1, rot_data);
  warpPerspective(src, dst, T*R, dsize, INTER_LINEAR, BORDER_CONSTANT);
  
   /* Point2f center;
    for(auto vec:maps){
      if(vec.id == Point(0,0)){
        center = Point2f((vec.global_id.x + 0.5) * MAP_SZ , (vec.global_id.y + 0.5) * MAP_SZ);
      }
    }

    
  double rot_data[9] = { cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1 };
  Mat r = Mat(3, 3, CV_64FC1, rot_data);
  
  Mat point = Mat(3, 1, CV_64FC1, {center.x,center.y,1});
  cout<<point<<endl;
  Mat newp = r*point;
  cout<<newp<<endl;*/
  globalMap = dst.clone();
  showmap(dst,"transformed");
  
  return;

}

/*--------------------------------------------------*/

void adjustMap(Point3f diff, Mat &globalMap, vector < Map > &maps, Point sz, struct Position originPos, vector<Map> receivedMap){

  Point center;
    for(auto vec:maps){
      if(vec.id == Point(0,0)){
        center = Point((vec.global_id.x + 0.5) * MAP_SZ , (vec.global_id.y + 0.5) * MAP_SZ);
      }
    }

    /* We dont need old data, they are all contained in globalMap */
    maps.clear();
    
   
    int xleft, xright, ytop, ybottom;
    
    xleft = center.x - MAP_SZ / 2;
    int startx = xleft - ceil(xleft / MAP_SZ) * MAP_SZ;
    
    xright = globalMap.size().width - (center.x + MAP_SZ / 2);
    int endx = center.x + MAP_SZ / 2 + ceil(xright / MAP_SZ) * MAP_SZ;
    
    
    ytop = center.y - MAP_SZ / 2;
    int starty = ytop - ceil(ytop / MAP_SZ) * MAP_SZ;
 

    ybottom = globalMap.size().height - (center.y + MAP_SZ / 2);
    int endy = center.y + MAP_SZ / 2 + ceil(ybottom / MAP_SZ) * MAP_SZ ;

    /* if (0,0) roi is outside of the map */
    if(ybottom < 0){
      ybottom = globalMap.size().height + fabs(ybottom);
      endy = center.y + MAP_SZ/2 - MAP_SZ;
    }
    
    
    //cout<<"xleft = "<<xleft<<"   xright = "<<xright<<"   ytop = "<<ytop<<"   ybottom = "<<ybottom<<endl;
   // cout<<"startx = "<<startx<<"   endx = "<<endx<<"   starty = "<<starty<<"   endy = "<<endy<<endl;
    
    int xgl, ygl, xszgl, yszgl;
    int xloc, yloc, xszloc, yszloc;
    
    for(int x = startx; x <= endx; x+=100 ){
    
       if( x < 0){
          xgl = 0; xszgl = MAP_SZ - fabs(x); xloc = fabs(x); xszloc = MAP_SZ - fabs(x);
          
        }else if((x + MAP_SZ) > globalMap.size().width - 1){
          xgl = x; xszgl = globalMap.size().width - x; xloc = 0; xszloc = globalMap.size().width - x;
          
        }else{
          xgl = x; xszgl = MAP_SZ; xloc = 0; xszloc = MAP_SZ;
        }
        
        for(int y = starty; y <= endy; y+=100){
          Point id;
          if( y < 0){
            ygl = 0; yszgl = MAP_SZ - fabs(y); yloc = fabs(y); yszloc = MAP_SZ - fabs(y);
            
          }else if((y + MAP_SZ) > globalMap.size().height - 1){
            ygl = y; yszgl = globalMap.size().height - y; yloc = 0; yszloc = globalMap.size().height - y;
            
          }else{
            ygl = y; yszgl = MAP_SZ; yloc = 0; yszloc = MAP_SZ;
          }

          /* Find local id in new coordinate system based on produced global map */
          id.x = (int)floor((x - (center.x - MAP_SZ / 2)) / MAP_SZ);
          id.y = (int)floor((y - (center.y - MAP_SZ / 2)) / MAP_SZ);
     
          if(yszgl!=0 && xszgl!=0){
            Map map;
            map.id = id;
            globalMap(Rect( xgl, ygl, xszgl, yszgl )).copyTo(map.map(Rect( xloc, yloc, xszloc, yszloc )));
            
            maps.push_back(map);
          }
        }
    }

   
  return;
}

/*--------------------------------------------------*/

void mergeMaps(vector<Map> &maps, vector<Map> receivedMaps){

  cout<<"Robot "<<readRobotId()<<" -> Merging Maps" << endl;
  bool hasBeenMerged;

  for(auto received:receivedMaps){
  
    hasBeenMerged = false;
    for(auto map:maps){

      if(received.id == map.id){
                 
        hasBeenMerged = true;
        Mat merged;
        merged = received.map + map.map;
        merged.copyTo(map.map);
        
      }
    }
    if(!hasBeenMerged){
      maps.push_back(received);
    }
  }
  

  return;
}

/*--------------------------------------------------*/

void showmap(Mat temp, const string& name){
      Mat dst2=Mat::zeros(temp.size(), CV_8UC3);
              Scalar white = Scalar(255, 255, 255);
              Scalar red = Scalar(0, 0, 255);
              for (int i = 0; i < temp.size().height; i++)
                  for (int j = 0; j < temp.size().width; j++)
                  {
                    if (temp.at<double>(i, j) > OCC_THRESH)
                    {
                      circle(dst2, Point(j, i), 1, red);
                    }
              
                    else if (temp.at<double>(i, j) < FREE_THRESH)
                    {
                      circle(dst2, Point(j, i), 1, white);
                    }
                  }
        
          imwrite(name+".jpg", dst2);

}