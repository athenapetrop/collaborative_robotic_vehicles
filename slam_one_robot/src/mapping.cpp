/*   @File: mapping.cpp
*    @Details: Functions about mapping
*/


void mapping(struct Position * pos, vector < double > depth, vector < double > angle, vector<Map> & maps)
{

    // 100 pixels -> 1 m , ara akribeia 0.01m gia kathe pixel

    // odds for log-map
    double log_odd_occ = 0.9;
    double log_odd_free = 0.5;
    double log_free, log_occ;

    // robot position in map
    Point2f robot_pos = robotPosInMap(pos);
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

  //decide in which map
  decide_map = Point(floor(x_pt / MAP_SZ), floor(y_pt / MAP_SZ));

  log_occ = log_odd_occ / dist;
  log_free = log_odd_free / dist;

  //occupied

  // more likely to be in the central map - save some time
  if (decide_map == Point(0, 0))
    map_id = 0;

  else
    map_id = find_map_id(decide_map, maps);

  maps[map_id].map.at<double>(-MAP_SZ * decide_map.y + y_pt, x_pt - MAP_SZ * decide_map.x) += log_occ;

  //free
  double d = 0;
  while (d < (fabs(depth[i]) - ROBOT_RADIUS)) {
    x_pt = robot_pos.x + d * cos(angle[i]) * sin(angle[i] + pos -> theta) * MAP_SZ;
    y_pt = robot_pos.y - d * cos(angle[i]) * cos(angle[i] + pos -> theta) * MAP_SZ;

    decide_map = Point(floor(x_pt / MAP_SZ), floor(y_pt / MAP_SZ));

    if (decide_map == Point(0, 0))
      map_id = 0;

    else
      map_id = find_map_id(decide_map, maps);

    maps[map_id].map.at<double>(-MAP_SZ * decide_map.y + y_pt, x_pt - MAP_SZ * decide_map.x) -= log_free;

    d += 0.01;
  }
}
return;
}

/*--------------------------------------------------*/


void show_maps(vector < Map > maps, const String & winname, int id, int robot_id)
{
  Scalar white = Scalar(255, 255, 255);
  // Scalar gray = Scalar(128, 128, 128);
  //  Scalar blue = Scalar(255, 0, 0);
  //  Scalar green = Scalar(0, 255, 0);
  Scalar red = Scalar(0, 0, 255);

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

  //namedWindow(winname, WINDOW_NORMAL);
  //imshow(winname, maps[id].map_draw);
  waitKey(TIME_STEP);
  imwrite("maps/robot" + to_string(robot_id) + "/" + winname + ".jpg", maps[id].map_draw);

  ofstream outStream("maps_data/robot" + to_string(robot_id) + "/" + "map" + winname + ".csv");
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

Point2f robotPosInMap(struct Position * pos)
{
  Point2f decide_map = Point2f(floor((pos -> xPos + 0.5) / MAP_SZ), floor((0.5 - pos -> zPos) / MAP_SZ));

  double xmap, ymap;
  xmap = MAP_SZ / 2 + (pos -> xPos * MAP_SZ) + MAP_SZ * decide_map.x + MAP_SZ * fabs(decide_map.x);
  ymap = MAP_SZ / 2 - (pos -> zPos * MAP_SZ) + MAP_SZ * decide_map.y + MAP_SZ * fabs(decide_map.y);

  return Point2f(xmap, ymap);
}

/*--------------------------------------------------*/

void makeFinalMap(vector < Map > maps, Point sz, int robot_id, Mat & globalMap, Mat & globalMapDraw)
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

Mat translate(double x, double y, Mat globalMap){

  Mat src = globalMap.clone();
  Mat dst;
  double data[6] = { 1, 0, x * MAP_SZ, 0, 1, - y * MAP_SZ
};
Mat M = Mat(2, 3, CV_64FC1, data);
Size dsize = Size(src.rows, src.cols);
// You can try more different parameters
warpAffine(src, dst, M, dsize, INTER_LINEAR, BORDER_CONSTANT);

return dst;

         /* Mat dst2=Mat::zeros(dsize, CV_8UC3);
              Scalar white = Scalar(255, 255, 255);
              Scalar red = Scalar(0, 0, 255);
              for (int i = 0; i < dsize.width; i++)
                  for (int j = 0; j < dsize.height; j++)
                  {
                    if (dst.at<double>(i, j) > OCC_THRESH)
                    {
                      circle(dst2, Point(j, i), 1, red);
                    }
              
                    else if (dst.at<double>(i, j) < FREE_THRESH)
                    {
                      circle(dst2, Point(j, i), 1, white);
                    }
                  }
        
          imwrite("maps/robot" + to_string(1) + "/" + to_string(0) + "_trans.jpg", dst2);*/
      }
/*--------------------------------------------------*/

Mat rotate(double angle, Mat globalMap) {

  Mat dst;
  Mat src = globalMap.clone();
  Mat r = getRotationMatrix2D(Point2f(), -angle * 180 / PI, 1.0);

  //4 coordinates of the image
  vector < Point2f > corners(4);
  corners[0] = Point2f(0, 0);
  corners[1] = Point2f(0, src.rows);
  corners[2] = Point2f(src.cols, 0);
  corners[3] = Point2f(src.cols, src.rows);

  vector < Point2f > cornersTransform(4);
  transform(corners, cornersTransform, r);

  //Copy the 2x3 transformation matrix into a 3x3 transformation matrix
  Mat H = Mat:: eye(3, 3, CV_64F);
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      H.at<double>(i, j) = r.at<double>(i, j);
    }
  }

  double offsetX = 0.0, offsetY = 0.0, maxX = 0.0, maxY = 0.0;
  //Get max offset outside of the image and max width / height
  for (size_t i = 0; i < 4; i++) {
    if (cornersTransform[i].x < offsetX) {
      offsetX = cornersTransform[i].x;
    }

    if (cornersTransform[i].y < offsetY) {
      offsetY = cornersTransform[i].y;
    }

    if (cornersTransform[i].x > maxX) {
      maxX = cornersTransform[i].x;
    }

    if (cornersTransform[i].y > maxY) {
      maxY = cornersTransform[i].y;
    }
  }

  offsetX = -offsetX;
  offsetY = -offsetY;
  maxX += offsetX;
  maxY += offsetY;

  Size size_warp(maxX, maxY);

  //Create the transformation matrix to be able to have all the pixels
  Mat H2 = Mat:: eye(3, 3, CV_64F);
  H2.at<double>(0, 2) = offsetX;
  H2.at<double>(1, 2) = offsetY;

  warpPerspective(src, dst, H2 * H, size_warp);

  return dst;

  /*   Mat dst2=Mat::zeros(size_warp, CV_8UC3);
     Scalar white = Scalar(255, 255, 255);
     Scalar red = Scalar(0, 0, 255);
     for (int i = 0; i < size_warp.width; i++)
         for (int j = 0; j < size_warp.height; j++)
         {
           if (dst.at<double>(i, j) > OCC_THRESH)
           {
             circle(dst2, Point(j, i), 1, red);
           }
     
           else if (dst.at<double>(i, j) < FREE_THRESH)
           {
             circle(dst2, Point(j, i), 1, white);
           }
         }
 
 imwrite("maps/robot" + to_string(1) + "/" + to_string(0) + "_rot.jpg", dst2);*/

}

/*--------------------------------------------------*/

void adjustMap(Point3f diff, Mat globalMap, vector < Map > maps, Point sz){

  int xSz = (int)ceil((float)(globalMap.cols / MAP_SZ));
  int ySz = (int)ceil((float)(globalMap.rows / MAP_SZ));


  if (xSz == sz.x && ySz == sz.y) {

    for (size_t i = 0; i < maps.size(); i++) {
      globalMap(Rect(maps[i].global_id.x * MAP_SZ, maps[i].global_id.y * MAP_SZ, MAP_SZ, MAP_SZ)).copyTo(maps[i].map);

    }

    return;
  }
  else {
    cout << readRobotId() << "   diff size" << endl;
    Point index = Point(0, 0);
    if (diff.x < 0) index.x = -1;
    else index.x = 1;

    if (diff.y < 0) index.y = 1;
    else index.y = -1;

    if (xSz > sz.x) {


    }

    if (ySz != sz.y) {


    }





  }
  return;
}
