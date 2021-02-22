// FUNCTIONS 

/*
*    Util
*/

void readMatrix( string img_path, 
                 Mat &matr );
                 
double square( double val );

int readRobotId();

double dist( Point2f point1, 
             Point2f point2 );
             
void initCameraParams(struct CamParam *cam);

void initOdom(struct Odometry *odom);

/*--------------------------------------------------*/
/*
*    Camera Params
*/

Mat  find_contour_down ( Mat frame );

vector<Point2f> shiTomasi  ( Mat img );

void calculate_depth( Mat img, 
                      vector<double> &depth, 
                      vector<double> &angle, 
                      struct CamParam *cam );
                      
bool robotRecognition( Rect &robot, 
                       Mat img );
                       
Rect getBoundingRect( Mat mask );

/*--------------------------------------------------*/
/*
*    Communication
*/


/*--------------------------------------------------*/
/*
*    Path Planning
*/
Point2f targetPoint( struct Position *pos, 
                     vector<Map> maps, 
                     vector<Point2f> visited_targets );

vector<Point2f> artificial_potential_field( struct Position *pos, 
                                            Point2f target_pt, 
                                            vector<Point2f> obst, 
                                            vector<Map> maps );

vector<Point2f > obstacles( vector<Map> maps );

Point2f globalRobotPos( struct Position *pos, 
                        vector<Map> maps );

Point2f toGlobalMap( Point pt, 
                     Point decide_map );
                     
void drawTrajectory( vector<Point2f> traj, 
                     Mat map );
/*--------------------------------------------------*/
/*
*    Mapping
*/

void mapping ( struct Position *pos, 
               vector<double> depth, 
               vector<double> angle, 
               vector<Map> &maps );  

void show_maps ( vector<Map> maps, 
                 const String& winname, 
                 int id,
                 int robot_id);

int  find_map_id ( Point decide_map, 
                   vector<Map> &maps );

Point2f robotPosInMap(struct Position *pos );

void makeFinalMap( vector<Map> maps, 
                   Point sz, 
                   int robot_id,
                   Mat& globalMap,
                   Mat& globalMapDraw );

Point findFinalMapSZ(vector<Map> maps);

void updateGlobalId(vector<Map>& maps);

void adjustMap( Point3f diff,
                Point sz,
                Mat globalMap, 
                vector<Map> maps );

Mat rotate(double angle, 
            Mat globalMap);
            
Mat translate(double x, 
              double y,
              Mat globalMap);

/*--------------------------------------------------*/
/*
*    Odometry
*/
void robot_controller( Point2f mypos, 
                       double theta, 
                       int pos_count, 
                       vector<Point2f> path, 
                       bool &atgoal, 
                       Point2f &speeds );
                       
static void compute_pos( struct Position *pos, 
                         struct Encoders *enc, 
                         struct Odometry *od );
                         
void setSpeed( double speed_l,
               double speed_r, 
               struct Odometry *od);
               
bool isFullRotation( double currTheta, 
                     double &prevTheta, 
                     double &deltaTheta );

