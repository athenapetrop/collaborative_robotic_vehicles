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
                      struct CamParam *cam,
                      Rect robot );
                      
bool robotRecognition( Rect &robot, 
                       Mat img );
                       
Rect getBoundingRect( Mat mask );

Rect checkIfRobotInFrame(Mat img);


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
                     Mat map,
                     Point2f target_pt );
                     
void drawObstacles(Mat global_map_draw, 
                   vector<Point2f> obstacles); 
   
/*--------------------------------------------------*/
/*
*    Mapping
*/

void mapping ( struct Position *pos, 
               vector<double> depth, 
               vector<double> angle, 
               vector<Map> &maps,
               Point2f diff );  

void drawMaps ( vector<Map> maps,
                int robot_id);

int  find_map_id ( Point decide_map, 
                   vector<Map> &maps );

Point2f robotPosInMap(struct Position *pos,
                      Point2f diff );

void makeFinalMap( vector<Map> maps, 
                   Point sz, 
                   int robot_id,
                   Mat& globalMap,
                   Mat& globalMapDraw );

Point findFinalMapSZ(vector<Map> maps);

void updateGlobalId(vector<Map>& maps);

void adjustMap( Point3f diff,
                Point sz,
                Mat &globalMap, 
                vector<Map> &maps,
                struct Position originPos,
                vector<Map> receivedMap );

void rotate( double angle, 
             Mat &globalMap,
             vector<Map> maps,
             double xdiff, 
             double ydiff );
             
            
void translate( double x, 
                double y,
                Mat &globalMap );
                
void transform_map(double angle, 
                   double x, 
                   double y, 
                   Mat &globalMap,
                   vector<Map> maps);
              
void mergeMaps( vector<Map> &maps, 
                vector<Map> receivedMaps );

void showmap(Mat temp, const string& name);
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

Point3f posInNewCoordinateSystem(struct Position *received_pos, 
                                 float received_distance, 
                                 struct Position *pos,
                                 struct Position &originPos );

Position findInitialPos(double x_new, 
                    double z_new, 
                    double th_new, 
                    struct Position *pos );
                    
double findOtherRobotsDist( Rect robot, 
                            double focal_y );
