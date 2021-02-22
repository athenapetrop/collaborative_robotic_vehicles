/*   @File: a_star.cpp
*    @Details: Class that finds the path with A* search algorithm
*/

class AstarAlgorithm
{
  
private:

      struct Cell
      {
        bool isObstacle = false;			// Is the node an obstruction?
        bool isVisited = false;			// Have we searched this node before?
        float distGlobalGoal;				// Distance to goal so far
        float distLocalGoal;				// Distance to goal if we took the alternative route
        int x;					// Nodes position in 2D space
        int y;
        vector<Cell*> vecNeighbours;	                   // Connections to neighbours
        Cell* parent;			         // Node connecting to this node that offers shortest parent
      };

      Cell *nodes = nullptr;
      int nMapWidth = 100;
      int nMapHeight = 100;
      Cell *nodeStart = nullptr;
      Cell *nodeEnd = nullptr;
      Mat map_draw;  
      int pos_count=0;
      Cell *path = nullptr;
		
public:	
      
      bool MapCreate(vector<Point2f> obst, Point2f target, Point2f mypos, double width, double height)
      {
        nMapWidth = width;
        nMapHeight = height;

        // Create a 2D array of nodes 
        nodes = new Cell[nMapWidth * nMapHeight];
        for (int x = 0; x < nMapWidth; x++)
          for (int y = 0; y < nMapHeight; y++){
            nodes[y * nMapWidth + x].x = x; 
            nodes[y * nMapWidth + x].y = y;
            nodes[y * nMapWidth + x].isObstacle = false;
            nodes[y * nMapWidth + x].parent = nullptr;
            nodes[y * nMapWidth + x].isVisited = false;
            }
            
        // Create connections
        for (int x = 0; x < nMapWidth; x++)
          for (int y = 0; y < nMapHeight; y++){
            if(y>0)
                nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x + 0)]);
            if(y<nMapHeight-1)
                nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x + 0)]);
            if (x>0)
                nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x - 1)]);
            if(x<nMapWidth-1)
                nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x + 1)]);
          }
          
        //Create the obstacles
        for (auto vec:obst){
            nodes[(int)vec.y*nMapWidth + (int)vec.x].isObstacle = true;
        }
        
        // Manually position the start and end markers so they are not nullptr
        nodeStart = &nodes[(int)mypos.y*nMapWidth + (int)mypos.x];
        nodeEnd = &nodes[(int)target.y*nMapWidth + (int)target.x];
        
        draw();
        
        return true;
      }
           
      /***********************************************************************************************************/

      bool Solve_AStar()
      {
      
        // Reset Navigation Graph - default all node states
        for (int x = 0; x < nMapWidth; x++)
            for (int y = 0; y < nMapHeight; y++){
                nodes[y*nMapWidth + x].isVisited = false;
                nodes[y*nMapWidth + x].distGlobalGoal = INFINITY;
                nodes[y*nMapWidth + x].distLocalGoal = INFINITY;
                nodes[y*nMapWidth + x].parent = nullptr;	// No parents
            }
  
        auto distance = [](Cell* a, Cell* b){
            return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
        };
  
        auto heuristic = [distance](Cell* a, Cell* b){
            return distance(a, b);
        };
  
        // Setup starting conditions
        Cell *nodeCurrent = nodeStart;
        nodeStart->distLocalGoal = 0.0f;
        nodeStart->distGlobalGoal = heuristic(nodeStart, nodeEnd);
        
        // Add start node to not tested list - this will ensure it gets tested.
        // As the algorithm progresses, newly discovered nodes get added to this
        // list, and will themselves be tested later
        list<Cell*> listNotTestedNodes;
        listNotTestedNodes.push_back(nodeStart);
        
        // if the not tested list contains nodes, there may be better paths
        // which have not yet been explored. However, we will also stop 
        // searching when we reach the target - there may well be better
        // paths but this one will do - it wont be the longest.
        
        while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd){
        
            // Sort Untested nodes by global goal, so lowest is first
            listNotTestedNodes.sort([](const Cell* lhs, const Cell* rhs){ 
                return lhs->distGlobalGoal < rhs->distGlobalGoal; } );
            
            // Front of listNotTestedNodes is potentially the lowest distance node. Our
            // list may also contain nodes that have been visited, so ditch these...
            while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->isVisited)
                listNotTestedNodes.pop_front();
                
            // ...or abort because there are no valid nodes left to test
            if (listNotTestedNodes.empty())
                break;
            
            nodeCurrent = listNotTestedNodes.front();
            nodeCurrent->isVisited = true; // We only explore a node once
            
            // Check each of this node's neighbours...
            for (auto nodeNeighbour : nodeCurrent->vecNeighbours) {
            
                // ... and only if the neighbour is not visited and is 
                // not an obstacle, add it to NotTested List
                if (!nodeNeighbour->isVisited && nodeNeighbour->isObstacle == false){
                    if(checkNeighbours(nodeNeighbour)){
                      listNotTestedNodes.push_back(nodeNeighbour);
                    }
                }
                    
                // Calculate the neighbours potential lowest parent distance
                float possiblyLowerGoal = nodeCurrent->distLocalGoal + distance(nodeCurrent, nodeNeighbour);
              
                // If choosing to path through this node is a lower distance than what 
                // the neighbour currently has set, update the neighbour to use this node
                // as the path source, and set its distance scores as necessary
                if (possiblyLowerGoal < nodeNeighbour->distLocalGoal){
                   nodeNeighbour->parent = nodeCurrent;
                   nodeNeighbour->distLocalGoal = possiblyLowerGoal;
                   
                   // The best path length to the neighbour being tested has changed, so
                   // update the neighbour's score. The heuristic is used to globally bias
                   // the path algorithm, so it knows if its getting better or worse. At some
                   // point the algo will realise this path is worse and abandon it, and then go 
                   // and search along the next best path.
                   nodeNeighbour->distGlobalGoal = nodeNeighbour->distLocalGoal + heuristic(nodeNeighbour, nodeEnd);
               }
           }
       }
       draw();	
       return true;
     }
     
     /***********************************************************************************************************/
     
     bool checkNeighbours(Cell* nodeNeighbour){

       for (auto neighbour : nodeNeighbour->vecNeighbours) {
            
           if (neighbour->isObstacle == true){
               return false;
           }
           for (auto neighbour2 : neighbour->vecNeighbours) {
            
             if (neighbour2->isObstacle == true){
                 return false;
             }
             for (auto neighbour3 : neighbour2->vecNeighbours) {
            
               if (neighbour3->isObstacle == true){
                   return false;
               }
               for (auto neighbour4 : neighbour3->vecNeighbours) {
            
                 if (neighbour4->isObstacle == true){
                     return false;
                 }
                 for (auto neighbour5 : neighbour4->vecNeighbours) {
            
                   if (neighbour5->isObstacle == true){
                       return false;
                   }
           
                 }
           
               }
           
             }
           
           }
       }
       return true;
     }

    /***********************************************************************************************************/
     void make_path(){
         Cell *p = nodeEnd;
         int count=0;
                  
         while (p->parent != nullptr){
             count++;
             p=p->parent;
         }
         p = nodeEnd;
         path = new Cell[count+1];
         
         for (int x = count; x >=0 ; x--){	
           path[x] = *p;
           if(p->parent != nullptr){
               p=p->parent;
           }
         }
     }
             
    /***********************************************************************************************************/
     
     vector<Point2f> path_points(){
              
         Cell *p = nodeEnd;
         vector<Point2f> path1;
        
         while (p->parent != nullptr){
           path1.push_back( Point2f(p->x,p->y) );
           p=p->parent;
         }
         reverse(path1.begin(),path1.end());
         return path1;
     }
     
    /***********************************************************************************************************/
     
     void draw(){
        map_draw = Mat::zeros(nMapHeight, nMapWidth, CV_8UC3);
        Scalar white = Scalar(255,255,255);
        Scalar gray = Scalar(128,128,128);
        Scalar blue = Scalar(255,0,0);
        Scalar green = Scalar(0,255,0);
        Scalar red = Scalar(0,0,255);
                  
        int pos;
        for (int x = 0; x < nMapWidth; x++)
          for (int y = 0; y < nMapHeight; y++) {
    				
            pos = y*nMapWidth + x;
            if (nodes[pos].isObstacle == true){
                          
              circle( map_draw,
                      Point( nodes[pos].x , nodes[pos].y ),
                      2,
                      blue,
                      FILLED,
                      LINE_8 );
            }   
          }
          if (nodeEnd != nullptr) {
            Cell *p = nodeEnd;
            while (p->parent != nullptr){
    
              circle( map_draw,
                      Point( p->x, p->y ),
                      2,
                      Scalar(100,0,255),
                      FILLED,
                      LINE_8 );                        
                              				
              circle( map_draw,
                      Point(  p->x  , p->y),
                      2,
                      green,
                      FILLED,
                      LINE_8 );
                      
              // Set next node to this node's parent
    	    p = p->parent;
            }
    
            circle( map_draw,
                    Point( nodeStart->x , nodeStart->y ),
                    4,
                    Scalar(200,0,150),
                    FILLED,
                    LINE_8 ); 
                             
            circle( map_draw,
                    Point( nodeEnd->x, nodeEnd->y ),
                    2,
                    Scalar(200,0,150),
                    FILLED,
                    LINE_8 ); 
                
          }
          
          // namedWindow("Path", WINDOW_NORMAL);
         //  imshow("Path",map_draw);
         //  waitKey(TIME_STEP);
           imwrite("path.jpg",map_draw);
      }
         
};


