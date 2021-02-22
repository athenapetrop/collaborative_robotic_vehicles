/*   @File: communication.cpp
*    @Details: Class about communication of the two robots
*/

      
class Communication{

  private:
      WbDeviceTag emitter;
      WbDeviceTag receiver;
      float *inbuffer;
      float received_distance = 0;
      Position received_pos = {0,0,0};
      bool communicating = false;
      bool adjusting_angle = false;
      vector<Map> receivedMaps;

      
  public:
      bool robot_found = false;
      bool mapAlert = false;
      bool firstMerge = true;
      bool gotPos = false;
      bool waitingStatus = false;
      bool otherFinished = false;
      
      void init(){
          /* Get emitter and receiver devices. */
          emitter = wb_robot_get_device("emitter");
          wb_emitter_set_channel(emitter, 1);
          receiver = wb_robot_get_device("receiver");
          wb_receiver_enable(receiver, TIME_STEP);
          
      }
      
   /*----------------------------------------------------------*/
      
      void emit(vector<float> msg){
         communicating = true;
         float message[msg.size()]; int i=0;
         for(auto m:msg){
           message[i] = m;
           i++;
         }
         wb_emitter_send(emitter, message, msg.size()*sizeof(float));
         
         if(message[0] == (float)CONTINUE){
           communicating = false;
         }
         return;
      }
      
    /*----------------------------------------------------------*/
     
      void receive(struct Odometry *od, vector<Map> &maps, struct Position *pos){
        while (wb_receiver_get_queue_length(receiver) > 0) {
          communicating = true;
          inbuffer = (float *)wb_receiver_get_data(receiver);
          
       
           if(inbuffer[0] == (float)SAW_YOU) {
             emit({GOT_IT});
             cout<<"Robot "<<readRobotId()<<" -> Received Dist: "<<inbuffer[1]<<endl;
             
             received_distance = inbuffer[1];
             
             received_pos.xPos = inbuffer[2];
             received_pos.zPos = inbuffer[3];
             received_pos.theta = inbuffer[4];
             
             setSpeed(0, 0, od);
             wb_robot_step(TIME_STEP);
             
             adjusting_angle = true;           
             
          }else if(inbuffer[0] == (float)FALSE_ALARM){
            setSpeed(0.1*MAX_SPEED, -0.1*MAX_SPEED, od);
            communicating = false;
            robot_found = false;
            adjusting_angle = false;
            
          }else if(inbuffer[0] == (float)GOT_IT){
            robot_found = true;
            
          }else if(inbuffer[0] == (float)CONTINUE){
            cout<<"Robot "<<readRobotId()<<" -> Continue with mapping" << endl;
            setSpeed(0.1*MAX_SPEED, -0.1*MAX_SPEED, od);
            communicating = false;
            firstMerge = false;

          }else if(inbuffer[0] == (float)SEND_MAP){
            sendMap(maps, (float)MAP_SENT);
            
            
          }else if(inbuffer[0] == (float)MAP_SENT){
            
            cout<<"Robot "<<readRobotId()<<" -> Received "<<inbuffer[1]<<" maps"<<endl;
            
            int data_sz = inbuffer[1] * (MAP_SZ * MAP_SZ + 2);
            cout<<"Robot "<<readRobotId()<<" -> Data size: "<<data_sz<<endl;
            
            int bufferIndex = 2;
            receivedMaps.clear();
            for(int i = 0; i < inbuffer[1]; i++){
                Map map;
                for(int row = 0; row < MAP_SZ; row++) {
                  for(int col = 0; col < MAP_SZ; col++) {
                      map.map.at<double>(row,col) = (double)inbuffer[bufferIndex++];
                  }
                }
                
                map.id.x = (int)inbuffer[bufferIndex];
                map.id.y = (int)inbuffer[++bufferIndex];
                bufferIndex++;
                receivedMaps.push_back(map);
            }
            
            mapAlert = true;
            
          }else if(inbuffer[0] == (float)SENDING_MERGED){
            cout<<"Robot "<<readRobotId()<<" -> Received "<<inbuffer[1]<<" maps"<<endl;
            
            int data_sz = inbuffer[1] * (MAP_SZ * MAP_SZ + 2);
            cout<<"Robot "<<readRobotId()<<" -> Data size: "<<data_sz<<endl;
            
            int bufferIndex = 2;
            maps.clear();
            for(int i = 0; i < inbuffer[1]; i++){
                Map map;
                for(int row = 0; row < MAP_SZ; row++) {
                  for(int col = 0; col < MAP_SZ; col++) {
                      map.map.at<double>(row,col) = (double)inbuffer[bufferIndex++];
                  }
                }
                
                map.id.x = (int)inbuffer[bufferIndex];
                map.id.y = (int)inbuffer[++bufferIndex];
                bufferIndex++;
                maps.push_back(map);
            }
 
            
          }else if(inbuffer[0] == (float)SEND_POS){
            //communicating = true;
            
            waitingStatus = true;

            cout<<"Robot "<<readRobotId()<<" -> Sending pos to the other robot: x = "<<pos->xPos<<"  z = "<<pos->zPos<<"  th = "<<pos->theta<<endl;
            emit({(float)POS_SENT, (float)pos->xPos, (float)pos->zPos,(float)pos->theta});
          
          }else if(inbuffer[0] == (float)POS_SENT){
            gotPos = true;
          //  communicating = true;
            
            received_pos.xPos = inbuffer[1];
            received_pos.zPos = inbuffer[2];
            received_pos.theta = inbuffer[3];
            cout<<"Robot "<<readRobotId()<<" -> Got other robot's position: x = "<<received_pos.xPos<<"  z = "<<received_pos.zPos<<"  th = "<<received_pos.theta<<endl;

          
          }else if(inbuffer[0] == (float)GO){
            waitingStatus = false;
            
          }else if(inbuffer[0] == (float)FINISHED){
            otherFinished = true;
            waitingStatus = false;
          }
          wb_receiver_next_packet(receiver);

        }
        wb_robot_step(TIME_STEP);
        return;
      }
    /*----------------------------------------------------------*/
    
     void sendMap(vector<Map> &maps, float message){
           vector<float> msg;
            /* sending the map -> 1st size of locals, then map data and local id for each map */
            msg.push_back(message);
            /* No of local maps */
            msg.push_back((float)maps.size());
            
            for(auto local:maps){
              for(int row = 0; row < MAP_SZ; ++row) {
                double* p = local.map.ptr<double>(row);
                for(int col = 0; col < MAP_SZ; ++col) {
                     msg.push_back((float)p[col]);
                    
                }
              }
              msg.push_back((float)local.id.x);
              msg.push_back((float)local.id.y);

            }
            cout<<"Robot "<<readRobotId()<<" -> Sending "<<msg[1]<<" maps with "<<msg.size()-2<<" elements"<<endl;
            emit(msg);
            
            return;
     }
     
    
   /*----------------------------------------------------------*/
     void setAdjustAngle(bool val){
       adjusting_angle = val;
       return;
     }
   /*----------------------------------------------------------*/

      bool commStatus(){
        return communicating;
      }
 
    /*----------------------------------------------------------*/
     
      bool adjustAngle(){
        return adjusting_angle;
      }

   /*----------------------------------------------------------*/
      
      void adjustAngleDone(){
        adjusting_angle = false;
        return;
      }
 
    /*----------------------------------------------------------*/
      
     Position getReceivedPos(){
       return received_pos;
     }
     
     /*----------------------------------------------------------*/
     
     float getReceivedDist(){
       return received_distance;
     }
     
     vector<Map> getReceivedMaps(){
       return receivedMaps;
     }
     
     
};