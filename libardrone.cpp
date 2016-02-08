#include "include/libardrone.hpp"

using namespace robot;

void run() 
{
    MessageClient msgClient;
    
    msgClient.announce("robot/go_next_destination");
    msgClient.publish("robot/go_next_destination", "true");
    
    
    


}
