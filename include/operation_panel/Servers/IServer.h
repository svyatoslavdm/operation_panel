#ifndef ISERVER_H
#define ISERVER_H

#include <ros/ros.h>

class IServer
{

 public:
     
    IServer();
    
    virtual void start()=0;
    
    virtual void stop()=0;

 private:

};

#endif // ISERVER_H
