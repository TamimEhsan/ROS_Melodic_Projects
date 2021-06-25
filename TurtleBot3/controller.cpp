#include <bits/stdc++.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
using namespace std;


class Position{

    long double x;
    long double y;
    long double theta;

    const long double PRECISION = 0.1;

public:
    Position(){
        x = 0.0;
        y = 0.0;
        theta = 0.0;
    }
    Position(long double _x,long double _y){
        x = _x;
        y = _y;
    }
    Position(long double _x,long double _y,long double _theta){
        x = _x;
        y = _y;
        theta = _theta;
    }

    bool isCloseTo(Position anotherPosition){
       if(  abs(x-anotherPosition.x)<PRECISION and abs(y-anotherPosition.y)<PRECISION ) return true;
       else return false;
    }

    void setAll(const nav_msgs::Odometry::ConstPtr &msg){
        setX( msg->pose.pose.position.x );
        setY( msg->pose.pose.position.y );
        setTheta( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
    }

    void setTheta(long double qx,long double qy,long double qz,long double qw){
        tf::Quaternion q(qx,qy,qz,qw);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta = yaw;
    }
    void setX(long double _x){
        x = _x;
    }
    void setY(long double _y){
        y = _y;
    }
    long double getTheta(){
        return theta;
    }
    long double getX(){
        return x;
    }
    long double getY(){
        return y;
    }
};



class Controller{

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    Position target;
    Position current;

    void callback(const nav_msgs::Odometry::ConstPtr &msg){
        current.setAll(msg);
    }

    void sendMoveCommand(long double x,long double z){
        geometry_msgs::Twist pubmsg;
        pubmsg.linear.x = x;
        pubmsg.angular.z = z;
        pub.publish(pubmsg);
    }

public:

    Controller(){
        sub = n.subscribe("/odom",1000,&Controller::callback,this);
        pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    }


    bool isInTarget(){
        return current.isCloseTo(target);
    }

    void setTarget(long double x,long double y){
        target.setX(x);
        target.setY(y);
    }


    void nextMove(){
        long double incX = target.getX() - current.getX();
        long double incY = target.getY() - current.getY();
        long double angle = atan2(incY,incX);

        if( (angle - current.getTheta()) >0.3 ){
            sendMoveCommand(0,0.3);
        }else if( (angle - current.getTheta()) <-0.3 ){
            sendMoveCommand(0,-0.3);
        }else if( (angle - current.getTheta()) >0.1 ){
            sendMoveCommand(0,0.1);
        }else if( (angle - current.getTheta()) <-0.1 ){
            sendMoveCommand(0,-0.1);
        }else{
            sendMoveCommand(0.5,0);
        }
    }

    void stop(){
        sendMoveCommand(0,0);
    }

};


int main(int argc,char **argv){

    ros::init(argc,argv,"controller");
    Controller turtle;
    ros::Rate loop_rate(10);

    int x,y;

    cout<<"input the target coordinate (x y)"<<endl;

    cin>>x>>y;

    turtle.setTarget(x,y);
    ros::spinOnce();
    while(ros::ok()){
        if(  turtle.isInTarget() ) {
            cout<<"You have arrived at your destination!"<<endl;
            turtle.stop();
            ros::spinOnce();
            loop_rate.sleep();
            break;
        }
        turtle.nextMove();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
