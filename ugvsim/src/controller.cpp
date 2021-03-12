#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<turtlesim/Pose.h>
#include<stdio.h>
#include<math.h>

class ugv
{
  public:
    struct xy
  {
    float x;
    float y;  
  };
  typedef struct xy xy;

  xy dest;
  ugv(float x, float y)
  {
    dest.x=x;
    dest.y=y;
  }

  float direct(xy inital, xy current)
  {
      printf("------------------------------\n");
      printf("Before transformation:\n");
      printf("Destination:\n");
      printf("      X= %f\n",dest.x);
      printf("      Y= %f\n",dest.y);
      printf("Initial:\n");
      printf("      X= %f\n",inital.x);
      printf("      Y= %f\n",inital.y);
      printf("Current:\n");
      printf("      X= %f\n",current.x);
      printf("      Y= %f\n",current.y);

      double temp_x=dest.x-inital.x;
      double temp_y=dest.y-inital.y;

      current.x=current.x-inital.x;
      current.y=current.y-inital.y;

      double x=atan2(temp_y,temp_x);
      double y=-current.x*sin(x)+current.y*cos(x);

      printf("After transformation:\n");
      printf("      X= %f\n",dest.x);
      printf("      Y= %f\n",dest.y);
      printf("Initial:\n");
      printf("      X= %f\n",inital.x);
      printf("      Y= %f\n",inital.y);
      printf("Current:\n");
      printf("      X= %f\n",current.x);
      printf("      Y= %f\n",current.y);
      printf("[+]Current Y= %lf \n",y);
      printf("[+]Destination: X= %lf  Y= %lf\n",(dest.x*cos(x)+dest.y*sin(x)), (-dest.x*sin(x)+dest.y*cos(x)));

      if (y<0)   return (rand()/pow(10,9));
      else   return (-rand()/pow(10,9));
  }

};

ros::Publisher ugvsim;
ros::Subscriber s;
geometry_msgs::Twist cmd; //    obj of the geometry_msgs/Twist message
turtlesim::Pose present; //    obj of the turtlesim/Pose message

double DEST_X;
double DEST_Y;
#define sample_dt 10 //   sample rate 10ms
double tolerate;

void move(float d, ugv::xy);
void rotate(float r);
void hault();
ugv::xy pose();
float distance(ugv::xy);
void print_data(ugv::xy,ugv::xy,float);
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ugvsim=n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
    s=n.subscribe("/turtle1/pose", 1, poseCallback);

    printf("[~]Enter Destination (x,y,tolerance):\n");
    scanf("%lf", &DEST_X);
    scanf("%lf", &DEST_Y);
    scanf("%lf",&tolerate);
    ugv algo(DEST_X,DEST_Y);
    ugv::xy initial, current;

    current.x=0.0;
    current.y=0.0;
    hault();
    do{
        initial.x=current.x;
        initial.y=current.y;
        current=pose();
        float speed=algo.direct(initial,current);

        print_data(initial,current,speed);
        rotate(speed);
        move(abs(speed),current);
        
    }while(distance(current)>tolerate);
    hault();

    ros::spin();
    return 0;
}

void poseCallback(const turtlesim::Pose::ConstPtr& pose_message)
{
    present.x=pose_message->x;
    present.y=pose_message->y;
    present.theta=pose_message->theta;
}

void move(float d, ugv::xy curren)
{
    cmd.linear.x=d;
    cmd.linear.y=0;
    cmd.linear.z=0;

    cmd.angular.x=0;
    cmd.angular.y=0;
    cmd.angular.z=0;

    //  To set the rate of the loop exection 
    printf("[+]Forward\n");
    ros::Rate loop_rate(sample_dt);
    double t0=ros::Time::now().toSec(); //    time counter
    double current_d=0;
    do
    {
        ugvsim.publish(cmd);
        double t1=ros::Time::now().toSec();
        current_d=d*(t1-t0);
        loop_rate.sleep();
        ros::spinOnce();
        if(distance(curren)<tolerate) break;
    } while (current_d<abs(d)); 
}

void rotate(float r)
{
    cmd.linear.x=0;
    cmd.linear.y=0;
    cmd.linear.z=0;

    cmd.angular.x=0;
    cmd.angular.y=0;
    cmd.angular.z=r;

     //  To set the rate of the loop exection 
    printf("[+]Rotate\n");
    ros::Rate loop_rate(sample_dt);
    double t0=ros::Time::now().toSec(); //    time counter
    double current_r=0;
    do
    {
       ugvsim.publish(cmd);
        double t1=ros::Time::now().toSec();
        current_r=abs(r)*(t1-t0);
        loop_rate.sleep();
        ros::spinOnce();
    } while (current_r<abs(r)); 
}

void hault()
{
    //  initialising with default values
    printf("Hault\n");
    cmd.linear.x=0.0;
    cmd.linear.y=0.0;
    cmd.linear.z=0.0;
    cmd.angular.x=0.0;
    cmd.angular.y=0.0;
    cmd.angular.z=0.0;
}

ugv::xy pose()
{
    ugv::xy pos;

    printf("------------------\n");
    printf("[+]Position:\n");
    printf("        X=%f\n",present.x);
    printf("        Y=%f\n",present.y);
    printf("        Theta=%f\n",present.theta);
    pos.x=present.x;
    pos.y=present.y;

    return pos;
}

float distance (ugv::xy present)
{
    present.x=present.x-DEST_X;
    present.y=present.y-DEST_Y;
    float d=sqrt(present.x*present.x+present.y*present.y);
    return d;
}

void print_data(ugv::xy ini, ugv::xy curr, float s)
{
    printf("------------------\n");
    printf("[+]Destination\n");
    printf("        X=%f\n",DEST_X);
    printf("        Y=%f\n",DEST_Y);
    printf("[+]initial\n");
    printf("        X=%f\n",ini.x);
    printf("        Y=%f\n",ini.y);
    printf("[+]Current\n");
    printf("        X=%f\n",curr.x);
    printf("        Y=%f\n",curr.y);
    printf("[+]Distance = %f\t tolerance = %f\n",distance(curr),tolerate);
    printf("[+]Speed(in MKS) = %f\n",s);
}