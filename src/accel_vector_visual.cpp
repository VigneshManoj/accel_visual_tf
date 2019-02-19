#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>

float weighted_avg[3]={0.5,0.3,0.2};
int j=0;
float array_accel_x[3],array_vel_x[3],array_accel_y[3],array_vel_y[3],array_accel_z[3],array_vel_z[3];
std::string wrenchheader="psm1_tool_pitch_link";
//std::string wrenchstamp="psm1_main_insertion_joint";
struct vel_val
{
    float vel_x;
    float vel_y;
    float vel_z;
}var1;
struct accel_val
{
    float accel_x;
    float accel_y;
    float accel_z;
}var2;

vel_val velocity_func(geometry_msgs::Twist twist)
{
    float vx,vy,vz;
    vx=twist.linear.x;
    vy=twist.linear.y;
    vz=twist.linear.z;
    printf("\nThe velocity in x direction is %f\n",vx);
    printf("\nThe velocity in y direction is %f\n",vy);
    printf("\nThe velocity in z direction is %f\n",vz);
    vel_val val_v={vx,vy,vz};
    return val_v;
}

accel_val accel_func(float vel1[2],float vel2[2],float vel3[2], float array_accel_x[2],float array_accel_y[2],float array_accel_z[2])
{
    float ax,ay,az;
    int t2=2,t1=1;
    ax=weighted_avg[0]*((vel1[1]-vel1[0])/(t2-t1)) + (weighted_avg[1]*array_accel_x[0]) + (weighted_avg[2]*array_accel_x[1]);
    ay=weighted_avg[0]*((vel2[1]-vel2[0])/(t2-t1)) + (weighted_avg[1]*array_accel_y[0]) + (weighted_avg[2]*array_accel_y[1]);
    az=weighted_avg[0]*((vel3[1]-vel3[0])/(t2-t1)) + (weighted_avg[1]*array_accel_z[0]) + (weighted_avg[2]*array_accel_z[1]);
    printf("\nThe Acceleration in x direction is %f\n",ax);
    printf("\nThe Acceleration in y direction is %f\n",ay);
    printf("\nThe Acceleration in z direction is %f\n",az);
    accel_val val_a={ax,ay,az};
    return val_a;
}

void initialise_func()
{
    for(int i=0;i<3;i++)
    {
        array_accel_x[i]=0;
        array_accel_y[i]=0;
        array_accel_z[i]=0;
        array_vel_x[i]=0;
        array_vel_y[i]=0;
        array_vel_z[i]=0;
    }
}

int main(int argc, char** argv)
{
    //ros::init(argc, argv, "my_tf_listener");
    ros::init(argc, argv, "accel_vector_visual");
    ros::NodeHandle node;
    tf::TransformListener listener;
    ros::Publisher chatter_pub = node.advertise<geometry_msgs::WrenchStamped>("/wrench_visualization/wrench", 1);
    ros::Rate rate(10);
    initialise_func();
    /*
    float array_accel_x[3]={0,0,0};
    float array_vel_x[3]={0,0,0};
    float array_accel_y[3]={0,0,0};
    float array_vel_y[3]={0,0,0};
    float array_accel_z[3]={0,0,0};
    float array_vel_z[3]={0,0,0};
    */
    int i=0,k=0;
    vel_val vel_v;
    accel_val accel_v;

    while(node.ok())
    {
        //tf::StampedTransform transform;
        geometry_msgs::Twist twist;
        geometry_msgs::WrenchStamped body_wrench;
        try
        {
              //listener.lookupTransform("base_link","ecm_base_link",ros::Time(0),transform);
              //x1=transform.getOrigin().x();
              //printf("%f\n",x1);

              //Calculation of Velocity
              listener.lookupTwist("psm1_main_insertion_link","psm1_tool_pitch_link",ros::Time(0),ros::Duration(0.1),twist);
              vel_v=velocity_func(twist);
              array_vel_x[i]=vel_v.vel_x;
              array_vel_y[i]=vel_v.vel_y;
              array_vel_z[i]=vel_v.vel_z;
              if(i%2==0)
              {
                  i=0;
              }
              else
              {
                  i=i+1;
              }

              //To ensure acceleration values run only after array has 3 values (at starting it will not be having 3 values)
              j=j+1;

              //Calculation of Acceleration
              if(j>3)
              {
                  accel_v=accel_func(array_vel_x,array_vel_y,array_vel_z,array_accel_x,array_accel_y,array_accel_z);
                  j=4;
                  array_accel_x[k]=accel_v.accel_x;
                  array_accel_y[k]=accel_v.accel_y;
                  array_accel_z[k]=accel_v.accel_z;
                  if(k%2==0)
                  {
                      k=0;
                  }
                  else
                  {
                      k=k+1;
                  }
              }
              printf("\n**The Acceleration in x direction is %f\n",array_accel_x[k]);
              printf("\n**The Acceleration in y direction is %f\n",array_accel_y[k]);
              printf("\n**The Acceleration in z direction is %f\n",array_accel_z[k]);

              body_wrench = geometry_msgs::WrenchStamped();
              body_wrench.header.frame_id = wrenchheader;
              //body_wrench.header.stamp= wrenchstamp;
              body_wrench.wrench.force.x = array_accel_x[k];
              body_wrench.wrench.force.y = array_accel_y[k];
              body_wrench.wrench.force.z = array_accel_z[k];

              chatter_pub.publish(body_wrench);

        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
    return 0;
}
