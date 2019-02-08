/************************************************************
 * Name: alpha_pkg_node.cpp
 
 * Author:  Shashank Shastry 	scshastr@eng.ucsd.edu
 		   	    Sumukha Harish 		ssumukha@eng.ucsd.edu
 		   	    Sai Adithya			  schittur@eng.ucsd.edu
 
 * Date: 02/06/2019
 
 * Description: This is the source file of the node in which 
 				the robot looks around to find a pink target
 				and approaches it. The code also takes care of 
 				obstacle avoidance when the obstacle is detected
 				by the depth camera or the bumper sensor.

 				Topics subscribed to:
 				1. "/blobs" 
 				2. "/camera/depth/points"
 				3. "/mobile_base/events/bumper"

 				Topics published:
 				1. "cmd_vel_mux/input/teleop"

* Usage: 		roscore
				roslaunch turtlebot_bringup minimal.launch
				roslaunch astra_launch astra_pro.launch

				rosparam set /cmvision/color_file <path/to/colors.txt/file>
				rosparam set /cmvision/mean_shift_on false
				rosparam set /cmvision/debug_on false
				rosparam set /cmvision/spatial_radius_pix 0
				rosparam set /cmvision/color_radius_pix 0
				rosrun cmvision cmvision image:=/camera/rgb/image_raw

				rosrun alpha_pkg alpha_pkg_node
 ************************************************************/

#include <kobuki_msgs/BumperEvent.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <stdio.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
#include <ros/console.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

uint16_t state = 0;
bool goal_found_flag = false;
bool obstacle_found_flag = false;
bool bumper_flag = false;
uint16_t goal_blob_area = 0;
float goal_x = 0;
float image_height = 480, image_width = 640;
float linear_speed =0.15, angular_speed = 0.7, angular_speed_thresh = 0.3;

/************************************************************
 * Function Name: blobsCallBack

 * Description: This is the callback function of the /blobs topic.
 				The callback function also computes the centroid 
 				of the blobs detected based on weighted average.
 				It also raises the goal_found_flag to true when the
 				goal is seen and false otherwise.
 ***********************************************************/

void blobsCallBack (const cmvision::Blobs& blobsIn) 
{
	/************************************************************
	* These blobsIn.blobs[i].red, blobsIn.blobs[i].green, and blobsIn.blobs[i].blue values depend on the
	* values those are provided in the colors.txt file.
	* For example, the color file is like:
	* 
	* [Colors]
	* (255, 0, 0) 0.000000 10 RED 
	* (255, 255, 0) 0.000000 10 YELLOW 
	* [Thresholds]
	* ( 127:187, 142:161, 175:197 )
	* ( 47:99, 96:118, 162:175 )
	* 
	* Now, if a red blob is found, then the blobsIn.blobs[i].red will be 255, and the others will be 0.
	* Similarly, for yellow blob, blobsIn.blobs[i].red and blobsIn.blobs[i].green will be 255, and blobsIn.blobs[i].blue will be 0.
	************************************************************/

  	if (blobsIn.blob_count > 0){
    	int num_goal_blobs = 0;		
	    float goal_sum_x=0, goal_sum_y=0;
	    goal_blob_area = 0;

	    int colors[2][3] = {{238, 114, 76},   	// Pink indoors
	                        {185, 66, 36}};  	// Pink outdoors

	    // Cumulate x, y, areas and number of blobs
		for (int i = 0; i < blobsIn.blob_count; i++){	  
	  		int c = 1; 
	  		if (blobsIn.blobs[i].red == colors[c][0] && blobsIn.blobs[i].green == colors[c][1] && blobsIn.blobs[i].blue == colors[c][2]){
			    int area = blobsIn.blobs[i].area;
			    goal_sum_x += area*blobsIn.blobs[i].x; //stores the average x-coordinate of the goal blobs
			    goal_sum_y += area*blobsIn.blobs[i].y; //stores the average y-coordinate of the goal blobs
			    goal_blob_area += area;
			    num_goal_blobs++;
	  		}
		}	

	    // std::cout << goal_blob_area << std::endl;

		// Raise goal_found_flag to true if goal_blob_area > threshold (3000)
	    if(goal_blob_area>3000){
		    goal_x = goal_sum_x/goal_blob_area;
		    goal_x-=image_width/2;
	      	if(!goal_found_flag){
	        	goal_found_flag=true;
	      	}
	    }
	    else{
	      	goal_found_flag=false;
	    }
  	}
}

/************************************************************
 * Function Name: PointCloud_Callback

 * Description: This is the callback function of the topic
 				"/camera/depth/points". The function also computes
 				the number of points that are closer than a threshold
 				z_min and raises the obstacle_found_flag if the
 				number of points are greater than a threshold (10)
*************************************************************/

void PointCloud_Callback (const PointCloud::ConstPtr& cloud){
	double min_z = 0.7;
	int y_point = 0;
	std::vector<double> PCL_closest_points;

  	// Iterate through all the points in the image and populate buffer
  	// if the z coordinate is lesser than threshold (min_z)
  	for(int k = 0; k < 240; k++){
    	for(int i = 0; i < 640; i++){
      		const pcl::PointXYZ & pt=cloud->points[640*(180+k)+(i)];
      		if((pt.z < min_z)){
        		PCL_closest_points.push_back(i); 
      		}
    	}
  	}	
	  
	// Raise obstacle_found_flag if the size of the buffer is greater than
	// threshold 10
	if(PCL_closest_points.size() > 10){
		obstacle_found_flag = true;
	}
  	else{
    	if(!bumper_flag){
      		obstacle_found_flag = false;
    	} 
  	}
}

/************************************************************
 * Function Name: Bumper_Callback

 * Description: This is the callback function of the topic
 				"/mobile_base/events/bumper". The function raises 
 				the obstacle_found_flag and bumper_flag if bumper 
 				press is detected.
*************************************************************/

void Bumper_Callback (const kobuki_msgs::BumperEvent::ConstPtr& bumper_msg){
	// Detect bumper press and raise flag
	if(bumper_msg->state == 1){
    	bumper_flag=true;
    	obstacle_found_flag = true;
    }
   	else{
    	bumper_flag=false;
  	}
}

/************************************************************
 * Function Name: rotate

 * Description: Generic function which makes the robot rotate
 				about its z axis at constant angular velocity
*************************************************************/

void rotate(ros::Publisher velocityPublisher){
	geometry_msgs::Twist T;
  	T.linear.x = 0.0; T.linear.y = 0.0; T.linear.z = 0.0;
  	T.angular.x = 0.0; T.angular.y = 0.0; T.angular.z = angular_speed;
  	velocityPublisher.publish(T);
}

/************************************************************
 * Function Name: seek

 * Description: Function to make the robot approach the target.
 				Control based on generic P control. 
*************************************************************/

void seek(ros::Publisher velocityPublisher){
  	float angular_control = -goal_x*angular_speed*0.7;

  	// Limit angular control to within angular_speed_thresh
  	if(std::abs(angular_control) > angular_speed_thresh){
    	angular_control = angular_control*angular_speed_thresh / std::abs(angular_control);
  	}

  	// Publish twist message
  	geometry_msgs::Twist T;
  	T.linear.x = linear_speed*0.7; T.linear.y = 0.0; T.linear.z = 0.0;
  	T.angular.x = 0.0; T.angular.y = 0.0; T.angular.z = angular_control;
  	velocityPublisher.publish(T);
}

/************************************************************
 * Function Name: advance

 * Description: Generic function which makes the robot 
 				move forward with constant linear velocity.
*************************************************************/

void advance(ros::Publisher velocityPublisher){
  geometry_msgs::Twist T;
  T.linear.x = linear_speed; T.linear.y = 0.0; T.linear.z = 0.0;
  T.angular.x = 0.0; T.angular.y = 0.0; T.angular.z = 0.0;
  velocityPublisher.publish(T);
}

/************************************************************
 * Function Name: retreat

 * Description: Generic function which makes the robot 
 				move backward with constant linear velocity.
*************************************************************/
void retreat(ros::Publisher velocityPublisher){
  	geometry_msgs::Twist T;
  	T.linear.x = -linear_speed; T.linear.y = 0.0; T.linear.z = 0.0;
  	T.angular.x = 0.0; T.angular.y = 0.0; T.angular.z = 0.0;
  	velocityPublisher.publish(T);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "blob");

  ros::NodeHandle nh;
  ros::Publisher velocityPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
  ros::Subscriber PCSubscriber = nh.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);
  ros::Subscriber BumperSubscriber = nh.subscribe<kobuki_msgs::BumperEvent>("/mobile_base/events/bumper", 1, Bumper_Callback);
  ros::Subscriber blobsSubscriber = nh.subscribe("/blobs", 50, blobsCallBack);

  ros::Rate loop_rate(10);

  //States variable initialized to 0
  state = 0;

  while(ros::ok()){

    std::cout<<"state: "<< state << " obstacle found: " << obstacle_found_flag <<  std::endl;

    switch(state){
      	// Functionalities of state 0
    	case 0:{
	      	// If obstacle detected switch to state 2
	        if(obstacle_found_flag){
	        	state = 2;
	          	break;
	        }

	        // If target detected switch to state 1
	        if(goal_found_flag){
	          	state=1;
	          	break;
	        }
	        
	        // Else rotate in state 0
	        rotate(velocityPublisher);
	        break;
	      }

	    // Functionalities of state 1
      	case 1:{
	      	// If obstacle detected switch to state 2	
	        if(obstacle_found_flag){
	        	state=2;	
	          	break;
	        }

	        // If target lost switch to state 0
	        if(!goal_found_flag){
	        	state=0;
	        	break;
	        }

	        // Else seek in state 1
	        seek(velocityPublisher);
	        break;
	      }

	    // Functionalities of state 2
	    case 2:{
	      	// If detected obstacle is target switch to state 3
	        if(goal_blob_area > (image_width*image_height*0.1)){
	        	state=3;
	          	break;
	        }
	        
	        // If obstacle detected is via bumper then execute retreat,
	        // rotate, advance and revert to state 0
	        if(bumper_flag){
	        	for(int i=0;i<40000;i++){
	            	retreat(velocityPublisher);
	          	}
	          	for(int i=0;i<40000;i++){
	            	rotate(velocityPublisher);
	         	}
	          	for(int i=0;i<40000;i++){
	            	advance(velocityPublisher);
	          	}
	          	state=0;
	          	break;
	        }

	        // If obstacle detected is via depth then rotate and
	        // advance and revert to state 0
	        if(obstacle_found_flag){
	        	rotate(velocityPublisher);
	        }
	        else{
	         	for(int i=0;i<100000;i++){
	            	advance(velocityPublisher);
	          	}
	          	state = 0;
	        }
	        break;
	      }

	    // Functionalities of state 3 
	    case 3:{
	    	// Stay in state 3 forever
        	state=3;
        	break;
      	}
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}