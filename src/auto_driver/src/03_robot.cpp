#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include <math.h>   /* M_PI */

#include <ros/ros.h>
#include <std_msgs/Int16.h>

using namespace std;

#define bbox_x1 -10
#define bbox_x2 10
#define bbox_y1 -10
#define bbox_y2 10
#define robot_length 0.15 /* [m] */
#define max_speed 500
#define min_speed 40
#define max_steering 180
#define min_steering 0

struct ControlU {
  double v;
  double gamma;
};

class Robot {
public:
  Robot(): speed_pub(NULL), steering_pub(NULL) {
    srand(time(NULL)); /* initialize random seed, if this is done outside the class, the lines below could be replace by a constructor deferal  */
    this->Set((rand()/RAND_MAX)*(bbox_x2 -bbox_x1) + bbox_x1,
	      (rand()/RAND_MAX)*(bbox_y2 -bbox_y1) + bbox_y1,
	      (rand()/RAND_MAX)*(M_PI));
    this->SetName("AutoNOMOs Simulation");
    length = robot_length;
    this->SetControlLaw(ControlMoveToPoint);
  }

  Robot(double x, double y, double theta): Robot(x,y,theta, "AutoNOMOs Simulation") {
  }

  Robot(double x, double y, double theta, string name, ros::Publisher *p_speed_pub=NULL, ros::Publisher *p_steering_pub=NULL): speed_pub(p_speed_pub), steering_pub(p_steering_pub) {
    this->Set(x,y,theta);
    this->SetName(name);
    length = robot_length;
    this->SetControlLaw(ControlMoveToPoint);
  }

  void SetControlLaw(string _name){
    if (_name == "MoveToPoint")
      this->SetControlLaw(ControlMoveToPoint);
    else if (_name == "FollowPath")
      this->SetControlLaw(ControlFollowPath);
    else if (_name == "MoveToPose")
      this->SetControlLaw(ControlMoveToPose);
    else if (_name == "FollowLine")
      this->SetControlLaw(ControlFollowLine);
    else // default
      this->SetControlLaw(ControlMoveToPoint);
  }

  void SetControlLaw(bool (*_controller)(double&, double&, double&, double, double, double, double, double, double, double, double, double, double), double _k_1=0.5, double _k_2 = 0.1, double _k_3=0.2, double _epsilon = 0.2) {
    controller_f = _controller;
    k_1 = _k_1;
    k_2 = _k_2;
    k_3 = _k_3;
    epsilon = _epsilon;
    error_prev = 0;
  }

  void SetGoal(double _x_goal, double _y_goal, double _theta_goal) {
  x_goal = _x_goal;
  y_goal = _y_goal;
  theta_goal = _theta_goal;
  }

  bool ComputeControls() {
    double v_goal, gamma_goal;
    bool achieved_goal = (*controller_f)(v_goal, gamma_goal, error_prev, x, y, theta, x_goal, y_goal, theta_goal, k_1, k_2, k_3, epsilon);
    this->MapControls(v_goal, gamma_goal);
	ROS_INFO_STREAM("La velocidad goal "<< v_goal <<" ,gamma goal: "<< gamma_goal);
    return achieved_goal;
  }

  void JumpTo(double x, double y, double theta) {
    this->Set(x,y,theta);
  }

  void MapControls(double v, double gamma) {
		this->v=v;
		this->gamma=gamma;
  ROS_INFO_STREAM("Gamma  "<< this->gamma);
	this->speed=v*-1000;

		if(this->speed<-1000){
			this->speed=-1000;
		}

		if(this->speed>1000){
			this->speed=1000;
		}

		this->steering=gamma;
    ROS_INFO_STREAM("steering: "<< steering);

    if(this->steering < (-M_PI)){
      steering=(-M_PI);
    }

    if(this->steering>M_PI){
      steering=M_PI;
    }

    this->steering=(90*8*gamma)/M_PI + 90;


		ROS_INFO_STREAM("Calculated velocity  "<< speed <<", Calculated steering: "<< steering);
  }

  void PublishControls() {
    std_msgs::Int16 to_publish;
	ROS_INFO_STREAM("Published velocity "<< speed <<" , Published steering : "<< steering);
    if (speed_pub != NULL) {
      to_publish.data = speed;
      speed_pub->publish(to_publish);
    }
    if (steering_pub != NULL) {
      to_publish.data = steering;
      steering_pub->publish(to_publish);
   }
  }

  static bool ControlMoveToPoint(double &v_goal, double &gamma_goal, double &error_prev,
				 double x, double y, double theta,
				 double x_goal, double y_goal, double theta_goal,
				 double k_v=3, double k_i=0.1, double k_h=0.5,
				 double epsilon = 3) {
    ROS_INFO_STREAM("ControlMoveToPoint");

  double v;
  double theta_star;
  double distance=sqrt(pow((x_goal-x),2.0)+ pow((y_goal-y),2.0));

  if(distance < epsilon){
    return true;
} else {

		v_goal = k_v*(distance);
		theta_star = atan2((y_goal-y),(x_goal-x));
		gamma_goal = k_h*(theta_star-theta);
    if((tan(gamma_goal/k_h))>M_PI){
      v_goal=-1*v_goal;
    }
		return false;
		}
  }

  static bool ControlFollowPath(double &v_goal, double &gamma_goal, double &error_prev, double x, double y, double theta, double x_goal, double y_goal, double theta_goal, double k_v=0.5, double k_i=0.05, double k_h=0.3, double d_goal = 2) {
    ROS_INFO_STREAM("ControlFollowPath");

  double v;
  double theta_star;
 double distance=sqrt(pow((x_goal-x),2.0)+ pow((y_goal-y),2.0));

  if(distance < d_goal){
	   error_prev=0;
    return true;

} else {

		v_goal=k_v*((distance)-d_goal)+(k_i*error_prev);
		theta_star=atan2((y_goal-y),(x_goal-x));
		gamma_goal=k_h*(theta_star-theta);
		error_prev=error_prev+distance-d_goal;
    if((tan(gamma_goal/k_h))>M_PI){
      v_goal=-1*v_goal;
    }
		return false;
		}
  }

  static bool ControlMoveToPose(double &v_goal, double &gamma_goal, double &error_prev, double x, double y, double theta, double x_goal, double y_goal, double theta_goal, double k_rho=0.5, double k_alpha=0.1, double k_beta=0.2, double epsilon = 3) {
    ROS_INFO_STREAM("ControlMoveToPose");
  
	double rho=sqrt(pow((x_goal-x),2.0)+ pow((y_goal-y),2.0));
	double alpha,beta;
	if(rho<=epsilon){
		return true;
}else{
	v_goal=k_rho*rho;
	alpha=atan2((y_goal-y),(x_goal-x))-theta;
	beta=-theta-alpha;
	gamma_goal=k_alpha*alpha+k_beta*beta;
	return false;

}
  }

    static bool ControlFollowLine(double &v_goal, double &gamma_goal, double &error_prev, double x, double y, double theta, double a_goal, double b_goal, double c_goal, double k_d=0.5, double k_v=0.1, double k_h=0.2, double epsilon = 3) {
    ROS_INFO_STREAM("ControlFollowLine");
	double distance,alpha,alpha2,theta_star;
	distance=(a_goal*x+b_goal*y+c_goal)/sqrt(pow((a_goal),2.0)+ pow((b_goal),2.0));
	if(distance<epsilon){
		return true;
}else{

	alpha=-k_d*distance;
	theta_star=atan2(-a_goal,b_goal);
	alpha2=k_h*(theta_star-theta);
	gamma_goal=alpha+alpha2;
	v_goal=k_v*100;
	return false;
}
  }

void SetVGamma(double v, double gamma) {
    this->v = v;
    this->gamma = gamma;
  }


  ControlU GetControllers() {
    ControlU u;
    u.v = v;
    u.gamma = gamma;
    return u;
  }

  void UpdateStatePose(double delta_t) {
    x = x + x_dot*delta_t;
    y = y + y_dot*delta_t;
    theta = fmod(theta + theta_dot*delta_t,2*M_PI);
    theta = (theta < -M_PI) ? theta + 2*M_PI : theta;
    theta = (theta > M_PI) ? theta - 2*M_PI : theta;
  }

  void UpdateStateVelocity(double x_dot, double y_dot, double theta_dot) {
    this->x_dot = x_dot;
    this->y_dot = y_dot;
    this->theta_dot = theta_dot;
  }

  // Operators
  friend std::ostream& operator << (std::ostream &out, const Robot &robot){
    out.precision(2); // sets decimal precision to 2 significant digits
    out << "\n\t" << robot.name << ": ";
    out << "Pose(" << robot.x << "," << robot.y << "," << robot.theta << ") ";
    out << "Goal(" << robot.x_goal << "," << robot.y_goal << "," << robot.theta_goal << ") ";
    out << "Control(" << robot.v  << "," << robot.gamma << ") ";

    return out;
  }

private:

  string name;
  double length;

  ros::Publisher *speed_pub, *steering_pub;

  double x, y, theta;                // Robot Pose
  double x_dot, y_dot, theta_dot;    // Robot velocity
  double x_goal, y_goal, theta_goal; // Goal to achieve
  double v, gamma;     // Controls to achieve goal (in metrics units)
  int speed, steering; // Controls to achieve goal (in robot units)

  double k_1, k_2, k_3; // Constants for controllers
  double epsilon; // Tolerance for controller error
  double error_prev; // Error used in integral and differential controls

  bool (*controller_f)(double&, double&, double&, double, double, double, double, double, double, double, double, double, double); // Pointer to control function

  void Set(double x, double y, double theta, double x_dot = 0, double y_dot = 0, double theta_dot = 0) {
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->x_dot = x_dot;
    this->y_dot = y_dot;
    this->theta_dot = theta_dot;
  }

  void SetName(string new_name) {
    this->name = new_name;
  }

};
#endif //ROBOT_H
