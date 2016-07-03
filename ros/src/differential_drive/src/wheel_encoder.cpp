#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <cmath>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <pthread.h>

using namespace std;

#define MEAN_NB 7 // On how many ticks are we doing a meaning.

struct count_t {
	int fdA;
	int fdB;
	long int cur_ticks;
	long int fwd_ticks;
	long int bwd_ticks;
};


void* encoderCount(void *arg) {
    char buffer[2];
	int inc;
    fd_set fds;
	count_t* count = (count_t*) arg;

    while (1) {
		int fwd=0;
		int bwd=0;
		for (int i=0;i<MEAN_NB;i++) {
			FD_ZERO(& fds);
			FD_SET(count->fdA, & fds);
			if (select(count->fdA+1, NULL, NULL, & fds, NULL) < 0) {
				perror("select");
				break;
			}
			if (read(count->fdB, & buffer, 2) != 2) {
				perror("read");
				break;
			}
			lseek(count->fdB, 0, SEEK_SET);
			if (buffer[0]=='0') fwd++;
			else                bwd++;

			if (read(count->fdA, & buffer, 2) != 2) {
				perror("read");
				break;
			}
			lseek(count->fdA, 0, SEEK_SET);
		}
		if (fwd>bwd) {
			count->cur_ticks += MEAN_NB;
			count->fwd_ticks += MEAN_NB;
		} else {
			count->cur_ticks -= MEAN_NB;
			count->bwd_ticks += MEAN_NB;
		}
    }
    return NULL;
}
void initGpio(string gpio,bool edge) {
	int fd;
	if ((fd = open("/sys/class/gpio/export", O_WRONLY)) < 0) {
		exit(EXIT_FAILURE);
	}
	write(fd,gpio.c_str(),strlen(gpio.c_str()));
	close(fd);
	string fileName = "/sys/class/gpio/gpio"+gpio+"/direction";
	if ((fd = open(fileName.c_str(), O_WRONLY)) < 0) {
		exit(EXIT_FAILURE);
	}
	write(fd,"in",2);
	close(fd);
	
	fileName = "/sys/class/gpio/gpio"+gpio+"/edge";
	if ((fd = open(fileName.c_str(), O_WRONLY)) < 0) {
		exit(EXIT_FAILURE);
	}
	if (edge)  write(fd,"rising",6);
	else       write(fd,"none",4);
	close(fd);
}

void closeGpio(string gpio) {
	int fd;
	if ((fd = open("/sys/class/gpio/unexport", O_WRONLY)) < 0) {
		exit(EXIT_FAILURE);
	}
	write(fd,gpio.c_str(),strlen(gpio.c_str()));
	close(fd);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "wheel_encoder");

	ros::NodeHandle nh;
	string nodeName = ros::this_node::getName();

	// ROS parameters
	int rate,tpr;
	string pinNameA,pinNameB;
	double radius;
	ros::param::param("~rate"              ,rate    ,20 );
	ros::param::param("~ticks_per_rotation",tpr     ,0  );
	ros::param::param("~wheel_radius"      ,radius  ,0.0);
	ros::param::param<string>("~pinNameA",pinNameA, "");
	ros::param::param<string>("~pinNameB",pinNameB, "");
	if (pinNameA=="" || pinNameB=="" || radius==0.0 || tpr==0) {
		ROS_FATAL("Parameters pinNameA, pinNameB, wheel_radius and ticks_per_rotation are required. Got %s %s %f %d"
		,	pinNameA.c_str(), pinNameB.c_str(), radius, tpr
		);
		exit(EXIT_FAILURE);
	}
	if (pinNameA!="P9_12" && pinNameA!="P9_15") {
		ROS_FATAL("Only P9_12 or P9_15 are supported for pinNameA");
		exit(EXIT_FAILURE);
	}
	if (pinNameB!="P9_11" && pinNameB!="P9_13") {
		ROS_FATAL("Only P9_11 or P9_13 are supported for pinNameB");
		exit(EXIT_FAILURE);
	}
	
	string gpioA = pinNameA == "P9_12" ? "60" : "48";
	string gpioB = pinNameB == "P9_11" ? "30" : "31";
	double m_per_tick = 2*M_PI*radius/tpr;

	// BeagleBone pin setup
	count_t* count = new count_t();
	initGpio(gpioA,1);
	initGpio(gpioB,0);
	string valueA = "/sys/class/gpio/gpio"+gpioA+"/value";
	string valueB = "/sys/class/gpio/gpio"+gpioB+"/value";
    count->fdA = open(valueA.c_str(), O_RDONLY);
    count->fdB = open(valueB.c_str(), O_RDONLY);
	count->cur_ticks = 0;
	count->bwd_ticks = 0;

	pthread_t tid;
	pthread_create(&tid, NULL, &encoderCount, (void*)count);

	// ROS topics
	ros::Publisher pub_vel = nh.advertise<std_msgs::Float64>("wheel_vel", 1);
	ros::Publisher pub_dst = nh.advertise<std_msgs::Float64>("wheel_dst", 10);

	ROS_INFO("%s started",nodeName.c_str());

	// spin
	ros::Rate rosRate(rate);
	ros::Time last_time = ros::Time::now();
	ros::Time cur_time;
	std_msgs::Float64 vel;
	std_msgs::Float64 dst;

	int nLoop=0;
	while (ros::ok()) {
		cur_time = ros::Time::now();
		double ticks_per_s = count->cur_ticks/(cur_time-last_time).toSec();
		vel.data = ticks_per_s*m_per_tick;
		pub_vel.publish(vel);
		dst.data = count->cur_ticks*m_per_tick;
		pub_dst.publish(dst);
		last_time = cur_time;
		count->cur_ticks = 0;
		// Debug, keep total counts
		nLoop++;
		if (nLoop==20) {
			ROS_DEBUG("%s : Fwd %ld , Bwd %ld",nodeName.c_str(),count->fwd_ticks,count->bwd_ticks);
			nLoop=0;
		}
		rosRate.sleep();
	}
	closeGpio(gpioA);
	closeGpio(gpioB);
}
