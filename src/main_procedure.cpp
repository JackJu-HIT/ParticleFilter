/***
 * Jack Ju
 * HITWH606
 * particle filter main procedure
 ***/
#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <string>
#include "particle_filter.h"
#include "particle_data.h"
// for convenience
using std::string;
using std::vector;

int main(int argc, char **argv){
    // ROS节点初始化
    ros::init(argc, argv, "Particle_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/odom_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher Particle_info_pub = n.advertise<my_particle_filter::particle_data>("/particle_info", 10);

    // 设置循环的频率
    ros::Rate loop_rate(1);
    //ROS_INFO("The data of GPS already published!");

    /****输入： Map 数据。
     * 车辆传感器数据（包括速度，转向角，障碍物方位感知）
     * 输出： 车辆在Map上的坐标
     * ****/
    // Set up parameters here
    double delta_t = 0.1;  // Time elapsed between measurements [sec]
    double sensor_range = 50;  // Sensor range [m]

    // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_pos [3] = {0.3, 0.3, 0.01};
    // Landmark measurement uncertainty [x [m], y [m]]
    double sigma_landmark [2] = {0.3, 0.3};

    // Read map data
    Map map;
    if (!read_map_data("../data/map_data.txt", map)) {
        std::cout << "Error: Could not open map file" << std::endl;
        return -1;
    }
    // Create particle filter
    ParticleFilter pf;
  
    while (ros::ok())
    {
    /*读取测量信息*/
 if (!pf.initialized()) {
                        // Sense noisy position data from the simulator初始位置获取
                        double sense_x =1 ;
                        double sense_y = 1;
                        double sense_theta =1;
                        /**粒子滤波初始化需要一个初始的位置、
                         * 初始的航向以及测量误差，
                         * 根据这些参数，我们就可以建立N个粒子。
                         * */
                        pf.init(sense_x, sense_y, sense_theta, sigma_pos);
                    } else {
                        // Predict the vehicle's next state from previous
                        //   (noiseless control) data.
                        //上一时刻的速度和转玩速度
                        double previous_velocity = 1;
                        double previous_yawrate =1 ;

                        pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);//sigma_pos预测的噪声
                    }




                    /**需要输入汽车传感器测量的参数**/
                    // receive noisy observation data from the simulator
                    // sense_observations in JSON format
                    //   [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
                    vector<LandmarkObs> noisy_observations;
                    double sense_observations_x = 1;//j[1]["sense_observations_x"];
                    double sense_observations_y = 1;//j[1]["sense_observations_y"];
                     /****导入观测的值****/
                    for (int i = 0; i < 1;i++){//x_sense.size(); ++i) {
                        LandmarkObs obs;
                        obs.x =1; //x_sense[i];
                        obs.y =1;// y_sense[i];
                        noisy_observations.push_back(obs);
                      }


                    // Update the weights and resample
                    pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
                    pf.resample();

                    /****下面的程序是计算权重的最大和平均值******/
                    // Calculate and output the average weighted error of the particle
                    //   filter over all time steps so far.
                    vector<Particle> particles = pf.particles;
                    int num_particles = particles.size();
                    double highest_weight = -1.0;
                    Particle best_particle;
                    double weight_sum = 0.0;
                    for (int i = 0; i < num_particles; ++i) {
                        if (particles[i].weight > highest_weight) {
                            highest_weight = particles[i].weight;
                            best_particle = particles[i];//这么多粒子里面选取权重最大的。
                        }

                        weight_sum += particles[i].weight;
                    }

                    std::cout << "highest w " << highest_weight << std::endl;
                    std::cout << "average w " << weight_sum/num_particles << std::endl;

                    my_particle_filter::particle_data msg;
                    msg.x=best_particle.x;
		            msg.y=best_particle.y;
                    msg.yaw_theta=best_particle.theta;
                    Particle_info_pub.publish(msg);
                    /****输出最好的粒子****/
                     //double msgJson;
                    //msgJson["best_particle_x"] = best_particle.x;
                   // msgJson["best_particle_y"] = best_particle.y;
                   // msgJson["best_particle_theta"] = best_particle.theta;
    }

}