
//        Author :- Pratham Mittal
//        Email  :- prathammittal2411@gmail.com
//        Date   :- 9th Aug, 2025
//        Description:- Robot odometry through merging and optimizing all the data from lidar, visual and wheel odometry.
//        Modified:- 3rd Jan, 2026

#include <iostream>
#include <vector>
#include <cmath>
#include <blaze/Math.h> // for faster calculation in matrix multiplication

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // for lidar odo
#include "geometry_msgs/msg/pose2_d.hpp" // for wheel and absolute odo
#include "nav_msgs/msg/odometry.hpp" // for visual odo
#include "std_msgs/msg/char.hpp"
#include "geometry_msgs/msg/twist.hpp" // knowing the velocity of the bot
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"


class robot_odo : public rclcpp::Node{


    // Initializing the publisher and subscriber
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lidar_odo_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odo_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr visual_odo_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr wheel_odo_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr imu_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;          // for transformation


    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::Time last_time;
    rclcpp::Time current_time;
    rclcpp::Time tf_current_time;


    // Initializing the msg for publiser and for transformations
    geometry_msgs::msg::Pose2D robot_odo_msg;
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::TransformStamped t_odom;
    geometry_msgs::msg::TransformStamped t_basefootprint;
    geometry_msgs::msg::TransformStamped t_baselink;
    // geometry_msgs::msg::TransformStamped t_laser;
    geometry_msgs::msg::TransformStamped t_camera;
    geometry_msgs::msg::TransformStamped t_wheel1;
    geometry_msgs::msg::TransformStamped t_wheel2;
    geometry_msgs::msg::TransformStamped t_wheel3;
    geometry_msgs::msg::TransformStamped t_wheel4;



    // Initialization


    // Transformations
    float tf_timer_period = 1.0f / 40.0f;
    const float baselink_z = 0.097f;

    const float tf_lidar_x = 0.265f;
    const float tf_lidar_y = 0.0f;
    const float tf_lidar_z = -0.057f;

    const float tf_camera_x = 0.245f;
    const float tf_camera_y = 0.0f;
    const float tf_camera_z = 0.528f;

    const float tf_wheel1_x = 0.205f;
    const float tf_wheel1_y = 0.2075f;
    const float tf_wheel1_z = -0.037f;

    const float tf_wheel2_x = 0.205f;
    const float tf_wheel2_y = -0.2075f;
    const float tf_wheel2_z = -0.037f;

    const float tf_wheel3_x = -0.205f;
    const float tf_wheel3_y = 0.2075f;
    const float tf_wheel3_z = -0.037f;

    const float tf_wheel4_x = -0.205f;
    const float tf_wheel4_y = -0.2075f;
    const float tf_wheel4_z = -0.037f;



    // Normal
    float time_period = 1.0f / 7.0f;    // 7 Hz
    float N = 3.0f;
    float robot_vel = 0.0f;
    float robot_angv = 0.0f;
    blaze::StaticMatrix<float, 4, 4> I = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    float bias = 0.1f;

    // lidar odometry
    float lidar_x = 0.0f;
    float lidar_y = 0.0f;
    float lidar_theta = 0.0f;

    // visual odometry
    float visual_x = 0.0f;
    float visual_y = 0.0f;
    float visual_theta = 0.0f;

    // wheel odometry
    float wheel_x = 0.0f;
    float wheel_y = 0.0f;
    float wheel_theta = 0.0f;

    // IMU odometery
    float imu_theta = 0.0f;
    float theta_pred;
    float z_imu;

    // Inital State
    float Xo = 0.0f;
    float Yo = 0.0f;
    float thetao = 0.0f;
    blaze::StaticMatrix<float, 1, 4> current_state_XoT = { {Xo, Yo, thetao, bias} };
    blaze::StaticMatrix<float, 4, 1> current_state_Xo = trans(current_state_XoT);


    // Initial uncertainty (covariance)
    float eta_x = 0.0005f;
    float eta_y = 0.0000001f;
    float eta_theta = 0.0001f;
    float eta_bias = 0.1f;
    float eta_bias_process = 7e-3f; // 7 mm per update

    blaze::StaticMatrix<float, 4, 4> Po = {
        {eta_x * eta_x, 0.0f, 0.0f, 0.0f},
        {0.0f, eta_y * eta_y, 0.0f, 0.0f},
        {0.0f, 0.0f, eta_theta * eta_theta, 0.0f},
        {0.0f, 0.0f, 0.0f, eta_bias * eta_bias}
    };

    // DON'T MESS WITH THE FURTHER CODE, WITHOUT THE PERMISSION OF THE AUTHOR
    // IT'S THE CORE OF EKF CALCULATION


    //  High varince means low trust
    // 0.005 is low variance
    // 0.05 is a high variance

    // uncertainty (variance) in the sensor measurements
    // Wheel
    float usm_wheel_x = 0.01f;
    float usm_wheel_y = 0.01f;
    //float usm_wheel_x = 0.0001f; // AGV
    //float usm_wheel_y = 0.0001f; // AGV
    float usm_wheel_theta = 0.0f;

    // Lidar
    //float usm_lidar_x = 0.9f; // AGV
    //float usm_lidar_y = 0.9f; // AGV
    float usm_lidar_x = 0.05f; // Droid
    float usm_lidar_y = 0.05f; // Droid
    float usm_lidar_theta = 1e-6f;

    // Visual
    float usm_visual_x = 0.59f; // Droid
    float usm_visual_y = 0.7f; // Droid
    //float usm_visual_x = 0.0005f; // AGV
    //float usm_visual_y = 0.0005f; // AGV
    float usm_visual_theta = M_PI / 38.0f;

    // IMU
    float usm_imu_theta = M_PI / 36.0f;             // +- 2.5 deg of noise




    // // Mean
    // float usm_mean_x = (usm_lidar_x + usm_visual_x + usm_wheel_x) / N;
    // float usm_mean_y = (usm_lidar_y + usm_visual_y + usm_wheel_y) / N;
    // float usm_mean_theta = (usm_lidar_theta + usm_visual_theta + usm_wheel_theta + usm_imu_theta) / (N+1.0f);

    // // Variance
    // float usm_variance_x = ( std::pow(usm_lidar_x - usm_mean_x, 2)
    //                         + std::pow(usm_visual_x - usm_mean_x, 2)
    //                         + std::pow(usm_wheel_x - usm_mean_x, 2) ) / N;
    
    // float usm_variance_y = ( std::pow(usm_lidar_y- usm_mean_y, 2)
    //                         + std::pow(usm_visual_y - usm_mean_y, 2)
    //                         + std::pow(usm_wheel_y - usm_mean_y, 2) ) / N;
    
    // float usm_variance_theta = ( std::pow(usm_lidar_theta - usm_mean_theta, 2)
    //                         + std::pow(usm_visual_theta - usm_mean_theta, 2)
    //                         + std::pow(usm_wheel_theta - usm_mean_theta, 2) 
    //                         + std::pow(usm_imu_theta - usm_mean_theta, 2) ) / (N+1.0f);

    // blaze::StaticMatrix<float, 3, 3> usm_noise_matrix = {
    //         {usm_variance_x, 0.0f, 0.0f},
    //         {0.0f, usm_variance_y, 0.0f},
    //         {0.0f, 0.0f, usm_variance_theta}
    //     };



    // USM in Wheel reading
    blaze::StaticMatrix<float, 3, 3> R_wheel = {
        {usm_wheel_x * usm_wheel_x, 0.0f, 0.0f},
        {0.0f, usm_wheel_y * usm_wheel_y, 0.0f},
        {0.0f, 0.0f, usm_wheel_theta * usm_wheel_theta}
    };

    // USM in LiDAR reading
    blaze::StaticMatrix<float, 3, 3> R_lidar = {
        {usm_lidar_x * usm_lidar_x, 0.0f, 0.0f},
        {0.0f, usm_lidar_y * usm_lidar_y, 0.0f},
        {0.0f, 0.0f, usm_lidar_theta * usm_lidar_theta}
    };


    // USM in Visual reading
    blaze::StaticMatrix<float, 3, 3> R_visual = {
        {usm_visual_x * usm_visual_x, 0.0f, 0.0f},
        {0.0f, usm_visual_y * usm_visual_y, 0.0f},
        {0.0f, 0.0f, usm_visual_theta * usm_visual_theta}
    };

    // USM in IMU reading
    // blaze::StaticMatrix<float, 3, 3> R_imu = {
    //     {1e-6f, 0.0f, 0.0f},
    //     {0.0f, 1e-6f, 0.0f},
    //     {0.0f, 0.0f, usm_imu_theta * usm_imu_theta}
    // };
    float R_imu = usm_imu_theta * usm_imu_theta;


    // Uncertainty in motion mean variables
    float um_mean_x;
    float um_mean_y;
    float um_mean_theta;

    // Uncertainty in motion variance variables
    float um_variance_x;
    float um_variance_y;
    float um_variance_theta;

    // Uncertainty in motion matrix variables
    blaze::StaticMatrix<float, 4, 4> um_noise_matrix;

    // Jacobian matrix variables
    blaze::StaticMatrix<float, 4, 4> Fk;

    // Measurement Jacobian
    blaze::StaticMatrix<float, 3, 4> H_lidar;
    blaze::StaticMatrix<float, 3, 4> H_visual;
    blaze::StaticMatrix<float, 1, 4> H_imu;

    // Predicted Co-variance variables
    blaze::StaticMatrix<float, 4, 4> Fk_T;

    // Initializing Residual variables
    blaze::StaticMatrix<float, 3, 1> residual_visual;
    blaze::StaticMatrix<float, 3, 1> residual_lidar;
    blaze::StaticMatrix<float, 3, 1> residual_imu;

    // Initializing Innovation covariance variables
    blaze::StaticMatrix<float, 3, 3> Inn_covariance_wheel;
    blaze::StaticMatrix<float, 3, 3> Inn_covariance_lidar;
    blaze::StaticMatrix<float, 3, 3> Inn_covariance_visual;
    // blaze::StaticMatrix<float, 3, 3> Inn_covariance_imu;
    float Inn_covariance_imu;

    // Initializing Kalman Gain filter variables
    blaze::StaticMatrix<float, 3, 3> Kalman_gain_wheel;
    blaze::StaticMatrix<float, 4, 3> Kalman_gain_lidar;
    blaze::StaticMatrix<float, 4, 3> Kalman_gain_visual;
    blaze::StaticMatrix<float, 4, 1> Kalman_gain_imu;







    public:

    robot_odo() : Node("Robot_Odometry_Node"){

        last_time = this->get_clock()->now();

        // Publsher and Subscriber
        publisher_  = this->create_publisher<geometry_msgs::msg::Pose2D>("/position", 10);
        // lidar_odo_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "/lidar_pose", 10, std::bind(&robot_odo::lidar_odo_func, this, std::placeholders::_1)
        // );
        lidar_odo_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_rf2o", 10, std::bind(&robot_odo::lidar_odo_func, this, std::placeholders::_1)
        );
        visual_odo_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/visual_pose", 10, std::bind(&robot_odo::visual_odo_func, this, std::placeholders::_1)
        );
        wheel_odo_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/wheel_pose", 10, std::bind(&robot_odo::wheel_odo_func, this, std::placeholders::_1)
        );
        velo_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&robot_odo::velo_func, this, std::placeholders::_1)
        );
        imu_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/imu_pose", 10, std::bind(&robot_odo::imu_func, this, std::placeholders::_1)
        );

        // Publisher for tf's
        // tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // calling function frequently
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(this->time_period),
            std::bind(&robot_odo::update_func, this)
        );


        // calling tf update function 
        //tf_timer_ = this->create_wall_timer(
        //    std::chrono::duration<double>(this->tf_timer_period),
        //    std::bind(&robot_odo::tf_update_func, this)
        //);

    }









    void imu_func(const geometry_msgs::msg::Pose2D &msg){

        this->imu_theta = msg.theta;

    }


    void lidar_odo_func(const nav_msgs::msg::Odometry &msg){

        // msg.header.stamp = this->now();
        // msg.header.frame_id = "laser";

        // this->lidar_x = msg.pose.position.x;
        // this->lidar_y = msg.pose.position.y;
        // this->lidar_theta = asin(msg.pose.orientation.z) * 2.0;
        // this->lidar_theta = this->imu_theta;

        this->lidar_x = msg.pose.pose.position.x;
        this->lidar_y = msg.pose.pose.position.y;
        this->lidar_theta = this->imu_theta;


    }

    void visual_odo_func(const geometry_msgs::msg::Pose2D &msg){

        this->visual_x = msg.x;
        this->visual_y = msg.y;
        this->visual_theta = msg.theta;

    }

    void wheel_odo_func(const geometry_msgs::msg::Pose2D &msg){

        this->wheel_x = msg.x;
        this->wheel_y = msg.y;
        // this->wheel_theta = msg.theta;
        this->wheel_theta = this->imu_theta;

    }

    void velo_func(const geometry_msgs::msg::Twist &msg){

        this->robot_vel = msg.linear.x;
        this->robot_angv = msg.angular.z;

    }








    void update_func(){


        // std::cout<<"Inside the update func!!"<<std::endl;

        this->current_time = this->get_clock()->now();
        float dt = (this->current_time - this->last_time).nanoseconds() / 1e9;
        this->last_time = this->current_time;




        // Noise matrix (uncertainty in motion)

        // first finding the variance in motion
        // mean of x y and theta
        this->um_mean_x = (this->lidar_x + this->visual_x + this->wheel_x) / this->N;
        this->um_mean_y = (this->lidar_y + this->visual_y + this->wheel_y) / this->N;
        this->um_mean_theta = (this->lidar_theta + this->visual_theta + this->wheel_theta + this->imu_theta) / (this->N + 1.0f);

        this->um_variance_x = ( std::pow(this->lidar_x - this->um_mean_x, 2)
                            + std::pow(this->visual_x - this->um_mean_x, 2)
                            + std::pow(this->wheel_x - this->um_mean_x, 2) ) / this->N;

        this->um_variance_y = ( std::pow(this->lidar_y - this->um_mean_y, 2)
                            + std::pow(this->visual_y - this->um_mean_y, 2)
                            + std::pow(this->wheel_y - this->um_mean_y, 2) ) / this->N;

        this->um_variance_theta = ( std::pow(this->lidar_theta - this->um_mean_theta, 2)
                            + std::pow(this->visual_theta - this->um_mean_theta, 2)
                            + std::pow(this->wheel_theta - this->um_mean_theta, 2) 
                            + std::pow(this->imu_theta - this->um_mean_theta, 2) ) / (this->N + 1.0f);


        this->um_noise_matrix = {
            {this->um_variance_x * this->um_variance_x, 0.0f, 0.0f, 0.0f},
            {0.0f, this->um_variance_y * this->um_variance_y, 0.0f, 0.0f},
            {0.0f, 0.0f, this->um_variance_theta * this->um_variance_theta, 0.0f},
            {0.0f, 0.0f, 0.0f, this->eta_bias_process * this->eta_bias_process}
        };


        // std::cout<<"Ending of the UM matrix"<<std::endl;


        // For Wheel Data
        // Motion model (nonlinear)
        this->Xo = this->Xo + ( this->robot_vel * dt * std::cos(this->thetao) );
        this->Yo = this->Yo + ( this->robot_vel * dt * std::sin(this->thetao) );
        this->thetao =  this->imu_theta;
        current_state_Xo = { {Xo}, {Yo}, {thetao}, {bias} };


        // Linearize with Jacobian (F_k)
        this->Fk = {
            {1.0f, 0.0f, -1.0f * this->robot_vel * dt * std::sin(this->thetao), 0.0f},
            {0.0f, 1.0f, 1.0f * this->robot_vel * dt * std::cos(this->thetao), 0.0f},
            {0.0f, 0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 1.0f}
        };


        // Predicted covariance
        this->Fk_T = trans(this->Fk);

        this->Po = ( this->Fk * this->Po * this->Fk_T ) + um_noise_matrix; // Predicted Covariance


        // std::cout<<"Ending of the Wheel data"<<std::endl;




        // Measurement update (do it sequentially for each sensor at this time step)
        
        // For Lidar data
        // Innovation (residual)
        this->residual_lidar(0,0) = this->lidar_x - ( this->Xo + this->bias );
        this->residual_lidar(1,0) = this->lidar_y - this->Yo;
        this->residual_lidar(2,0) = this->lidar_theta - this->thetao;

        this->H_lidar = {
            {1.0f, 0.0f, 0.0f, 1.0f},   // x + bias affects measured x
            {0.0f, 1.0f, 0.0f, 0.0f},   // y directly observed
            {0.0f, 0.0f, 1.0f, 0.0f}    // θ directly observed
        };

        // Innovation covariance
        this->Inn_covariance_lidar = this->H_lidar * this->Po * trans(this->H_lidar) + this->R_lidar;

        // Now applying Kalman gain factor
        this->Kalman_gain_lidar = this->Po * trans(this->H_lidar)* inv(this->Inn_covariance_lidar);

        // State update
        this->current_state_Xo += (this->Kalman_gain_lidar * this->residual_lidar);

        // Covariance update (Joseph form — numerically robust)
        this->Po = ( (this->I - this->Kalman_gain_lidar * this->H_lidar) * this->Po * trans(this->I - this->Kalman_gain_lidar * this->H_lidar) )
                    + (this->Kalman_gain_lidar * this->R_lidar * trans(this->Kalman_gain_lidar));


        // std::cout<<"Ending of the Lidar data"<<std::endl;




        // For Visual data
        // Innovation (residual)
        this->residual_visual(0,0) = this->visual_x - ( this->Xo + this->bias );
        this->residual_visual(1,0) = this->visual_y - this->Yo;
        this->residual_visual(2,0) = this->visual_theta - this->thetao;

        this->H_visual = {
            {1.0f, 0.0f, 0.0f, 1.0f},   // x + bias affects measured x
            {0.0f, 1.0f, 0.0f, 0.0f},   // y directly observed
            {0.0f, 0.0f, 1.0f, 0.0f}    // θ directly observed
        };

        // Innovation covariance
        this->Inn_covariance_visual = this->H_visual * this->Po * trans(this->H_visual) + this->R_visual;

        // Now applying Kalman gain factor
        this->Kalman_gain_visual = this->Po * trans(this->H_visual) * inv(this->Inn_covariance_visual);

        // State update
        this->current_state_Xo += (this->Kalman_gain_visual * this->residual_visual);

        // Covariance update (Joseph form — numerically robust)
        this->Po = ( (this->I - this->Kalman_gain_visual * this->H_visual) * this->Po * trans(this->I - this->Kalman_gain_visual * this->H_visual) )
                    + (this->Kalman_gain_visual * this->R_visual * trans(this->Kalman_gain_visual));


        // std::cout<<"Ending of the Visual data"<<std::endl;




        // For IMU Data
        this->z_imu = this->imu_theta;              // measured yaw from IMU
        this->theta_pred = this->current_state_Xo(2,0);

        // Innovation (residual)
        this->residual_imu(0,0) = 0.0f;
        this->residual_imu(1,0) = 0.0f;
        this->residual_imu(2,0) = std::atan2(std::sin(this->z_imu - this->theta_pred), std::cos(this->z_imu - this->theta_pred)); // scalar

        this->H_imu = { {0.0f, 0.0f, 1.0f, 0.0f} };

        // Innovation covariance
        // this->Inn_covariance_imu = this->H_imu * this->Po * trans(this->H_imu) + this->R_imu;
        this->Inn_covariance_imu = ( H_imu * this->Po * trans(H_imu) )(0,0) + this->R_imu; // 1x1 -> extract (0,0)

        // Now applying Kalman gain factor
        // this->Kalman_gain_imu = this->Po * trans(this->H_imu) * inv(this->Inn_covariance_imu);
        this->Kalman_gain_imu = this->Po * trans(H_imu) * (1.0f / this->Inn_covariance_imu);

        // State update
        // this->current_state_Xo += (this->Kalman_gain_imu * this->residual_imu);
        this->current_state_Xo += this->Kalman_gain_imu *this->residual_imu(2,0);

        // Covariance update (Joseph form — numerically robust)
        // this->Po = ( (this->I - this->Kalman_gain_imu * this->H_imu) * this->Po * trans(this->I - this->Kalman_gain_imu * this->H_imu) )
        //             + (this->Kalman_gain_imu * this->R_imu * trans(this->Kalman_gain_imu));
        this->Po = ( this->I - (this->Kalman_gain_imu * this->H_imu ) ) * this->Po * trans(( this->I - (this->Kalman_gain_imu * this->H_imu ) ))
                    + ( this->Kalman_gain_imu * this->R_imu * trans( this->Kalman_gain_imu ) );


        // std::cout<<"Ending of the IMU data"<<std::endl;




        this->Xo     = this->current_state_Xo(0,0);
        this->Yo     = this->current_state_Xo(1,0);
        this->thetao = this->current_state_Xo(2,0);
        this->bias   = this->current_state_Xo(3,0);
        // this->thetao = this->imu_theta;

        robot_odo_msg.x = this->Xo;
        robot_odo_msg.y = -1 * this->Yo;	// this -ve is coz of the tf's issues and the thing that the angle's sign convention was not taken into considerstion at first place.
        robot_odo_msg.theta = -1 * this->thetao;

        // RCLCPP_INFO(this->get_logger(), "The pose of to Robot after applying EKF is (%f, %f, %f)", this->Xo, this->Yo, this->thetao);
        publisher_->publish(robot_odo_msg);


        // std::cout<<"Ending of update function"<<std::endl;
    }






    // void tf_update_func(){

    //     tf_current_time = this->get_clock()->now();

    //     t_odom.header.stamp = this->tf_current_time;
    //     t_basefootprint.header.stamp = this->tf_current_time;
    //     t_baselink.header.stamp = this->tf_current_time;
    //     // t_laser.header.stamp = this->tf_current_time;
    //     t_camera.header.stamp = this->tf_current_time;
    //     t_wheel1.header.stamp = this->tf_current_time;
    //     t_wheel2.header.stamp = this->tf_current_time;
    //     t_wheel3.header.stamp = this->tf_current_time;
    //     t_wheel4.header.stamp = this->tf_current_time;


    //     // tf form odom to base_footprint
    //     t_odom.header.frame_id = "odom";
    //     t_odom.child_frame_id = "base_footprint";

    //     t_odom.transform.translation.x = this->Xo;
    //     t_odom.transform.translation.y = this->Yo;
    //     t_odom.transform.translation.z = 0.0f;

    //     tf2::Quaternion q_odom;
    //     q_odom.setRPY(0, 0, this->thetao);
    //     t_odom.transform.rotation.x = q_odom.x();
    //     t_odom.transform.rotation.y = q_odom.y();
    //     t_odom.transform.rotation.z = q_odom.z();
    //     t_odom.transform.rotation.w = q_odom.w();




    //     // tf form base_footprint to base_link
    //     t_basefootprint.header.frame_id = "base_footprint";
    //     t_basefootprint.child_frame_id = "base_link";

    //     t_basefootprint.transform.translation.x = 0.0f;
    //     t_basefootprint.transform.translation.y = 0.0f;
    //     t_basefootprint.transform.translation.z = this->baselink_z;

    //     tf2::Quaternion q_footprint;
    //     q_footprint.setRPY(0, 0, 0);
    //     t_basefootprint.transform.rotation.x = q_footprint.x();
    //     t_basefootprint.transform.rotation.y = q_footprint.y();
    //     t_basefootprint.transform.rotation.z = q_footprint.z();
    //     t_basefootprint.transform.rotation.w = q_footprint.w();




    //     // tf form base_link to laser
    //     t_baselink.header.frame_id = "base_link";
    //     t_baselink.child_frame_id = "laser";

    //     t_baselink.transform.translation.x = this->tf_lidar_x;
    //     t_baselink.transform.translation.y = this->tf_lidar_y;
    //     t_baselink.transform.translation.z = this->tf_lidar_z;

    //     tf2::Quaternion q_baselink;
    //     q_baselink.setRPY(0, 0, 0);
    //     t_baselink.transform.rotation.x = q_baselink.x();
    //     t_baselink.transform.rotation.y = q_baselink.y();
    //     t_baselink.transform.rotation.z = q_baselink.z();
    //     t_baselink.transform.rotation.w = q_baselink.w();




    //     // tf form base_link to camera
    //     t_camera.header.frame_id = "base_link";
    //     t_camera.child_frame_id = "camera";

    //     t_camera.transform.translation.x = this->tf_camera_x;
    //     t_camera.transform.translation.y = this->tf_camera_y;
    //     t_camera.transform.translation.z = this->tf_camera_z;

    //     tf2::Quaternion q_camera;
    //     q_camera.setRPY(0, 0, 0);
    //     t_camera.transform.rotation.x = q_camera.x();
    //     t_camera.transform.rotation.y = q_camera.y();
    //     t_camera.transform.rotation.z = q_camera.z();
    //     t_camera.transform.rotation.w = q_camera.w();




    //     // tf form base_link to wheel1
    //     t_wheel1.header.frame_id = "base_link";
    //     t_wheel1.child_frame_id = "wheel1";

    //     t_wheel1.transform.translation.x = this->tf_wheel1_x;
    //     t_wheel1.transform.translation.y = this->tf_wheel1_y;
    //     t_wheel1.transform.translation.z = this->tf_wheel1_z;

    //     tf2::Quaternion q_wheel1;
    //     q_wheel1.setRPY(0, 0, 0);
    //     t_wheel1.transform.rotation.x = q_wheel1.x();
    //     t_wheel1.transform.rotation.y = q_wheel1.y();
    //     t_wheel1.transform.rotation.z = q_wheel1.z();
    //     t_wheel1.transform.rotation.w = q_wheel1.w();




    //     // tf form base_link to wheel2
    //     t_wheel2.header.frame_id = "base_link";
    //     t_wheel2.child_frame_id = "wheel2";

    //     t_wheel2.transform.translation.x = this->tf_wheel2_x;
    //     t_wheel2.transform.translation.y = this->tf_wheel2_y;
    //     t_wheel2.transform.translation.z = this->tf_wheel2_z;

    //     tf2::Quaternion q_wheel2;
    //     q_wheel2.setRPY(0, 0, 0);
    //     t_wheel2.transform.rotation.x = q_wheel2.x();
    //     t_wheel2.transform.rotation.y = q_wheel2.y();
    //     t_wheel2.transform.rotation.z = q_wheel2.z();
    //     t_wheel2.transform.rotation.w = q_wheel2.w();




    //     // tf form base_link to wheel3
    //     t_wheel3.header.frame_id = "base_link";
    //     t_wheel3.child_frame_id = "wheel3";

    //     t_wheel3.transform.translation.x = this->tf_wheel3_x;
    //     t_wheel3.transform.translation.y = this->tf_wheel3_y;
    //     t_wheel3.transform.translation.z = this->tf_wheel3_z;

    //     tf2::Quaternion q_wheel3;
    //     q_wheel3.setRPY(0, 0, 0);
    //     t_wheel3.transform.rotation.x = q_wheel3.x();
    //     t_wheel3.transform.rotation.y = q_wheel3.y();
    //     t_wheel3.transform.rotation.z = q_wheel3.z();
    //     t_wheel3.transform.rotation.w = q_wheel3.w();




    //     // tf form base_link to wheel4
    //     t_wheel4.header.frame_id = "base_link";
    //     t_wheel4.child_frame_id = "wheel4";

    //     t_wheel4.transform.translation.x = this->tf_wheel4_x;
    //     t_wheel4.transform.translation.y = this->tf_wheel4_y;
    //     t_wheel4.transform.translation.z = this->tf_wheel4_z;

    //     tf2::Quaternion q_wheel4;
    //     q_wheel4.setRPY(0, 0, 0);
    //     t_wheel4.transform.rotation.x = q_wheel4.x();
    //     t_wheel4.transform.rotation.y = q_wheel4.y();
    //     t_wheel4.transform.rotation.z = q_wheel4.z();
    //     t_wheel4.transform.rotation.w = q_wheel4.w();




    //     tf_broadcaster->sendTransform(t_odom);
    //     tf_broadcaster->sendTransform(t_basefootprint);
    //     tf_broadcaster->sendTransform(t_baselink);
    //     tf_broadcaster->sendTransform(t_camera);
    //     tf_broadcaster->sendTransform(t_wheel1);
    //     tf_broadcaster->sendTransform(t_wheel2);
    //     tf_broadcaster->sendTransform(t_wheel3);
    //     tf_broadcaster->sendTransform(t_wheel4);

    // }

};


int main(int argc, char **argv){

    rclcpp::init(argc,argv);
    auto node = std::make_shared<robot_odo>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;

}
