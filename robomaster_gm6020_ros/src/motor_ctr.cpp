#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <robomaster_gm6020_ros/MotorMessage.h>

#include <iostream>
#include <queue>
#include <deque>
#include <mutex>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>

using namespace std;

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;

    float ref;    // target value
    float fdb;    // feedback value
    float err[2]; // error and last error

    float p_out;
    float i_out;
    float d_out;
    float output;
} pid_struct_t;

typedef struct
{
    int16_t target_volt;
    uint16_t fdb_enc;
    int16_t fdb_rpm;
    int16_t fdb_current;
} motor_ctrl_t;

pid_struct_t motor_pid;  // 电机PID值全局变量
motor_ctrl_t glb_motor;  // 单机反馈的数据和要发送给电机的数据

float speed;        // 设置旋转速度
float angle;        // 设置旋转角度范围

float pid_p;    //pid参数设置
float pid_i;
float pid_d;

int delay_time;

ros::Publisher motor_pub;   // 电机话题发布器

int s;
float glb_angle;
mutex glb_angle_lock;

int direction = 1;      // 旋转方向标记（用于测试方向改变时点云的错位情况）



// pid参数初始化函数
static void pid_init(pid_struct_t *pid, float kp, float ki, float kd, float i_max, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
}

// pid计算函数
static float pid_calc(pid_struct_t *pid, float ref, float fdb)
{
    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;

    pid->p_out = pid->kp * pid->err[0];
    pid->i_out += pid->ki * pid->err[0];
    pid->d_out = pid->kd * pid->err[0] - pid->err[1];
    LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
    return pid->output;
}

// CAN信号发送函数，发送给电机
static void *can_send_thread(void *arg)
{

    float target_rpm = speed; // speed 全局变量，在主函数设置

    //pid_init(&motor_pid, 10, 2, 0, 30000, 30000); 
    pid_init(&motor_pid, pid_p, pid_i, pid_d, 30000, 30000);

    glb_motor.target_volt = 0;

    int nbytes;
    struct can_frame frame;     // 发送给电机的报文

    frame.can_id = 0x2FF; // 电机接受报文标识符 1-4 0x1FF 2-7 0x2FF
    frame.can_dlc = 8;  // 报文长度
    int i=0;

    while (1)
    {
        if(i++%10==0)
        {
            // glb_motor.target_volt 为16位数据，右移8位将高八位赋值给data[4]
            frame.data[4] = (glb_motor.target_volt >> 8) & 0xff;  
            frame.data[5] = (glb_motor.target_volt) & 0xff;     // 低8位赋值给data[5]

            nbytes = write(s, &frame, sizeof(frame));
            if (nbytes != sizeof(frame))
            {
                printf("Send Error frame\n!");
                break;
            }
        }
        glb_angle_lock.lock();
        
        if (glb_angle > 177 + angle/2) //357
        {
            target_rpm = -speed;
            // direction = 1;
        }

        if (glb_angle < 183 - angle/2) //3
        {
            target_rpm = speed;
            // direction = 0;
        }

        if(target_rpm > 0) direction = 1;
        else direction = 0;

        robomaster_gm6020_ros::MotorMessage msg;
        msg.angle = glb_angle;
        msg.direction = direction;
        motor_pub.publish(msg);

            
        glb_angle_lock.unlock();

        glb_motor.target_volt = pid_calc(&motor_pid, target_rpm, glb_motor.fdb_rpm);
        
        usleep(delay_time);  // us
    }

    return NULL;
}

// CAN信号读取函数，接收从电机反馈来的帧数据，主要是记录当前的角度值存放至全局变量 glb_angle 中
static void *can_recv_thread(void *arg)
{
    struct can_frame frame;  // 接受的CAN信号的数据
    struct timeval tv;  // 时间戳
    fd_set rset;    // long数组，用来存放文件标识符

    int nbytes, ret;

    while (1)
    {
        tv.tv_sec = 0;
        tv.tv_usec = 200;

        FD_ZERO(&rset);
        FD_SET(s, &rset);

        // int select(int maxfd, fd_set *readfds, fd_set *writefds,fd_set *exceptfds, struct timeval *timeout); 
        // 将timeout置为NULL时，表明此时select是阻塞的，但是已经设为阻塞模式了那下面的判定 ret为0还有意义吗？
        // 总之上面的 fd_set 相关的东西就是设置文件标识符啥的，然后最后给select()函数作为参数，select()函数又是和套接字息息相关的
        // 这又和 linux 一切程序都是文件形式息息相关，包括套接字
        ret = select(s + 1, &rset, NULL, NULL, NULL);

        if (0 == ret)  // 接受超时      
        {
            return NULL;
        }


        // 从下面开始读取数据
        nbytes = read(s, &frame, sizeof(frame));

        if (nbytes > 0)
        {
            // 左移运算符 高八位数据左移八位再与低八位数据做或运算
            glb_motor.fdb_enc = ((frame.data[0]) << 8 | frame.data[1]);     // 角度
            glb_motor.fdb_rpm = ((frame.data[2]) << 8 | frame.data[3]);     // 转速
            glb_motor.fdb_current = ((frame.data[4]) << 8 | frame.data[5]);     // 电流

            glb_angle_lock.lock();      // 加锁，写入当前角度值
            glb_angle = glb_motor.fdb_enc * 360 / 8192.0;
            glb_angle_lock.unlock();
            //printf("enc:%d rpm:%d angle:%.2lf\n", glb_motor.fdb_enc, glb_motor.fdb_rpm, glb_angle);

        }
    }

    return NULL;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_ctr");
    ros::NodeHandle n;

    motor_pub = n.advertise<robomaster_gm6020_ros::MotorMessage>("motor_info", 1000);

    n.param<float>("/speed", speed, 1);
    n.param<float>("/angle", angle, 180);
    n.param<float>("/pid_p", pid_p, 10);
    n.param<float>("/pid_i", pid_i, 2);
    n.param<float>("/pid_d", pid_d, 0);
    n.param<int>("/delay_time", delay_time, 1000);

    // speed = 1;
    // angle = 180;

    ROS_INFO("/speed: %f", speed);
    ROS_INFO("/angle: %f", angle);
    ROS_INFO("/pid_p: %f", pid_p);
    ROS_INFO("/pid_i: %f", pid_i);
    ROS_INFO("/pid_d: %f", pid_d);
    ROS_INFO("/delay_time: %d", delay_time);
    

    //-------------------------------CAN通信配置------------------------------------
    
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter rfilter[1];

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);  // 创建套接字
    strcpy(ifr.ifr_name, "can0");
    //can0
    ioctl(s, SIOCGIFINDEX, &ifr);  // 指定CAN0设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr *)&addr, sizeof(addr));   // 将套接字与CAN0绑定

    //set up filter rules
    rfilter[0].can_id = 0x20B;   // 电机ID设为7 电机反馈报文标识符为：0x204+ID
    rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));  // 设置过滤规则

    //-----------------------------------------------------------------------------

    //-------------------------------收发多线程配置-----------------------------------
    
    pthread_t thread_send;
    pthread_t thread_recv;


    /* 线程创建函数 
    int pthread_create(pthread_t *tid, const pthread_attr_t *attr, void* (*pSink)(void*), void *arg);
    参数分别为：线程ID 线程属性 线程回调函数 和 传入线程的参数

    */
    pthread_create(&thread_send, NULL, can_send_thread, NULL);  // CAN信号发送线程
    pthread_create(&thread_recv, NULL, can_recv_thread, NULL);  // CAN信号接受线程

    //-----------------------------------------------------------------------------

    cout<<"Motor Start."<<endl;
    


    ros::spin();
    return 0;
}
