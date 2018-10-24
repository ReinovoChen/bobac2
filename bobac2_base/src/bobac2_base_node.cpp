#include "ros/ros.h"
#include "bobac2_msgs/car_data.h"
#include "modbus/modbus-rtu.h"
#include "vector"
#include "string"
#include  "bobac2_msgs/car_cmd.h"

#define READ_SPEED 0
#define READ_CRASH 12
#define READ_INFRARED 15
#define READ_ULTRASONIC 18
#define READ_SMOKE  24
#define READ_POWER_VOLTAGE  25
#define READ_MPU  26
#define READ_HTU21D 44

using namespace std;

uint16_t mbbuf[49];
uint16_t probeen=0;
bobac2_msgs::car_data  my_data;
modbus_t *mb=NULL;

void translate_mb_data(void)
{
    union {
        double dd;
        uint16_t id[4];
    } d_to_uint;
    uint16_t *mbp=&mbbuf[READ_SPEED];
    for(int i=0; i<3; i++) {
        d_to_uint.id[3]=*mbp++;
        d_to_uint.id[2]=*mbp++;
        d_to_uint.id[1]=*mbp++;
        d_to_uint.id[0]=*mbp++;
        my_data.speed[i]=d_to_uint.dd;
    }
    mbp=&mbbuf[READ_CRASH];
    for(int i=0; i<3; i++) {
        my_data.crash[i]=*mbp++;
    }
    mbp=&mbbuf[READ_INFRARED];
    for(int i=0; i<3; i++) {
        my_data.infrared[i]=*mbp++;
    }
    mbp=&mbbuf[READ_ULTRASONIC];
    for(int i=0; i<6; i++) {
        my_data.ultrasonic[i]=*mbp++;
    }
    my_data.smoke=mbbuf[READ_SMOKE];
    my_data.power_voltage=mbbuf[READ_POWER_VOLTAGE];
    mbp=&mbbuf[READ_MPU];
    union {
        float fd;
        uint16_t id[2];
    } f_to_uint;
    for(int i=0; i<9; i++) {
        f_to_uint.id[1]=*mbp++;
        f_to_uint.id[0]=*mbp++;
        my_data.mpu[i]=f_to_uint.fd;
    }
    mbp=&mbbuf[READ_HTU21D];
    f_to_uint.id[1]=*mbp++;
    f_to_uint.id[0]=*mbp++;
    my_data.tempareture=f_to_uint.fd;
    f_to_uint.id[1]=*mbp++;
    f_to_uint.id[0]=*mbp++;
    my_data.humidity=f_to_uint.fd;
}

void twistCallback(const bobac2_msgs::car_cmd & mycmd)
{
    double speed[3];
    speed[0]=mycmd.speed[0];
    speed[1]=mycmd.speed[1];
    if(mycmd.speed.size()==2)
        speed[2]=0;
    else
        speed[2]=mycmd.speed[2];
    union {
        double dd;
        uint16_t id[4];
    } d_to_uint;
    uint16_t send_data[13];
    uint16_t *p=send_data;
    for(int i=0; i<3; i++) {
        d_to_uint.dd=speed[i];
        *p++=d_to_uint.id[3];
        *p++=d_to_uint.id[2];
        *p++=d_to_uint.id[1];
        *p++=d_to_uint.id[0];
    }
    if(mycmd.been!=probeen)  send_data[12]=mycmd.been;
    else send_data[12]=0;
    probeen=mycmd.been;
    int res = modbus_write_registers(mb,0,13,send_data);
    //std::cout << "res: " << res << std::endl;
}
std::string serial_port="/dev/ttyS";
std::string usbserial_port="/dev/ttyUSB";
std::string connection_port = "";
int main(int argc, char **argv)
{
    std::vector<std::string> port;
    port.push_back(serial_port);
    port.push_back(usbserial_port);
    ROS_INFO("scaning available port");
    bool is_connected = false;

    for(int j=0; j<port.size(); j++) {
        for(int i=0; i < 127; i++) {
            std::string curr_port = port[j] + std::to_string(i);
            if(!(mb=modbus_new_rtu(curr_port.c_str(),115200,'N',8,1))) continue;
            if(modbus_set_slave(mb,1)==-1) {
                modbus_free(mb);
                continue;
            }
            if(modbus_connect(mb)==-1) {
                modbus_free(mb);
                continue;
            }
            modbus_set_response_timeout(mb,0,200000);
            if(modbus_read_input_registers(mb,0,49,mbbuf)==49) {
                connection_port = curr_port;
                is_connected = true;
                break;
            }
        }//end for

        if(is_connected) break;
        continue;
    }//end for
    if(!is_connected) {
        ROS_ERROR_STREAM("control borad connection failed");
        return -1;
    }
    ROS_INFO_STREAM("available prot: " << connection_port);
    modbus_set_response_timeout(mb,0,200000);

    my_data.speed.resize(3);
    my_data.crash.resize(3);
    my_data.infrared.resize(3);
    my_data.ultrasonic.resize(6);
    my_data.mpu.resize(9);

    ros::init(argc, argv, "car_controller");
    ros::NodeHandle nh;
    ros::Publisher car_data_pub = nh.advertise<bobac2_msgs::car_data>("car_data", 10);
    ros::Subscriber car_control_sub=nh.subscribe("car_cmd",10,twistCallback);

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        memset(mbbuf,0,49*2);
        if(modbus_read_input_registers(mb,0,49,mbbuf)==49) {
            translate_mb_data();
            car_data_pub.publish(my_data);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    modbus_close(mb);
    modbus_free(mb);
    return 0;
}
