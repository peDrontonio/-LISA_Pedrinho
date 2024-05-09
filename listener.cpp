#include "ros/ros.h" //tem no outro la,presta atençao
#include "std_msgs/String.h"//esse tmb

class Listener{
    public: 
    Listener(){
        sub = nh.subscribe("chatter", 1000, &Listener::chatterCallback, this); //'increve o código' no tópico chatter

    }
     void chatterCallback(const std_msgs::String::ConstPtr& msg) { //sempre que uma msg é recevida no tópicp
        ROS_INFO("[Listener] I heard: %s", msg->data.c_str());//ele imprime no console
    }

    private:
    ros::NodeHandle nh; //tem no pub
    ros::Subscriber sub;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");//inicia um nó
    Listener listener;//cria a instancia Listener
    ros::spin();//entra em ros spin, que é um loop de callbacks 
    return 0;
}
