#include "ros/ros.h" // Funcionalidade básica do ROS
#include "std_msgs/String.h" // Tipo de mensagem String do pacote std_msgs
#include <sstream> // Biblioteca para manipulação de strings

class Talker {
public:
    Talker() {
        // Anuncia que vamos publicar mensagens do tipo std_msgs::String no tópico "chatter"
        pub = nh.advertise<std_msgs::String>("chatter", 1000); // Buffer de 1000 mensagens
    }

    void publishMsg() {
        std_msgs::String msg; // Este objeto armazena a mensagem que será enviada
        std::stringstream ss; // Stream para construção da string da mensagem
        ss << "Hello World, disse a função Talker"; // Mensagem a ser enviada
        msg.data = ss.str(); // Atribui a string do stringstream ao campo de dados da mensagem

        pub.publish(msg); // Publica a mensagem
        ROS_INFO("[Talker] Eu disse: %s", msg.data.c_str()); // Imprime a mensagem no console do ROS
    }

private:
    ros::NodeHandle nh; // Handle do nó, usado para criar subscribers e publishers
    ros::Publisher pub; // Objeto publicador
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker"); // Inicializa o nó com o nome "talker"
    Talker talker; // Cria uma instância da classe Talker
    ros::Rate loop_rate(10); // Define a frequência do loop em 10 Hz

    while (ros::ok()) { // Loop enquanto o ROS estiver funcionando corretamente
        talker.publishMsg(); // Chama o método para publicar a mensagem
        ros::spinOnce(); // Permite que o ROS processe callbacks pendentes
        loop_rate.sleep(); // Dorme o tempo necessário para manter a frequência do loop
    }
    return 0;
}
