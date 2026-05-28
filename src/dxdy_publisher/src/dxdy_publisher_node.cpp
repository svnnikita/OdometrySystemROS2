// dxdy_publisher_node.cpp -- узел, слушающий порт 8888
// и публикующий смещения в соответствующие топики

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// создадим выровненную структуру с данными для отправки
#pragma pack(push, 1)
struct SensorPacket {
    // левый датчик
    uint8_t  l_motion;
    int8_t   l_dx;
    int8_t   l_dy;
    uint8_t  l_squal;
    uint16_t l_shutter;
    uint8_t  l_max_pix;
    // правый датчик
    uint8_t  r_motion;
    int8_t   r_dx;
    int8_t   r_dy;
    uint8_t  r_squal;
    uint16_t r_shutter;
    uint8_t  r_max_pix;
};
// возвращаем выравнивание
#pragma pack(pop)

class DxDyPublisher : public rclcpp::Node
{
public:
    DxDyPublisher():
        Node("dxdy_publisher")
    {
        // публикаторы смещений для каждой камеры
        r_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "robot/right_point", 10);
        l_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "robot/left_point", 10);

        // создаем udp сокет для получения данных от датчиков
        sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        // проверяем соединение
        if (sock_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        // заполняем структуру с параметрами сервера
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;      // семейство адресов (интернет)
        addr.sin_port = htons(8888);    // порт, к которому привязан сервер
        addr.sin_addr.s_addr = INADDR_ANY;

        // привязываем сокет к адресу и порту и проверяем соединение
        if (bind(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind to port 8888");
            close(sock_fd);
            return;
        }

        // запускаем прослушивание порта и публикацию смещений
        // в отдельном потоке
        udp_thread = std::thread([this]() { udpReceiverLoop(); });
        // udp_thread = std::thread(&DxDyPublisher::udpReceiverLoop(), this);

    }

    // деструктор
    ~DxDyPublisher()
    {
        // опускаем флаг
        running = false;
        //
        if (udp_thread.joinable()) udp_thread.join();
        if (sock_fd >= 0) close(sock_fd);
    }

private:
    // публикаторы смещений
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr r_point_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr l_point_pub;

    // udp сокет
    int sock_fd = -1;
    // отдельный поток для прослушивания и публикации
    std::thread udp_thread;
    // флаг работы потока с udpReceiverLoop()
    bool running = true;

    // метод, в котором слушаем порт и публикуем смещения
    void udpReceiverLoop()
    {
        // создаем буффер, в который будем копировать полученную датаграмму со структурой
        uint8_t buffer[sizeof(SensorPacket)];
        // создаем структуру с ip адресом и портом отправителя
        struct sockaddr_in client_addr;
        // длина структуры адреса
        socklen_t client_len = sizeof(client_addr);

        // бесконечный цикл прослушивания и отправки
        while (rclcpp::ok() && running) {
            // принимаем пакет
            int n = recvfrom(sock_fd, buffer, sizeof(buffer), 0,
                             (struct sockaddr*)&client_addr, &client_len);

            // проверяем пакет
            if (n == sizeof(SensorPacket)) {
                const SensorPacket *pkt = reinterpret_cast<const SensorPacket*>(buffer);

                // публикуем смещения
                publishTotalDxDy("right", pkt->r_dx, pkt->r_dy);
                publishTotalDxDy("left", pkt->l_dx, pkt->l_dy);
            }
        }
    }

    // публикуем глобальные смещения для каждой камеры
    void publishTotalDxDy(const std::string &camera_name, float dx, float dy)
    {
        // создаем объект сообщения (для PointStamped)
        auto msg = geometry_msgs::msg::PointStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.point.x = dx;
        msg.point.y = dy;
        msg.point.z = 0.0;

        // в зависимости от названия камеры публикуем смещения
        if (camera_name == "right") {
            r_point_pub->publish(msg);
            // RCLCPP_INFO_STREAM(this->get_logger(), "\nr_point: \ndx = " << dx << "\ndy = " << dy);
        } else if (camera_name == "left") {
            l_point_pub->publish(msg);
            // RCLCPP_INFO_STREAM(this->get_logger(), "\nl_point: \ndx = " << dx << "\ndy = " << dy);
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DxDyPublisher>());
    rclcpp::shutdown();
    return 0;
}
