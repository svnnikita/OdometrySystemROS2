// trajectory_publisher_v2_node -- узел для построения траектории с учетом положения робота

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <math.h>

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher():
        Node("trajectory_publisher_node")
    {
        // коэффициент преобразования пиксельного смещения в метрическое
        this->declare_parameter("pixel2meter", 0.001);
        pixel2meter = this->get_parameter("pixel2meter").as_double();

        // расстояние между камерами по Y
        this->declare_parameter("baseline", 0.46);
        baseline = this->get_parameter("baseline").as_double();

        // смещение камер по X
        this->declare_parameter("forward_offset", 0.072);
        forward_offset = this->get_parameter("forward_offset").as_double();

        // фильтры-подписчики на мгновенные смещения изображения
        r_sub.subscribe(this, "/robot/right_point");
        l_sub.subscribe(this, "/robot/left_point");

        // синхронизатор с приблизительным временем
        sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), r_sub, l_sub);
        sync->registerCallback(std::bind(&TrajectoryPublisher::callback,
                                         this,
                                         std::placeholders::_1,
                                         std::placeholders::_2));

        // публикатор траектории
        pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/trajectory", 10);
    }
private:
    // фильтры-подписчики на смещения изображения с камер
    message_filters::Subscriber<geometry_msgs::msg::PointStamped> r_sub, l_sub;

    // используем синхронизатор для получения данных с близкими временными метками
    // при этом создадим псевдоним для более короткой записи
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<    // указываем типы
            geometry_msgs::msg::PointStamped, geometry_msgs::msg::PointStamped>;
    // объявляем синхронизатор
    std::shared_ptr<
        message_filters::Synchronizer<SyncPolicy>> sync;

    // публикатор траектории с учетом ориентации
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    // публикатор траектории
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

    // глобальные смещения
    double x = 0.0, y = 0.0, theta = 0.0;

    // параметры для вычисления траектории
    double pixel2meter;
    double baseline;
    double forward_offset;

    // double rx_sum, ry_sum, lx_sum, ly_sum;

    // получаем синхронизированные смещения из топиков и вычисляем траекторию
    void callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr &r_point,
                  const geometry_msgs::msg::PointStamped::ConstSharedPtr &l_point)
    {
        // сохраняем данные о смещении из соответствующих полей
        double r_dx = r_point->point.x * pixel2meter;
        double r_dy = r_point->point.y * pixel2meter;
        double l_dx = l_point->point.x * pixel2meter;
        double l_dy = l_point->point.y * pixel2meter;

        // выводим значения для отладки
        // RCLCPP_INFO(this->get_logger(), "r_dx = %.4f, r_dy = %.4f, l_dx = %.4f, l_dy = %.4f", r_dx, r_dy, l_dx, l_dy);

        // rx_sum += r_dx;
        // ry_sum += r_dy;
        // lx_sum += l_dx;
        // ly_sum += l_dy;

        // RCLCPP_INFO(this->get_logger(), "r_dx = %.4f, r_dy = %.4f, l_dx = %.4f, l_dy = %.4f", rx_sum, ry_sum, lx_sum, ly_sum);

        // Фильтрация малых шумов (опционально)
        // double delta = 1e-7;
        // if (fabs(r_dx) < delta && fabs(l_dx) < delta && fabs(r_dy) < delta && fabs(l_dy) < delta)
        //     return;

        // вычисляем угловое приращение в радианах
        double dtheta = (r_dx - l_dx) / baseline;
        // линейное приращение центра робота
        double dx_center = (r_dx + l_dx) / 2.0;
        double dy_center = ((r_dy + l_dy) / 2.0) - (forward_offset * dtheta);

        // RCLCPP_INFO(this->get_logger(), "dx_center = %.4f, dy_center = %.4f, dtheta = %.4f", dx_center, dy_center, dtheta);

        // преобразуем координаты камер в глобальные координаты
        double dx_global = dx_center * cos(theta) - dy_center * sin(theta);
        double dy_global = dx_center * sin(theta) + dy_center * cos(theta);

        // RCLCPP_INFO(this->get_logger(), "dx_global = %.4f, dy_global = %.4f", dx_global, dy_global);

        x += dx_global;
        y += dy_global;
        theta += dtheta;
        theta = atan2(sin(theta), cos(theta));

        // выводим значения для оценки точности
        RCLCPP_INFO(this->get_logger(), "x = %.4f, y = %.4f, theta = %.4f", x, y, theta);

        // формируем позицию робота
        // создаем объект для построения траектории
        geometry_msgs::msg::PoseStamped pose;
        // заполняем поля сообщения
        pose.header.stamp = this->now();
        pose.header.frame_id = "odom";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        // создаем кватернион для задания ориентации
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        pose.pose.orientation = tf2::toMsg(q);

        // публикуем траекторию
        pose_pub->publish(pose);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
