// Subscriber-only receipt log for ROS versions whose rclpy omits publisher_gid.
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using Message = tf2_msgs::msg::TFMessage;

auto subscribe(
  const rclcpp::Node::SharedPtr & node, const char * topic, const rclcpp::QoS & qos,
  std::ofstream & receipts)
{
  return node->create_subscription<Message>(
    topic, qos,
    [topic, &receipts](Message::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
      const auto & gid = info.get_rmw_message_info().publisher_gid;
      std::ostringstream hex;
      hex << std::hex << std::setfill('0');
      for (const auto octet : gid.data) {
        hex << std::setw(2) << static_cast<unsigned>(octet);
      }
      for (const auto & tf : message->transforms) {
        const auto stamp = static_cast<std::int64_t>(tf.header.stamp.sec) * 1000000000 +
          tf.header.stamp.nanosec;
        receipts << "TF_RECEIPT " << topic << ' ' << hex.str() << ' '
                  << stamp << ' ' << std::quoted(tf.header.frame_id) << ' '
                  << std::quoted(tf.child_frame_id) << '\n';
      }
      receipts << std::flush;
    });
}

int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "Expected a receipt journal path\n";
    return 2;
  }
  std::ofstream receipts;
  receipts.exceptions(std::ios::failbit | std::ios::badbit);
  receipts.open(argv[1]);
  rclcpp::init(0, nullptr);
  const auto node = std::make_shared<rclcpp::Node>("sie_step755_tf_receipts");
  const auto dynamic_tf = subscribe(node, "/tf", rclcpp::QoS(256), receipts);
  const auto static_tf = subscribe(node, "/tf_static", rclcpp::QoS(32).transient_local(), receipts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  receipts.close();
  return 0;
}
