/**
 * CRC Calculator Node
 * Subscribes to /lowcmd_raw, calculates CRC, and publishes to /lowcmd
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "common/motor_crc.h" // CRC計算用ヘッダファイル

class CRCCalculator : public rclcpp::Node
{
public:
    CRCCalculator() : Node("crc_calculator")
    {
        // /lowcmd_raw を購読
        raw_subscriber_ = this->create_subscription<unitree_go::msg::LowCmd>(
            "/lowcmd_raw",
            10,
            std::bind(&CRCCalculator::raw_callback, this, std::placeholders::_1)
        );

        // /lowcmd を配信
        crc_publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

        RCLCPP_INFO(this->get_logger(), "CRC Calculator Node has been started.");
    }

private:
    void raw_callback(const unitree_go::msg::LowCmd::SharedPtr msg)
    {
        unitree_go::msg::LowCmd crc_msg = *msg; // メッセージをコピー

        // CRCを計算
        get_crc(crc_msg);

        // CRC付きメッセージを配信
        crc_publisher_->publish(crc_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published /lowcmd with CRC.");
    }

    rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr raw_subscriber_; // /lowcmd_raw 購読者
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr crc_publisher_;      // /lowcmd パブリッシャー
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                             // rclcpp を初期化
    auto node = std::make_shared<CRCCalculator>();        // ノードを作成
    rclcpp::spin(node);                                   // ノードをスピン
    rclcpp::shutdown();                                   // シャットダウン
    return 0;
}
