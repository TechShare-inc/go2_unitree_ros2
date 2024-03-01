#include "rclcpp/rclcpp.hpp"
#include "apriltag_service/srv/detect_apriltag.hpp"
#include "sport_rotation.h"
#include <thread>
#include <chrono>

// タグ検出の処理を行う関数
std::shared_ptr<apriltag_service::srv::DetectApriltag::Response> detect_apriltag(
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Client<apriltag_service::srv::DetectApriltag>::SharedPtr& client) {
    auto request = std::make_shared<apriltag_service::srv::DetectApriltag::Request>();
    request->tag_size = 0.162; // 例: 16.2cmのタグサイズ

    auto future_result = client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS) {
        return future_result.get();
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call failed.");
        return nullptr;
    }
}

// 回転の処理を行う関数
void perform_rotation(const std::shared_ptr<sport_rotation>& rotation) {
    rotation->Start();
    rotation->WaitForRotationToComplete();
    std::cout << "180 degree rotation has completed." << std::endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("apriltag_client_simple");
    auto client = node->create_client<apriltag_service::srv::DetectApriltag>("detect_apriltag");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "サービスの中断");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "サービスが利用可能になるのを待っています...");
    }

    // service call
    auto response = detect_apriltag(node, client);
    if (response) {
        if (response->result == 0) {
            RCLCPP_INFO(node->get_logger(), "成功: AprilTag: ID %ld at [x: %f, y: %f, z: %f, rotation: %f]",
                        response->apriltag_id, response->x, response->y, response->z, response->rotation);
        } else if (response->result == 1) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "検出失敗");
        } else if (response->result == 99) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "カメラエラー");
        }
    }

    // go2旋回
    auto rotation = std::make_shared<sport_rotation>(node);
    // spinを別スレッドで実行
    std::thread spin_thread([&]() { rclcpp::spin(node); });

    if(response && response->apriltag_id == 303){
        perform_rotation(rotation);
    }

    // spinスレッドが終了するのを待つ
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
