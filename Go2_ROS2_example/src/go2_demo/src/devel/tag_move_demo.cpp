#include "rclcpp/rclcpp.hpp"
#include "apriltag_service/srv/detect_apriltag.hpp"

#include "sport_rotation.h"
#include <thread>

using namespace std::chrono_literals; // この行を追加して、'1s' のような時間リテラルを使えるようにします。

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

    auto request = std::make_shared<apriltag_service::srv::DetectApriltag::Request>();
    // tag_sizeをリクエストに設定します。具体的な値は、使用するタグのサイズによります。
    request->tag_size = 0.162; // 例: 16.2cmのタグサイズ

    auto future_result = client->async_send_request(request);

    // レスポンスを待機
    auto response = std::make_shared<apriltag_service::srv::DetectApriltag::Response>();
    if (rclcpp::spin_until_future_complete(node, future_result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        response = future_result.get();
        if (response->result == 0) {
            RCLCPP_INFO(node->get_logger(), "成功: AprilTag: ID %ld at [x: %f, y: %f, z: %f, rotation: %f]",
                        response->apriltag_id, response->x, response->y, response->z, response->rotation);
        } else if (response->result == 1) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "検出失敗");
        } else if (response->result == 99) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "カメラエラー");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call failed.");
    }


    auto rotation = std::make_shared<sport_rotation>(node);
    // spinを別スレッドで実行
    std::thread spin_thread([&]() { rclcpp::spin(node); });

    if(response->apriltag_id == 303){
        // 明示的に動作を開始する
        rotation->Start();
        // 180度回転が完了するまで待機
        rotation->WaitForRotationToComplete();
        std::cout << "180 degree rotation has completed." << std::endl;
    }

    // spinスレッドが終了するのを待つ
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
