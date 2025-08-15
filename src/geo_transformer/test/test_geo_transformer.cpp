#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

#include "geo_transformer_interfaces/srv/set_origin.hpp"
#include "geo_transformer_interfaces/srv/get_origin.hpp"
#include "geo_transformer_interfaces/srv/from_ll.hpp"
#include "geo_transformer_interfaces/srv/to_ll.hpp"

using namespace std::chrono_literals;

class GeoTransformerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("geo_transformer_test");

    client_set_origin_ = node_->create_client<geo_transformer_interfaces::srv::SetOrigin>("/local_coordinate/set");
    client_get_origin_ = node_->create_client<geo_transformer_interfaces::srv::GetOrigin>("/local_coordinate/get");
    client_from_ll_ = node_->create_client<geo_transformer_interfaces::srv::FromLL>("/from_ll");
    client_to_ll_   = node_->create_client<geo_transformer_interfaces::srv::ToLL>("/to_ll");

    std::this_thread::sleep_for(500ms);
  }

  void TearDown() override
  {
    // Destroy clients before shutting down
    client_set_origin_.reset();
    client_get_origin_.reset();
    client_from_ll_.reset();
    client_to_ll_.reset();
    node_.reset();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr callService(
    typename ServiceT::Request::SharedPtr req,
    typename rclcpp::Client<ServiceT>::SharedPtr client)
  {
    if (!client->wait_for_service(2s)) {
      throw std::runtime_error("Service not available");
    }
    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, future, 2s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("Service call failed");
    }
    return future.get();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<geo_transformer_interfaces::srv::SetOrigin>::SharedPtr client_set_origin_;
  rclcpp::Client<geo_transformer_interfaces::srv::GetOrigin>::SharedPtr client_get_origin_;
  rclcpp::Client<geo_transformer_interfaces::srv::FromLL>::SharedPtr client_from_ll_;
  rclcpp::Client<geo_transformer_interfaces::srv::ToLL>::SharedPtr client_to_ll_;
};


TEST_F(GeoTransformerTest, SetAndGetOrigin)
{
  auto req = std::make_shared<geo_transformer_interfaces::srv::SetOrigin::Request>();
  req->latitude = 37.0;
  req->longitude = -122.0;
  req->altitude = 10.0;

  auto resp = callService<geo_transformer_interfaces::srv::SetOrigin>(req, client_set_origin_);
  ASSERT_TRUE(resp->success);

  auto get_req = std::make_shared<geo_transformer_interfaces::srv::GetOrigin::Request>();
  auto get_resp = callService<geo_transformer_interfaces::srv::GetOrigin>(get_req, client_get_origin_);

  EXPECT_NEAR(get_resp->latitude, 37.0, 1e-6);
  EXPECT_NEAR(get_resp->longitude, -122.0, 1e-6);
  EXPECT_NEAR(get_resp->altitude, 10.0, 1e-3);
}

TEST_F(GeoTransformerTest, RoundTripLL)
{
  // Set origin first
  auto req = std::make_shared<geo_transformer_interfaces::srv::SetOrigin::Request>();
  req->latitude = 37.0;
  req->longitude = -122.0;
  req->altitude = 0.0;
  callService<geo_transformer_interfaces::srv::SetOrigin>(req, client_set_origin_);

  // Convert from lat/lon to local
  auto from_req = std::make_shared<geo_transformer_interfaces::srv::FromLL::Request>();
  from_req->latitude = 37.0001;
  from_req->longitude = -122.0001;
  from_req->altitude = 5.0;
  auto from_resp = callService<geo_transformer_interfaces::srv::FromLL>(from_req, client_from_ll_);

  ASSERT_TRUE(from_resp->success);

  // Convert back to lat/lon
  auto to_req = std::make_shared<geo_transformer_interfaces::srv::ToLL::Request>();
  to_req->x = from_resp->x;
  to_req->y = from_resp->y;
  to_req->z = from_resp->z;
  auto to_resp = callService<geo_transformer_interfaces::srv::ToLL>(to_req, client_to_ll_);

  ASSERT_TRUE(to_resp->success);

  EXPECT_NEAR(to_resp->latitude, from_req->latitude, 1e-7);
  EXPECT_NEAR(to_resp->longitude, from_req->longitude, 1e-7);
  EXPECT_NEAR(to_resp->altitude, from_req->altitude, 1e-3);
}
