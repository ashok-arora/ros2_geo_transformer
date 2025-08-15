#include <memory>
#include <GeographicLib/LocalCartesian.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geo_transformer_interfaces/srv/set_origin.hpp"
#include "geo_transformer_interfaces/srv/get_origin.hpp"
#include "geo_transformer_interfaces/srv/from_ll.hpp"
#include "geo_transformer_interfaces/srv/to_ll.hpp"

class GeoTransformerNode : public rclcpp::Node
{
public:
  GeoTransformerNode()
      : Node("geo_transformer_node"), origin_set_(false)
  {
    // Service to set the origin
    set_origin_srv_ = create_service<geo_transformer_interfaces::srv::SetOrigin>(
        "/local_coordinate/set",
        std::bind(&GeoTransformerNode::setOrigin, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service to get the origin
    get_origin_srv_ = create_service<geo_transformer_interfaces::srv::GetOrigin>(
        "/local_coordinate/get",
        std::bind(&GeoTransformerNode::getOrigin, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service to convert from latitude/longitude to local coordinates
    from_ll_srv_ = create_service<geo_transformer_interfaces::srv::FromLL>(
        "/from_ll",
        std::bind(&GeoTransformerNode::fromLL, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service to convert from local coordinates to latitude/longitude
    to_ll_srv_ = create_service<geo_transformer_interfaces::srv::ToLL>(
        "/to_ll",
        std::bind(&GeoTransformerNode::toLL, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "GeoTransformer node started.");
  }

private:
  // ----- Member variables -----
  bool origin_set_;
  double lat_origin_{}, lon_origin_{}, alt_origin_{};
  GeographicLib::LocalCartesian proj_;

  rclcpp::Service<geo_transformer_interfaces::srv::SetOrigin>::SharedPtr set_origin_srv_;
  rclcpp::Service<geo_transformer_interfaces::srv::GetOrigin>::SharedPtr get_origin_srv_;
  rclcpp::Service<geo_transformer_interfaces::srv::FromLL>::SharedPtr from_ll_srv_;
  rclcpp::Service<geo_transformer_interfaces::srv::ToLL>::SharedPtr to_ll_srv_;

  // ----- Service callbacks -----
  void setOrigin(
      const std::shared_ptr<geo_transformer_interfaces::srv::SetOrigin::Request> req,
      std::shared_ptr<geo_transformer_interfaces::srv::SetOrigin::Response> res)
  {
    // Validate latitude and longitude
    if (req->latitude < -90.0 || req->latitude > 90.0 ||
        req->longitude < -180.0 || req->longitude > 180.0)
    {
      res->success = false;
      res->message = "Invalid latitude/longitude range.";
      RCLCPP_WARN(get_logger(), "Rejecting origin: lat=%.6f lon=%.6f",
                  req->latitude, req->longitude);
      return;
    }

    // Set origin and reset the projection
    lat_origin_ = req->latitude;
    lon_origin_ = req->longitude;
    alt_origin_ = req->altitude;
    proj_.Reset(lat_origin_, lon_origin_, alt_origin_);
    origin_set_ = true;

    res->success = true;
    res->message = "Origin set successfully.";
    RCLCPP_INFO(get_logger(), "Origin set: lat=%.6f lon=%.6f alt=%.3f",
                lat_origin_, lon_origin_, alt_origin_);
  }

  void getOrigin(
      const std::shared_ptr<geo_transformer_interfaces::srv::GetOrigin::Request> /*req*/,
      std::shared_ptr<geo_transformer_interfaces::srv::GetOrigin::Response> res)
  {
    if (!origin_set_)
    {
      RCLCPP_WARN(get_logger(), "Origin not set; returning zeros.");
      res->latitude = 0.0;
      res->longitude = 0.0;
      res->altitude = 0.0;
      return;
    }

    res->latitude = lat_origin_;
    res->longitude = lon_origin_;
    res->altitude = alt_origin_;
  }

  void fromLL(
      const std::shared_ptr<geo_transformer_interfaces::srv::FromLL::Request> req,
      std::shared_ptr<geo_transformer_interfaces::srv::FromLL::Response> res)
  {
    if (!origin_set_)
    {
      res->success = false;
      res->message = "Origin not set.";
      RCLCPP_WARN(get_logger(), "Cannot convert: origin not set.");
      return;
    }

    double x, y, z;
    try
    {
      proj_.Forward(req->latitude, req->longitude, req->altitude, x, y, z);
      res->x = x;
      res->y = y;
      res->z = z;
      res->success = true;
      res->message = "OK";
    }
    catch (const std::exception &e)
    {
      res->success = false;
      res->message = e.what();
      RCLCPP_ERROR(get_logger(), "fromLL exception: %s", e.what());
    }
  }

  void toLL(
      const std::shared_ptr<geo_transformer_interfaces::srv::ToLL::Request> req,
      std::shared_ptr<geo_transformer_interfaces::srv::ToLL::Response> res)
  {
    if (!origin_set_)
    {
      res->success = false;
      res->message = "Origin not set.";
      RCLCPP_WARN(get_logger(), "Cannot convert: origin not set.");
      return;
    }

    double lat, lon, alt;
    try
    {
      proj_.Reverse(req->x, req->y, req->z, lat, lon, alt);
      res->latitude = lat;
      res->longitude = lon;
      res->altitude = alt;
      res->success = true;
      res->message = "OK";
    }
    catch (const std::exception &e)
    {
      res->success = false;
      res->message = e.what();
      RCLCPP_ERROR(get_logger(), "toLL exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeoTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
