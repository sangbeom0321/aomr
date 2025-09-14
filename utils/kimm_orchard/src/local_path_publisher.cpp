#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

class PathProcessor : public rclcpp::Node {
public:
  PathProcessor()
  : Node("local_path_processor")
  {
    // 파라미터 설정 (옵션으로 선언 가능)
    this->declare_parameter<double>("utm_x_zero", 346638.99532500084);
    this->declare_parameter<double>("utm_y_zero", 4070388.235393337);
    this->get_parameter("utm_x_zero", utm_x_zero_);
    this->get_parameter("utm_y_zero", utm_y_zero_);

    // 구독자
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10,
      std::bind(&PathProcessor::pathCallback, this, _1));

    pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/Local/utm", 10,
      std::bind(&PathProcessor::posCallback, this, _1));

    // 퍼블리셔
    local_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/Planning/local_path", 10);
    viz_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/Planning/local_path_viz", 10);

    // 타이머 10Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&PathProcessor::timerCallback, this));
  }

private:
  // 콜백
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    // 마지막 포인트 제외하고 offset 적용
    global_path_.header = msg->header;
    global_path_.poses.clear();
    for (size_t i = 0; i + 1 < msg->poses.size(); ++i) {
      geometry_msgs::msg::PoseStamped p = msg->poses[i];
      p.pose.position.x -= utm_x_zero_;
      p.pose.position.y -= utm_y_zero_;
      global_path_.poses.push_back(p);
    }
  }

  void posCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
      current_pose_.pose.position.x = msg->data[0] - utm_x_zero_;
      current_pose_.pose.position.y = msg->data[1] - utm_y_zero_;
      current_pose_.pose.position.z = 0.0;
      current_pose_.header.frame_id = "map";
      current_pose_.header.stamp = this->now();
      has_position_ = true;
    }
  }

  void timerCallback() {
    if (!has_position_ || global_path_.poses.empty()) return;

    auto selected = findClosestPoints(global_path_, current_pose_, 100, 30);
    if (selected.empty()) return;

    // Viz 퍼블리시 (선택된 포인트)
    nav_msgs::msg::Path viz_msg;
    viz_msg.header.frame_id = "map";
    viz_msg.header.stamp = this->now();
    viz_msg.poses = selected;
    viz_pub_->publish(viz_msg);

    // 스무딩 적용
    auto smooth = bsplineInterpolation(selected);
    auto resampled = resamplePath(smooth, 0.05);

    // 원 좌표계 복원 및 퍼블리시
    nav_msgs::msg::Path out_msg;
    out_msg.header.frame_id = "map";
    out_msg.header.stamp = this->now();
    for (auto &p : resampled) {
      geometry_msgs::msg::PoseStamped pp = p;
      pp.pose.position.x += utm_x_zero_;
      pp.pose.position.y += utm_y_zero_;
      out_msg.poses.push_back(pp);
    }
    local_pub_->publish(out_msg);
  }

  // 가장 가까운 점 선택
  std::vector<geometry_msgs::msg::PoseStamped>
  findClosestPoints(const nav_msgs::msg::Path &path,
                    const geometry_msgs::msg::PoseStamped &cur,
                    size_t forward, size_t backward)
  {
    std::vector<std::pair<double, size_t>> dist_idx;
    dist_idx.reserve(path.poses.size());
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto &p = path.poses[i].pose;
      double d = euclidean(cur.pose, p);
      dist_idx.emplace_back(d, i);
    }
    std::sort(dist_idx.begin(), dist_idx.end(),
              [](auto &a, auto &b){ return a.first < b.first; });
    size_t idx = dist_idx.front().second;
    size_t start = (idx + 2 >= backward ? idx + 2 - backward : 0);
    size_t end = std::min(idx + forward, path.poses.size());
    std::vector<geometry_msgs::msg::PoseStamped> sel;
    for (size_t i = start; i < end; ++i) sel.push_back(path.poses[i]);
    if (sel.size() < 3 && path.poses.size() >= 2) {
      sel.clear();
      sel.push_back(path.poses[path.poses.size()-2]);
      sel.push_back(path.poses.back());
    }
    return sel;
  }

  double euclidean(const geometry_msgs::msg::Pose &a,
                   const geometry_msgs::msg::Pose &b)
  {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  // B-spline 보간 (3차)
  std::vector<geometry_msgs::msg::PoseStamped>
  bsplineInterpolation(const std::vector<geometry_msgs::msg::PoseStamped> &pts)
  {
    if (pts.size() < 4) return pts;
    auto b0 = [](double t){ return (1-t)*(1-t)*(1-t)/6.0; };
    auto b1 = [](double t){ return (3*t*t*t - 6*t*t + 4)/6.0; };
    auto b2 = [](double t){ return (-3*t*t*t + 3*t*t + 3*t + 1)/6.0; };
    auto b3 = [](double t){ return t*t*t/6.0; };
    std::vector<geometry_msgs::msg::PoseStamped> res;
    for (size_t i = 0; i+3 < pts.size(); ++i) {
      for (int j = 0; j < 20; ++j) {
        double t = double(j)/20.0;
        double x = b0(t)*pts[i].pose.position.x + b1(t)*pts[i+1].pose.position.x
                 + b2(t)*pts[i+2].pose.position.x + b3(t)*pts[i+3].pose.position.x;
        double y = b0(t)*pts[i].pose.position.y + b1(t)*pts[i+1].pose.position.y
                 + b2(t)*pts[i+2].pose.position.y + b3(t)*pts[i+3].pose.position.y;
        geometry_msgs::msg::PoseStamped p;
        p.header = pts[i].header;
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.position.z = 0.0;
        if (!res.empty()) {
          auto &prev = res.back().pose.position;
          double dx = x - prev.x, dy = y - prev.y;
          if (std::hypot(dx, dy) < 1e-6) continue;
          double yaw = std::atan2(dy, dx);
          tf2::Quaternion q; q.setRPY(0,0,yaw);
          p.pose.orientation = tf2::toMsg(q);
        } else {
          double dx = pts[i+1].pose.position.x - pts[i].pose.position.x;
          double dy = pts[i+1].pose.position.y - pts[i].pose.position.y;
          double yaw = std::atan2(dy, dx);
          tf2::Quaternion q; q.setRPY(0,0,yaw);
          p.pose.orientation = tf2::toMsg(q);
        }
        res.push_back(p);
      }
    }
    res.push_back(pts.back());
    return res;
  }

  // 선형 재샘플링
  std::vector<geometry_msgs::msg::PoseStamped>
  resamplePath(const std::vector<geometry_msgs::msg::PoseStamped> &path,
               double interval)
  {
    if (path.empty()) return {};
    std::vector<double> dists{0.0};
    for (size_t i=1; i<path.size(); ++i)
      dists.push_back(dists.back() + euclidean(path[i-1].pose, path[i].pose));
    double total = dists.back();
    int steps = int(total/interval);
    std::vector<geometry_msgs::msg::PoseStamped> out;
    for (int i=0; i<=steps; ++i) {
      double td = i*interval;
      if (td>total) { out.push_back(path.back()); break; }
      size_t j=0;
      while (j+1<dists.size() && dists[j+1]<td) ++j;
      double seg = dists[j+1]-dists[j];
      double t = (seg<1e-6?0:(td-dists[j])/seg);
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = path[j].pose.position.x + t*(path[j+1].pose.position.x - path[j].pose.position.x);
      p.pose.position.y = path[j].pose.position.y + t*(path[j+1].pose.position.y - path[j].pose.position.y);
      p.pose.position.z = 0.0;
      double dx = path[j+1].pose.position.x - path[j].pose.position.x;
      double dy = path[j+1].pose.position.y - path[j].pose.position.y;
      double yaw = std::atan2(dy, dx);
      tf2::Quaternion q; q.setRPY(0,0,yaw);
      p.pose.orientation = tf2::toMsg(q);
      p.header = path[j].header;
      out.push_back(p);
    }
    return out;
  }

  // 멤버 변수
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_pub_, viz_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path global_path_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_position_{false};
  double utm_x_zero_{0.0}, utm_y_zero_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathProcessor>());
  rclcpp::shutdown();
  return 0;
}