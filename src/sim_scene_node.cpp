// Copyright 2026 Duatic AG
// Duatic Commercial License 1.0 (DCL-1)

// Welds Gazebo models together on request, and releases them again.
//
// Gazebo offers no service for this. Its DetachableJoint system has to be declared in
// SDF per pair, and gz-sim 8 has no suppress_initial_attach, so anything declared is
// welded from the moment the world loads and dragged along as the robot drives. This
// node injects the system at call time through the world's entity/system/add service,
// which is what allows arbitrary pairs.
//
// C++ rather than Python because gz-transport has no Python bindings: the alternative
// is shelling out to the `gz` CLI, which costs a process per call and parses
// human-readable output. Everything needed is already available through the ROS
// vendor packages — gz-transport13 and gz-msgs10, including entity_plugin_v.
//
// Welding only — no levelling, deliberately. "Level" cannot be written generically:
// zeroing roll and pitch is what it means for most models, but a mesh whose long axis
// runs along local z needs a 90 degree roll to lie down at all, and zeroing it stands
// the object on its end. Only the caller knows its model's convention.
//
// Simulation only. Nothing above this should depend on it, or that thing will not run
// on hardware; the /sim namespace is meant to make that obvious at a glance, and there
// is deliberately no skill descriptor so the UI never offers it as a building block.

#include <map>
#include <memory>
#include <set>
#include <chrono>
#include <thread>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include "duatic_helper_msgs/srv/attach_model.hpp"
#include "duatic_helper_msgs/srv/detach_model.hpp"

namespace
{
constexpr char kDetachableJointName[] = "gz::sim::systems::DetachableJoint";
constexpr char kDetachableJointFile[] = "gz-sim-detachable-joint-system";
// Gazebo's own services answer in well under a second; this is only a backstop so a
// stalled simulation surfaces as a failed service call rather than a hung one.
constexpr unsigned int kGzTimeoutMs = 5000;
}  // namespace

class SimSceneNode : public rclcpp::Node
{
public:
  SimSceneNode() : rclcpp::Node("sim_scene")
  {
    world_ = declare_parameter<std::string>("world", "bag_sorting");
    robot_ = declare_parameter<std::string>("robot_model", "dxtr");

    attach_srv_ = create_service<duatic_helper_msgs::srv::AttachModel>(
      "~/attach",
      [this](
        const std::shared_ptr<duatic_helper_msgs::srv::AttachModel::Request> req,
        std::shared_ptr<duatic_helper_msgs::srv::AttachModel::Response> res) { onAttach(req, res); });

    detach_srv_ = create_service<duatic_helper_msgs::srv::DetachModel>(
      "~/detach",
      [this](
        const std::shared_ptr<duatic_helper_msgs::srv::DetachModel::Request> req,
        std::shared_ptr<duatic_helper_msgs::srv::DetachModel::Response> res) { onDetach(req, res); });

    RCLCPP_INFO(
      get_logger(), "Ready for world '%s', default holder '%s' (services under %s/)",
      world_.c_str(), robot_.c_str(), get_name());
  }

private:
  // ── Gazebo lookups ──────────────────────────────────────────────────────

  // Entity id and first link of every model in the scene, static ones included. The
  // pose topics only carry entities that move, so they cannot see the floor — and the
  // floor is a legitimate holder for a placed object.
  bool sceneModels(std::map<std::string, std::pair<uint32_t, std::string>> & out)
  {
    gz::msgs::Empty req;
    gz::msgs::Scene rep;
    bool ok{false};
    if (!gz_.Request("/world/" + world_ + "/scene/info", req, kGzTimeoutMs, rep, ok) || !ok) {
      return false;
    }
    for (const auto & model : rep.model()) {
      std::string first_link = model.link_size() > 0 ? model.link(0).name() : std::string{};
      out[model.name()] = {model.id(), first_link};
      // Nested models can hold things too — a deck that is its own model, say.
      for (const auto & nested : model.model()) {
        std::string nested_link = nested.link_size() > 0 ? nested.link(0).name() : std::string{};
        out[nested.name()] = {nested.id(), nested_link};
      }
    }
    return !out.empty();
  }

  // ── Services ────────────────────────────────────────────────────────────

  void onAttach(
    const std::shared_ptr<duatic_helper_msgs::srv::AttachModel::Request> req,
    std::shared_ptr<duatic_helper_msgs::srv::AttachModel::Response> res)
  {
    const std::string child = req->child_model;
    const std::string holder = req->parent_model.empty() ? robot_ : req->parent_model;
    const std::string tag = req->tag.empty() ? "default" : req->tag;

    if (child.empty()) {
      res->success = false;
      res->message = "child_model must not be empty";
      return;
    }
    if (child == holder) {
      res->success = false;
      res->message = "child_model and parent_model are the same model";
      return;
    }

    // One parent joint per body. A second live coupling is not a second constraint but
    // a closed loop — base_link -> load -> gripper -> arm -> base_link — which the
    // physics engine resolves by having the two ends fight, so a joint ends far from its
    // command and the controller aborts. Refused rather than warned: the caller cannot
    // see that consequence, and the remedy is always to release the other coupling.
    for (const auto & [other_tag, models] : live_) {
      if (other_tag != tag && models.count(child)) {
        res->success = false;
        res->message =
          "'" + child + "' is already held as '" + other_tag + "'. Two holders close a " +
          "kinematic loop; detach that one first.";
        return;
      }
    }

    // Already injected for this pair: the joint is still there, so re-welding is just
    // a message on its attach topic.
    if (injected_[tag].count(child)) {
      if (!publishOn(attachTopic(tag, child))) {
        res->success = false;
        res->message = "Could not publish on " + attachTopic(tag, child);
        return;
      }
      live_[tag].insert(child);
      res->success = true;
      res->message = "'" + child + "' re-attached as '" + tag + "'";
      return;
    }

    std::map<std::string, std::pair<uint32_t, std::string>> models;
    if (!sceneModels(models)) {
      res->success = false;
      res->message = "Gazebo did not answer /world/" + world_ + "/scene/info";
      return;
    }
    if (!models.count(child)) {
      res->success = false;
      res->message = "No model named '" + child + "' in world '" + world_ + "'";
      return;
    }
    if (!models.count(holder)) {
      res->success = false;
      res->message = "No model named '" + holder + "' in world '" + world_ + "'";
      return;
    }

    const std::string child_link = models[child].second;
    if (child_link.empty()) {
      res->success = false;
      res->message = "Model '" + child + "' reports no link to weld";
      return;
    }
    std::string holder_link = req->parent_link;
    if (holder_link.empty()) {
      holder_link = models[holder].second;
      if (holder_link.empty()) {
        res->success = false;
        res->message = "Model '" + holder + "' reports no link to weld to";
        return;
      }
    }

    // The plugin goes on the HOLDER, whose link is <parent_link>. The child model is
    // the dependent body: its pose is resolved through the joint. Hosting it on the load
    // instead inverts that — commanding the load then moves the holder, however heavy
    // the holder and however light the load.
    const std::string inner =
      "<parent_link>" + holder_link + "</parent_link>" +
      "<child_model>" + child + "</child_model>" +
      "<child_link>" + child_link + "</child_link>" +
      "<attach_topic>" + attachTopic(tag, child) + "</attach_topic>" +
      "<detach_topic>" + detachTopic(tag, child) + "</detach_topic>" +
      "<output_topic>/sim/" + tag + "/" + child + "/state</output_topic>";

    gz::msgs::EntityPlugin_V req_msg;
    req_msg.mutable_entity()->set_id(models[holder].first);
    req_msg.mutable_entity()->set_type(gz::msgs::Entity::MODEL);
    auto * plugin = req_msg.add_plugins();
    plugin->set_name(kDetachableJointName);
    plugin->set_filename(kDetachableJointFile);
    plugin->set_innerxml(inner);

    gz::msgs::Boolean rep;
    bool ok{false};
    if (
      !gz_.Request("/world/" + world_ + "/entity/system/add", req_msg, kGzTimeoutMs, rep, ok) ||
      !ok || !rep.data())
    {
      res->success = false;
      res->message = "Gazebo refused the '" + tag + "' joint for '" + child + "'";
      return;
    }

    // Adding the system attaches it straight away.
    injected_[tag].insert(child);
    live_[tag].insert(child);
    res->success = true;
    res->message =
      "'" + child + "' welded to " + holder + "/" + holder_link + " as '" + tag + "'";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void onDetach(
    const std::shared_ptr<duatic_helper_msgs::srv::DetachModel::Request> req,
    std::shared_ptr<duatic_helper_msgs::srv::DetachModel::Response> res)
  {
    const std::string child = req->child_model;
    if (child.empty()) {
      res->success = false;
      res->message = "child_model must not be empty";
      return;
    }

    std::vector<std::string> tags;
    if (req->tag.empty()) {
      for (const auto & [tag, models] : live_) {
        if (models.count(child)) {
          tags.push_back(tag);
        }
      }
    } else if (live_[req->tag].count(child)) {
      tags.push_back(req->tag);
    }

    if (tags.empty()) {
      // Not an error: releasing something that is not held is what a reset does.
      res->success = true;
      res->message = "'" + child + "' was not held";
      return;
    }

    for (const auto & tag : tags) {
      // live_ is only cleared when the message actually went out. Clearing it on a
      // lost detach is what blinded the loop check.
      if (publishOn(detachTopic(tag, child))) {
        live_[tag].erase(child);
        res->released.push_back(tag);
      }
    }
    res->success = !res->released.empty();
    res->message = res->success
                     ? "'" + child + "' released"
                     : "Could not publish a detach for '" + child + "'";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  // Publish on an attach/detach topic, keeping the publisher alive and waiting for the
  // DetachableJoint plugin to connect first.
  //
  // Advertising and publishing in the same breath loses the message: gz-transport needs
  // a moment to wire publisher to subscriber, and until it has, Publish() succeeds while
  // nothing receives it — so a detach reports success while the load stays welded, and
  // the next attach closes the very loop this node exists to prevent. Publishers are
  // cached because re-advertising the same topic starts the race over.
  bool publishOn(const std::string & topic)
  {
    auto it = pubs_.find(topic);
    if (it == pubs_.end()) {
      auto pub = gz_.Advertise<gz::msgs::Empty>(topic);
      if (!pub) {
        return false;
      }
      it = pubs_.emplace(topic, std::move(pub)).first;
    }
    for (int i = 0; i < 50 && !it->second.HasConnections(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!it->second.HasConnections()) {
      RCLCPP_WARN(get_logger(), "Nothing is listening on %s", topic.c_str());
      return false;
    }
    gz::msgs::Empty msg;
    return it->second.Publish(msg);
  }

  std::string attachTopic(const std::string & tag, const std::string & child) const
  {
    return "/sim/" + tag + "/" + child + "/attach";
  }
  std::string detachTopic(const std::string & tag, const std::string & child) const
  {
    return "/sim/" + tag + "/" + child + "/detach";
  }

  std::string world_;
  std::string robot_;
  gz::transport::Node gz_;
  // Injected: the joint exists in Gazebo, so re-welding needs only the attach topic.
  // Live: it is currently closed. Only the live set can answer "does this body already
  // have a parent", and the two differ because a detached joint stays injected.
  std::map<std::string, gz::transport::Node::Publisher> pubs_;
  std::map<std::string, std::set<std::string>> injected_;
  std::map<std::string, std::set<std::string>> live_;
  rclcpp::Service<duatic_helper_msgs::srv::AttachModel>::SharedPtr attach_srv_;
  rclcpp::Service<duatic_helper_msgs::srv::DetachModel>::SharedPtr detach_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSceneNode>());
  rclcpp::shutdown();
  return 0;
}
