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
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include "duatic_helper_msgs/srv/attach_model.hpp"
#include "duatic_helper_msgs/srv/detach_model.hpp"

namespace
{
constexpr int kDetachAttempts = 3;
constexpr int kDetachWaitTicks = 25;  // x 20 ms
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

    // Already closed under this tag: nothing to do. Anything else gets a FRESH joint,
    // even for a pair that was welded before. A DetachableJoint that has been detached
    // does not weld again when its attach topic is published on: the message goes out,
    // gz-transport still reports a listener, and the load simply stays where it is. So
    // every attach injects its own plugin instance, which is the path that demonstrably
    // works, and each instance gets its own topics via a per-pair counter.
    if (live_[tag].count(child)) {
      res->success = true;
      res->message = "'" + child + "' is already held as '" + tag + "'";
      RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
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
    // A new instance, so a new pair of topics. The first attach of a pair is instance 0.
    auto & n = instance_[tag + "/" + child];
    if (injected_[tag].count(child)) {
      ++n;
    }

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

    // Adding the system attaches it straight away. Recorded here rather than waited for:
    // the state topic is shared by every instance of a pair, so a detach that arrives before
    // this attach's own "attached" would otherwise read the PREVIOUS cycle's "detached" and
    // confirm a release that never happened.
    watchState(tag, child);
    setState(stateTopic(tag, child), "attached");
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

    std::vector<std::string> stuck;
    for (const auto & tag : tags) {
      // live_ is only cleared when the joint is actually gone.
      if (detachAndConfirm(tag, child)) {
        live_[tag].erase(child);
        res->released.push_back(tag);
      } else {
        stuck.push_back(tag);
      }
    }
    res->success = stuck.empty() && !res->released.empty();
    if (!stuck.empty()) {
      res->message = "'" + child + "' is still held as '" + stuck.front() +
                     "': the joint did not report itself detached";
    } else {
      res->message = res->success
                       ? "'" + child + "' released"
                       : "Could not publish a detach for '" + child + "'";
    }
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  /** Detach, and do not report it until the plugin says it happened.
   *
   * The plugin reports "detached" on its output topic. Publishing on the detach topic only
   * says the request went out, so the answer is what a release is confirmed by.
   */
  bool detachAndConfirm(const std::string & tag, const std::string & child)
  {
    watchState(tag, child);
    const std::string topic = stateTopic(tag, child);
    for (int attempt = 0; attempt < kDetachAttempts; ++attempt) {
      if (!publishOn(detachTopic(tag, child))) {
        return false;
      }
      for (int i = 0; i < kDetachWaitTicks; ++i) {
        if (lastState(topic) == "detached") {
          if (attempt > 0) {
            RCLCPP_WARN(
              get_logger(), "'%s' needed %d detach attempts as '%s'", child.c_str(),
              attempt + 1, tag.c_str());
          }
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    RCLCPP_ERROR(
      get_logger(), "'%s' did not report itself detached as '%s' after %d attempts",
      child.c_str(), tag.c_str(), kDetachAttempts);
    return false;
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

  // One topic pair per injected instance. Reusing a topic would address the previous,
  // already detached joint, which no longer welds.
  std::string instanceBase(const std::string & tag, const std::string & child) const
  {
    auto it = instance_.find(tag + "/" + child);
    const int n = it == instance_.end() ? 0 : it->second;
    return "/sim/" + tag + "/" + child + "/" + std::to_string(n);
  }
  std::string attachTopic(const std::string & tag, const std::string & child) const
  {
    return instanceBase(tag, child) + "/attach";
  }
  /** Shared by every instance of a pair: the plugin reports the state, not which plugin. */
  std::string stateTopic(const std::string & tag, const std::string & child) const
  {
    return "/sim/" + tag + "/" + child + "/state";
  }

  std::string detachTopic(const std::string & tag, const std::string & child) const
  {
    return instanceBase(tag, child) + "/detach";
  }

  std::string world_;
  std::string robot_;
  /** What the plugin last said about a pair, from its <output_topic>: "attached" or
   *  "detached". The only account of the joint that does not come from our own bookkeeping. */
  std::string lastState(const std::string & topic)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = state_.find(topic);
    return it == state_.end() ? std::string{} : it->second;
  }

  void setState(const std::string & topic, const std::string & value)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_[topic] = value;
  }

  void watchState(const std::string & tag, const std::string & child)
  {
    const std::string topic = stateTopic(tag, child);
    if (!watched_.insert(topic).second) {
      return;
    }
    gz_.Subscribe<gz::msgs::StringMsg>(
      topic, [this, topic](const gz::msgs::StringMsg & msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_[topic] = msg.data();
      });
  }

  gz::transport::Node gz_;
  // Injected: a joint for this pair has been created at least once, which is what makes
  // the next attach a new instance rather than the first. Live: one is currently closed —
  // only that set can answer "does this body already have a parent".
  std::map<std::string, gz::transport::Node::Publisher> pubs_;
  std::map<std::string, std::set<std::string>> injected_;
  std::map<std::string, std::set<std::string>> live_;
  // tag/child -> index of the joint instance currently injected for that pair.
  std::map<std::string, int> instance_;
  std::set<std::string> watched_;
  std::map<std::string, std::string> state_;
  std::mutex state_mutex_;
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
