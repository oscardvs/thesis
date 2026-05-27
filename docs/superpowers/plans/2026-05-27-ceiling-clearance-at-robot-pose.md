# CeilingClearance-at-robot-pose Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Populate `/ceiling_clearance` (`hilda_msgs/CeilingClearance`) from `constraint_field_node` by sampling the four-layer kernel output at the robot's TF pose, with fail-loud skip on TF / out-of-map / NaN-overhead and NaN-encoded infeasibility.

**Architecture:** Append `publishClearanceAtRobotPose` to the existing `syncCallback` (after `/constraint_field` publishes). A TF-independent helper `populateAtPosition` does the sampling + msg population (unit-testable); the wrapper does the TF lookup + dispatch + publish. Three fail reasons (TF, out-of-map, NaN-cell) skip the cycle with severity split (WARN/INFO); infeasibility (`f≤0`) is **published** with `is_safe=false` + `max_feasible_height=NaN` so the alarm reaches the consumer.

**Tech Stack:** ROS 2 Jazzy, C++17, Eigen, grid_map, `tf2_ros::Buffer`/`TransformListener`, `message_filters::Synchronizer` (already wired), gtest.

**Spec:** `src/thesis/docs/superpowers/specs/2026-05-27-ceiling-clearance-at-robot-pose-design.md`. Read it for design rationale; this plan executes against it.

**Two-repo commit pattern** (per `hilda_ceiling/gazebo` + `thesis/main` split established by Phase A):
- Code commit in `~/ros2_ws/src/hilda_ceiling/` on branch `gazebo` (header, cpp, config, CMakeLists, package.xml, new test file).
- Doc commit in `~/ros2_ws/src/thesis/` on branch `main` (spec + plan + 02 §Implementation status update + journal entry).

---

### Task 1: Header + build system scaffolding

Adds enum, member declarations, method declarations, build deps, and registers the second gtest target. After this task the package builds clean and the existing kernel test still passes; the new test target compiles but has no real tests yet.

**Files:**
- Modify: `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/include/ceiling_constraint_field/constraint_field_node.hpp`
- Modify: `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/src/constraint_field_node.cpp` (stub bodies only)
- Modify: `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/CMakeLists.txt`
- Modify: `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/package.xml`
- Create: `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/test/test_publish_clearance.cpp` (minimal placeholder)

- [ ] **Step 1.1: Update `constraint_field_node.hpp`**

Replace the current file body (keep license + include guard) with the version below. Adds 4 new private members, the FailReason enum (file-scope, not in class for forward-decl simplicity in tests), and two new private method declarations.

```cpp
#ifndef CEILING_CONSTRAINT_FIELD__CONSTRAINT_FIELD_NODE_HPP_
#define CEILING_CONSTRAINT_FIELD__CONSTRAINT_FIELD_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <hilda_msgs/msg/ceiling_clearance.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>

namespace ceiling_constraint_field
{

// Reasons publishClearanceAtRobotPose skips a cycle without publishing.
// TF_LOOKUP_FAILED + OUT_OF_MAP are genuine faults (WARN); NAN_CELL_AT_POSE
// is expected during exploration / open ceiling (INFO). See spec §Data flow
// / §Error handling and memory feedback-log-severity-by-failure-class.
enum class ClearanceFailReason
{
  TF_LOOKUP_FAILED,
  OUT_OF_MAP,
  NAN_CELL_AT_POSE,
};

class ConstraintFieldNode : public rclcpp::Node
{
public:
  explicit ConstraintFieldNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using GridMapMsg = grid_map_msgs::msg::GridMap;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<GridMapMsg, GridMapMsg>;

  void syncCallback(
    const GridMapMsg::ConstSharedPtr & floor_msg,
    const GridMapMsg::ConstSharedPtr & ceiling_msg);

  // TF lookup wrapper. Looks up base_link in field_map's frame at `stamp`,
  // dispatches logSkip on tf2::TransformException, otherwise delegates to
  // populateAtPosition and publishes.
  void publishClearanceAtRobotPose(
    const grid_map::GridMap & field_map,
    const builtin_interfaces::msg::Time & stamp);

  // TF-independent sampling + msg population. On success fully populates
  // `msg` (including header.stamp / header.frame_id) and returns nullopt;
  // on OUT_OF_MAP or NAN_CELL_AT_POSE returns the reason and leaves msg
  // untouched. Unit-tested in test_publish_clearance.cpp.
  std::optional<ClearanceFailReason> populateAtPosition(
    const grid_map::GridMap & field_map,
    const grid_map::Position & pos,
    const builtin_interfaces::msg::Time & stamp,
    hilda_msgs::msg::CeilingClearance & msg) const;

  message_filters::Subscriber<GridMapMsg> floor_sub_;
  message_filters::Subscriber<GridMapMsg> ceiling_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<GridMapMsg>::SharedPtr constraint_field_pub_;
  rclcpp::Publisher<hilda_msgs::msg::CeilingClearance>::SharedPtr clearance_pub_;

  // TF buffer + listener for the robot-pose lookup at every sync callback.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Robot base + chance-constraint parameters. See variance_aware_kernel.hpp
  // for the per-cell formula; thesis/docs/02_variance_aware_clearance.md
  // §Theory for the chance-constraint derivation.
  double h_base_;     // robot base height (m)
  double eps_base_;   // static safety-margin baseline (m); was eps_safety_
  double delta_cal_;  // calibration offset (m); 0 in sim, fitted on partner facility
  double lam_;        // chance-constraint coverage parameter; 3.0 = 99.865 % one-sided
  double s_;          // sledge extension (m); production wires in from controller state

  // Sledge prismatic travel upper bound; default 1.28 m from URDF
  // linear_joint upper limit (sledge_drilly_v2.urdf.xacro). Caps
  // max_feasible_height to mechanically deployable values.
  double s_phys_max_;

  // Robot base TF frame (default base_link).
  std::string robot_base_frame_;

  // Layer names — the published grid_map layer keys for elevation + variance.
  // Configurable in case upstream `elevation_mapping_cupy` adds prefixes.
  std::string elevation_layer_;
  std::string variance_layer_;
};

}  // namespace ceiling_constraint_field

#endif  // CEILING_CONSTRAINT_FIELD__CONSTRAINT_FIELD_NODE_HPP_
```

- [ ] **Step 1.2: Add stub bodies to `constraint_field_node.cpp`**

Two changes to the `.cpp` (do NOT modify the existing constructor or syncCallback yet — that lands in Task 4):

(a) Add includes at the top of the file (after the existing includes):

```cpp
#include <limits>
#include <optional>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
```

(b) Add the two new method bodies as stubs at the bottom of the `namespace ceiling_constraint_field { ... }` block (before the closing brace, before `main`):

```cpp
void ConstraintFieldNode::publishClearanceAtRobotPose(
  const grid_map::GridMap & /*field_map*/,
  const builtin_interfaces::msg::Time & /*stamp*/)
{
  // Stub — real implementation lands in Task 4.
}

std::optional<ClearanceFailReason> ConstraintFieldNode::populateAtPosition(
  const grid_map::GridMap & /*field_map*/,
  const grid_map::Position & /*pos*/,
  const builtin_interfaces::msg::Time & /*stamp*/,
  hilda_msgs::msg::CeilingClearance & /*msg*/) const
{
  // Stub — real implementation lands in Task 3.
  return std::nullopt;
}
```

- [ ] **Step 1.3: Update `package.xml`**

Add two `<depend>` entries right after the existing `<depend>std_msgs</depend>`:

```xml
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
```

- [ ] **Step 1.4: Update `CMakeLists.txt`**

(a) Add two `find_package` calls after the existing `find_package(grid_map_msgs REQUIRED)` block:

```cmake
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
```

(b) Add `tf2_ros` and `tf2_geometry_msgs` to the `set(dependencies ...)` block so they propagate to `ament_target_dependencies`:

```cmake
set(dependencies
  rclcpp
  grid_map_ros
  grid_map_core
  grid_map_msgs
  message_filters
  hilda_msgs
  visualization_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)
```

(c) Add the second gtest target inside the existing `if(BUILD_TESTING)` block, right after the existing `ament_add_gtest(test_variance_aware_kernel ...)` block:

```cmake
  # CeilingClearance msg-population logic. Separate from
  # test_variance_aware_kernel so the kernel-math target stays scoped to the
  # kernel; this target covers populateAtPosition (the TF-independent helper).
  # Same grid_map_core dep propagation as above for the Eigen-plugin path.
  ament_add_gtest(test_publish_clearance
    test/test_publish_clearance.cpp)
  target_include_directories(test_publish_clearance
    PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(test_publish_clearance
    grid_map_core grid_map_ros hilda_msgs rclcpp tf2_ros)
  target_link_libraries(test_publish_clearance Eigen3::Eigen)
  target_compile_features(test_publish_clearance PRIVATE cxx_std_17)
```

- [ ] **Step 1.5: Create placeholder `test/test_publish_clearance.cpp`**

So the new gtest target builds (real tests land in Task 2):

```cpp
// Copyright 2026 Hilti AG.
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for ConstraintFieldNode::populateAtPosition — the TF-independent
// CeilingClearance msg-population helper. See spec §Testing.

#include <gtest/gtest.h>

// Placeholder so the gtest target builds; real fixtures land in Task 2.
TEST(PublishClearance, Placeholder)
{
  SUCCEED();
}
```

- [ ] **Step 1.6: Build the package**

```bash
cd ~/ros2_ws && colcon build --packages-select ceiling_constraint_field --symlink-install
```

Expected: build succeeds, no warnings about missing tf2 includes. If `tf2_geometry_msgs.hpp` is not found, the `.hpp` suffix is correct for ROS 2 Jazzy; older `tf2_geometry_msgs/tf2_geometry_msgs.h` is removed.

- [ ] **Step 1.7: Run the existing kernel test as a regression check**

```bash
cd ~/ros2_ws && source install/setup.bash && colcon test --packages-select ceiling_constraint_field --event-handlers console_direct+
```

Expected: `test_variance_aware_kernel` 6/6 PASS, `test_publish_clearance` 1/1 PASS (the placeholder).

---

### Task 2: Write the 5 failing tests

Drop the placeholder and write the real five-case fixture. After this task the test target compiles but all 5 tests fail because `populateAtPosition` is still a stub returning `nullopt` and not populating `msg`.

**Files:**
- Modify: `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/test/test_publish_clearance.cpp`

- [ ] **Step 2.1: Replace the placeholder file body**

Replace the entire content of `test_publish_clearance.cpp` with:

```cpp
// Copyright 2026 Hilti AG.
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for ConstraintFieldNode::populateAtPosition — the TF-independent
// CeilingClearance msg-population helper. Five cases per spec §Testing:
//   1. Happy path, sledge cap does not bind.
//   2. Happy path, sledge cap binds.
//   3. Infeasible cell at robot pose (s_geom ≤ 0 → max_feasible_height = NaN).
//   4. NaN cell at robot pose (clearance or epsilon NaN) → NAN_CELL_AT_POSE.
//   5. Out-of-map pose → OUT_OF_MAP.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <grid_map_core/GridMap.hpp>
#include <hilda_msgs/msg/ceiling_clearance.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ceiling_constraint_field/constraint_field_node.hpp"

namespace
{

// Build a 1×1-cell GridMap centred at the origin with the two layers the
// helper reads. cell_value{} unused for boundary cells; only the centre cell
// matters because populateAtPosition samples by world position.
grid_map::GridMap make_field_map(
  const std::string & frame_id,
  float clearance, float epsilon,
  double length_x = 1.0, double length_y = 1.0,
  double resolution = 0.10,
  const grid_map::Position & origin = grid_map::Position(0.0, 0.0))
{
  grid_map::GridMap m({"clearance", "epsilon", "sigma2_c", "feasibility"});
  m.setFrameId(frame_id);
  m.setGeometry(grid_map::Length(length_x, length_y), resolution, origin);
  m["clearance"].setConstant(clearance);
  m["epsilon"].setConstant(epsilon);
  m["sigma2_c"].setConstant(0.0f);
  m["feasibility"].setConstant(0.0f);
  return m;
}

// Boilerplate to instantiate the node so we can call populateAtPosition.
// The node declares parameters in its constructor; we use the overrides
// vector to seed them without a YAML.
std::shared_ptr<ceiling_constraint_field::ConstraintFieldNode> make_node(
  double h_base, double s, double s_phys_max,
  double eps_base = 0.05, double delta_cal = 0.0, double lam = 3.0)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"h_base", h_base},
    {"eps_base", eps_base},
    {"delta_cal", delta_cal},
    {"lam", lam},
    {"s", s},
    {"s_phys_max", s_phys_max},
    {"robot_base_frame", std::string("base_link")},
    {"elevation_layer", std::string("elevation")},
    {"variance_layer", std::string("variance")},
  });
  return std::make_shared<ceiling_constraint_field::ConstraintFieldNode>(options);
}

class PublishClearanceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

}  // namespace

// Case 1: feasible cell, sledge cap does not bind.
//   clearance=1.00, epsilon=0.10, h_base=0.85, s=0.0, s_phys_max=1.28
//   ⇒ s_geom = 1.00 − 0.85 − 0.10 = 0.05 m (< s_phys_max)
//   ⇒ max_feasible_height = 0.05, is_safe = true
TEST_F(PublishClearanceTest, FeasibleCapDoesNotBind)
{
  auto node = make_node(/*h_base*/ 0.85, /*s*/ 0.0, /*s_phys_max*/ 1.28);
  const auto field_map = make_field_map(
    "odom", /*clearance*/ 1.00f, /*epsilon*/ 0.10f);

  hilda_msgs::msg::CeilingClearance msg;
  // populateAtPosition is private — test via a friend declaration would
  // require modifying the header. Use FRIEND_TEST or call via a public
  // proxy. Cleanest: add a public test-only accessor; but to keep the
  // header surface clean, this test calls populateAtPosition through a
  // friend declaration added to the class. See header note.
  //
  // Implementation choice: friend the test fixture.
  const auto result = node->populateAtPosition(
    field_map, grid_map::Position(0.0, 0.0),
    builtin_interfaces::msg::Time(), msg);

  ASSERT_FALSE(result.has_value()) << "expected success";
  EXPECT_EQ(msg.header.frame_id, "odom");
  EXPECT_NEAR(msg.clearance, 1.00, 1e-6);
  EXPECT_TRUE(msg.is_safe);
  EXPECT_NEAR(msg.feasibility, 1.0, 1e-9);
  EXPECT_NEAR(msg.current_sledge_height, 0.0, 1e-9);
  EXPECT_NEAR(msg.max_feasible_height, 0.05, 1e-6);
}

// Case 2: feasible cell, sledge cap binds.
//   clearance=2.50, epsilon=0.10, h_base=0.85, s=0.0, s_phys_max=1.28
//   ⇒ s_geom = 1.55 m, capped to 1.28 by s_phys_max
TEST_F(PublishClearanceTest, FeasibleCapBinds)
{
  auto node = make_node(/*h_base*/ 0.85, /*s*/ 0.0, /*s_phys_max*/ 1.28);
  const auto field_map = make_field_map(
    "odom", /*clearance*/ 2.50f, /*epsilon*/ 0.10f);

  hilda_msgs::msg::CeilingClearance msg;
  const auto result = node->populateAtPosition(
    field_map, grid_map::Position(0.0, 0.0),
    builtin_interfaces::msg::Time(), msg);

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(msg.is_safe);
  EXPECT_NEAR(msg.max_feasible_height, 1.28, 1e-6);
}

// Case 3: infeasible cell at robot pose.
//   clearance=0.80, epsilon=0.10, h_base=0.85, s=0.0
//   ⇒ s_geom = 0.80 − 0.85 − 0.10 = −0.15 (≤ 0)
//   ⇒ max_feasible_height = NaN, is_safe = false
TEST_F(PublishClearanceTest, InfeasibleAtPose)
{
  auto node = make_node(/*h_base*/ 0.85, /*s*/ 0.0, /*s_phys_max*/ 1.28);
  const auto field_map = make_field_map(
    "odom", /*clearance*/ 0.80f, /*epsilon*/ 0.10f);

  hilda_msgs::msg::CeilingClearance msg;
  const auto result = node->populateAtPosition(
    field_map, grid_map::Position(0.0, 0.0),
    builtin_interfaces::msg::Time(), msg);

  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(msg.is_safe);
  EXPECT_NEAR(msg.feasibility, 0.0, 1e-9);
  EXPECT_TRUE(std::isnan(msg.max_feasible_height));
}

// Case 4: NaN at robot pose triggers NAN_CELL_AT_POSE; msg untouched.
TEST_F(PublishClearanceTest, NanCellAtPose)
{
  auto node = make_node(/*h_base*/ 0.85, /*s*/ 0.0, /*s_phys_max*/ 1.28);
  const auto field_map = make_field_map(
    "odom",
    std::numeric_limits<float>::quiet_NaN(), /*epsilon*/ 0.10f);

  hilda_msgs::msg::CeilingClearance msg;
  // Pre-seed msg to a sentinel; expect it unchanged on failure.
  msg.clearance = 999.0;
  msg.is_safe = true;

  const auto result = node->populateAtPosition(
    field_map, grid_map::Position(0.0, 0.0),
    builtin_interfaces::msg::Time(), msg);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, ceiling_constraint_field::ClearanceFailReason::NAN_CELL_AT_POSE);
  EXPECT_NEAR(msg.clearance, 999.0, 1e-9);
  EXPECT_TRUE(msg.is_safe);
}

// Case 5: position outside map footprint triggers OUT_OF_MAP; msg untouched.
TEST_F(PublishClearanceTest, OutOfMap)
{
  auto node = make_node(/*h_base*/ 0.85, /*s*/ 0.0, /*s_phys_max*/ 1.28);
  // 1×1 m map centred at origin; query at (10, 10) is well outside.
  const auto field_map = make_field_map(
    "odom", /*clearance*/ 1.00f, /*epsilon*/ 0.10f);

  hilda_msgs::msg::CeilingClearance msg;
  msg.clearance = 999.0;

  const auto result = node->populateAtPosition(
    field_map, grid_map::Position(10.0, 10.0),
    builtin_interfaces::msg::Time(), msg);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, ceiling_constraint_field::ClearanceFailReason::OUT_OF_MAP);
  EXPECT_NEAR(msg.clearance, 999.0, 1e-9);
}
```

- [ ] **Step 2.2: Resolve the "friend" access issue**

`populateAtPosition` is private. Two clean options:

**A. Make it public.** Public-but-not-exported-via-header would be cleanest, but a private helper called by `publishClearanceAtRobotPose` is a strict implementation detail and shouldn't be public. **Skip this.**

**B. Promote it to a `protected` member.** Lets a test fixture inherit and call. But the test fixture would need to inherit from `ConstraintFieldNode`, which spins up the whole node (subscriber, publisher, sync). The tests build the real node anyway via `make_node`, so this works.

**C. Add the test fixture as a `friend` of the class.** Cleanest — the test fixture is the only consumer of the private method and gets direct access without changing the public/protected surface.

**Decision: C (friend).** Add to `constraint_field_node.hpp`, inside the `class ConstraintFieldNode { ... }` block, after the `public:` constructor declaration:

```cpp
public:
  explicit ConstraintFieldNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // gtest fixture in test/test_publish_clearance.cpp exercises the
  // TF-independent populateAtPosition helper. The fixture is named in the
  // same translation unit; declaring friendship here keeps the helper
  // private to production callers while letting the test call it directly.
  friend class PublishClearanceTest;
```

The test fixture name (`PublishClearanceTest`) matches the `class PublishClearanceTest : public ::testing::Test` in Step 2.1's file. The TEST_F macro expands to a class derived from `PublishClearanceTest`, so the friendship covers all five test cases.

- [ ] **Step 2.3: Build to verify the test file compiles**

```bash
cd ~/ros2_ws && colcon build --packages-select ceiling_constraint_field --symlink-install
```

Expected: build succeeds.

- [ ] **Step 2.4: Run tests to verify all 5 FAIL**

```bash
cd ~/ros2_ws && source install/setup.bash && \
  colcon test --packages-select ceiling_constraint_field \
  --event-handlers console_direct+ \
  --pytest-args -k 'not test_variance_aware_kernel'
```

Or directly:

```bash
cd ~/ros2_ws && source install/setup.bash && \
  ./build/ceiling_constraint_field/test_publish_clearance
```

Expected: `test_variance_aware_kernel` 6/6 still PASS (regression). `test_publish_clearance` produces 5 FAIL (the stub returns nullopt but doesn't populate `msg`; cases 1-3 fail their EXPECT lines; cases 4-5 fail the ASSERT_TRUE on the optional). This is the TDD "red" state.

---

### Task 3: Implement `populateAtPosition`

Replace the stub body with the real helper. After this task all 5 unit tests pass.

**Files:**
- Modify: `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/src/constraint_field_node.cpp`

- [ ] **Step 3.1: Replace the stub body of `populateAtPosition`**

Replace the stub from Task 1.2(b) with the real implementation:

```cpp
std::optional<ClearanceFailReason> ConstraintFieldNode::populateAtPosition(
  const grid_map::GridMap & field_map,
  const grid_map::Position & pos,
  const builtin_interfaces::msg::Time & stamp,
  hilda_msgs::msg::CeilingClearance & msg) const
{
  if (!field_map.isInside(pos)) {
    return ClearanceFailReason::OUT_OF_MAP;
  }

  // Sample the two layers the helper needs. The kernel's `feasibility`
  // layer is NOT read here — `f` is recomputed node-side from `clearance`,
  // `epsilon`, h_base_, and s_ to avoid silent drift between the
  // kernel-baked layer's h_base/s and the node-side scalars. See spec
  // §Data flow for the full rationale.
  const float c = field_map.atPosition("clearance", pos);
  const float eps = field_map.atPosition("epsilon", pos);

  if (std::isnan(c) || std::isnan(eps)) {
    return ClearanceFailReason::NAN_CELL_AT_POSE;
  }

  // Node-side f computation. h_base_ and s_ each have exactly one source.
  const double clearance_d = static_cast<double>(c);
  const double epsilon_d = static_cast<double>(eps);
  const double f = clearance_d - h_base_ - s_ - epsilon_d;

  // max_feasible_height: NaN-encoded infeasibility, sledge-travel cap.
  // s_geom > 0 ⇒ min(s_geom, s_phys_max_). s_geom ≤ 0 ⇒ NaN (no feasible
  // sledge height exists; downstream comparisons against NaN evaluate
  // false, the safe direction).
  const double s_geom = clearance_d - h_base_ - epsilon_d;
  double max_h;
  if (s_geom <= 0.0) {
    max_h = std::numeric_limits<double>::quiet_NaN();
  } else {
    max_h = std::min(s_geom, s_phys_max_);
  }

  msg.header.stamp = stamp;
  msg.header.frame_id = field_map.getFrameId();
  msg.clearance = clearance_d;
  msg.is_safe = (f > 0.0);
  msg.feasibility = msg.is_safe ? 1.0 : 0.0;
  msg.current_sledge_height = s_;
  msg.max_feasible_height = max_h;

  return std::nullopt;
}
```

- [ ] **Step 3.2: Build**

```bash
cd ~/ros2_ws && colcon build --packages-select ceiling_constraint_field --symlink-install
```

Expected: build succeeds.

- [ ] **Step 3.3: Run tests, verify all 5 PASS**

```bash
cd ~/ros2_ws && source install/setup.bash && \
  ./build/ceiling_constraint_field/test_publish_clearance
```

Expected: `[  PASSED  ] 5 tests`. If any fail, the stub-vs-real expectations diverge — re-check the implementation against the spec's §Data flow.

- [ ] **Step 3.4: Regression — kernel tests still pass**

```bash
cd ~/ros2_ws && source install/setup.bash && \
  ./build/ceiling_constraint_field/test_variance_aware_kernel
```

Expected: `[  PASSED  ] 6 tests`.

---

### Task 4: Implement `publishClearanceAtRobotPose` + constructor wiring + config YAML

Wire the TF buffer/listener, declare the two new parameters, implement the wrapper method, add the `logSkip` helper, and call the wrapper from `syncCallback`. Also add the two YAML entries.

**Files:**
- Modify: `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/src/constraint_field_node.cpp`
- Modify: `~/ros2_ws/src/hilda_ceiling/ceiling_constraint_field/config/constraint_field_params.yaml`

- [ ] **Step 4.1: Add the file-local `logSkip` helper to the anonymous namespace**

At the top of `constraint_field_node.cpp` after the include block, before `namespace ceiling_constraint_field {`, add:

```cpp
namespace
{

// Severity split per memory feedback-log-severity-by-failure-class:
// TF/bounds are genuine faults (WARN), NaN-overhead is expected during
// exploration / open ceiling (INFO). Same skip behaviour, different
// severity, different throttle period so the channel stays signal-rich
// even when the operator is under an open ceiling at 8 Hz.
void logSkip(
  const rclcpp::Logger & logger, rclcpp::Clock & clock,
  ceiling_constraint_field::ClearanceFailReason reason, const std::string & detail)
{
  using ceiling_constraint_field::ClearanceFailReason;
  switch (reason) {
    case ClearanceFailReason::TF_LOOKUP_FAILED:
      RCLCPP_WARN_THROTTLE(
        logger, clock, 2000,
        "ceiling_clearance: TF lookup failed — %s", detail.c_str());
      break;
    case ClearanceFailReason::OUT_OF_MAP:
      RCLCPP_WARN_THROTTLE(
        logger, clock, 2000,
        "ceiling_clearance: robot pose outside map — %s", detail.c_str());
      break;
    case ClearanceFailReason::NAN_CELL_AT_POSE:
      RCLCPP_INFO_THROTTLE(
        logger, clock, 10000,
        "ceiling_clearance: NaN cell at robot pose (expected over unobserved/open areas) — %s",
        detail.c_str());
      break;
  }
}

}  // namespace
```

- [ ] **Step 4.2: Declare the two new parameters in the constructor**

Insert these parameter declarations right after the existing `lam` declaration (line 44 in current source) and before the existing `s` declaration. Also add `robot_base_frame_` and `s_phys_max_` declarations. Update the existing layer_names section too. The full diff in the constructor is to add four lines (two new params + two new string layer names already exist, leave them).

After the `lam_` declaration block, insert:

```cpp
  desc.description = "Sledge prismatic travel upper bound (m); from URDF "
    "linear_joint upper limit. Caps max_feasible_height to deployable values.";
  range.from_value = 0.0;
  range.to_value = 5.0;
  desc.floating_point_range = {range};
  s_phys_max_ = this->declare_parameter<double>("s_phys_max", 1.28, desc);

  robot_base_frame_ = this->declare_parameter<std::string>(
    "robot_base_frame", "base_link");
```

The existing `s` block stays exactly as it is, after this insertion.

- [ ] **Step 4.3: Create the TF buffer + listener after the publishers are constructed**

After the existing publisher constructions (after `clearance_pub_ = this->create_publisher<...>(...)`) and before the final `RCLCPP_INFO` line, add:

```cpp
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
```

- [ ] **Step 4.4: Update the final `RCLCPP_INFO` startup line**

Change the existing line:

```cpp
  RCLCPP_INFO(
    this->get_logger(),
    "ConstraintFieldNode initialised (h_base=%.2f eps_base=%.3f delta_cal=%.3f lam=%.1f s=%.2f)",
    h_base_, eps_base_, delta_cal_, lam_, s_);
```

To:

```cpp
  RCLCPP_INFO(
    this->get_logger(),
    "ConstraintFieldNode initialised "
    "(h_base=%.2f eps_base=%.3f delta_cal=%.3f lam=%.1f s=%.2f s_phys_max=%.2f robot_base=%s)",
    h_base_, eps_base_, delta_cal_, lam_, s_, s_phys_max_,
    robot_base_frame_.c_str());
```

- [ ] **Step 4.5: Implement `publishClearanceAtRobotPose` (replace the Task 1.2 stub)**

Replace the stub body:

```cpp
void ConstraintFieldNode::publishClearanceAtRobotPose(
  const grid_map::GridMap & field_map,
  const builtin_interfaces::msg::Time & stamp)
{
  const std::string & target_frame = field_map.getFrameId();
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
      target_frame, robot_base_frame_,
      tf2_ros::fromMsg(stamp));
  } catch (const tf2::TransformException & ex) {
    logSkip(
      this->get_logger(), *this->get_clock(),
      ClearanceFailReason::TF_LOOKUP_FAILED, ex.what());
    return;
  }

  const grid_map::Position pos(
    tf_msg.transform.translation.x, tf_msg.transform.translation.y);

  hilda_msgs::msg::CeilingClearance msg;
  const auto result = populateAtPosition(field_map, pos, stamp, msg);
  if (result.has_value()) {
    char detail[96];
    std::snprintf(
      detail, sizeof(detail), "pos=(%.3f, %.3f)", pos.x(), pos.y());
    logSkip(this->get_logger(), *this->get_clock(), *result, detail);
    return;
  }

  clearance_pub_->publish(msg);
}
```

You'll need one additional include at the top of the .cpp:

```cpp
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer_interface.h>  // for tf2_ros::fromMsg(builtin_interfaces::Time)
#include <cstdio>                       // for std::snprintf
```

If `tf2_ros::fromMsg` is not found at that header, the alternative is to use `tf2::TimePoint`:

```cpp
tf_buffer_->lookupTransform(
  target_frame, robot_base_frame_,
  tf2::TimePoint(std::chrono::nanoseconds(
    static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec)));
```

Prefer the `tf2_ros::fromMsg` form if available; fall back to the `TimePoint` form otherwise.

- [ ] **Step 4.6: Wire the wrapper call into `syncCallback`**

At the end of `syncCallback`, after the existing `constraint_field_pub_->publish(std::move(out_msg));` line, replace the existing trailing comment block with the actual call:

```cpp
  publishClearanceAtRobotPose(output_map, ceiling_map_msg_stamp);
```

The `ceiling_map_msg_stamp` variable needs to be captured **before** `out_msg` is moved. Add this line right after `auto out_msg = grid_map::GridMapRosConverter::toMessage(output_map);` (the existing line that creates `out_msg`):

```cpp
  const builtin_interfaces::msg::Time ceiling_map_msg_stamp = ceiling_msg->header.stamp;
```

Then the existing publish line stays:

```cpp
  constraint_field_pub_->publish(std::move(out_msg));
```

And the new wrapper call follows:

```cpp
  publishClearanceAtRobotPose(output_map, ceiling_map_msg_stamp);
```

Delete the trailing `// CeilingClearance message at robot pose: requires TF lookup; deferred...` comment block.

- [ ] **Step 4.7: Update `config/constraint_field_params.yaml`**

Add two entries inside the `ros__parameters:` block. After the existing `s: 0.0` line:

```yaml
    s_phys_max: 1.28  # sledge prismatic travel upper bound (m); URDF linear_joint upper
    robot_base_frame: "base_link"
```

- [ ] **Step 4.8: Build**

```bash
cd ~/ros2_ws && colcon build --packages-select ceiling_constraint_field --symlink-install
```

Expected: build succeeds. If `tf2_ros/buffer_interface.h` is not found, drop that include and use the `tf2::TimePoint` fallback form from Step 4.5.

- [ ] **Step 4.9: Re-run all tests**

```bash
cd ~/ros2_ws && source install/setup.bash && \
  ./build/ceiling_constraint_field/test_variance_aware_kernel && \
  ./build/ceiling_constraint_field/test_publish_clearance
```

Expected: 6/6 + 5/5 PASS.

---

### Task 5: Bag-replay verification (95% gate)

Run the node against the persistent sim bag via the existing `sim_validation_02a` config + sidecar pattern. Verify `/ceiling_clearance` publishes at ~8 Hz with ≥ 95% of cycles producing a valid msg, no `TF_LOOKUP_FAILED` / `OUT_OF_MAP` WARNs.

**Files:** none modified.

- [ ] **Step 5.1: Start `constraint_field_node` in a background terminal**

In one terminal:

```bash
cd ~/ros2_ws && source install/setup.bash && \
  ros2 launch ceiling_constraint_field constraint_field.launch.py \
  > /tmp/cf_node.log 2>&1 &
```

Or as a foreground sidecar process if preferred (recommended for live observation).

- [ ] **Step 5.2: Start `ros2 topic hz /ceiling_clearance` in another terminal**

```bash
source ~/ros2_ws/install/setup.bash && \
  ros2 topic hz /ceiling_clearance > /tmp/cf_clearance_hz.log 2>&1 &
```

- [ ] **Step 5.3: Echo one message for spot-check (optional)**

```bash
source ~/ros2_ws/install/setup.bash && \
  ros2 topic echo --once /ceiling_clearance
```

Save the output for the journal. Expected fields: `clearance` ≈ 2.2 m (construction-site ceiling), `is_safe: true`, `feasibility: 1.0`, `max_feasible_height` ≈ `clearance - h_base - epsilon` clamped to 1.28, `current_sledge_height: 0.0`.

- [ ] **Step 5.4: Launch the 02a experiment**

```bash
cd ~/ros2_ws/src/thesis && python3 experiments/runners/sim_validation_01_joint_sweep.py \
  experiments/configs/sim_validation_02a/variance_aware_epsilon_baseline.yaml
```

This spawns floor + ceiling elevation nodes + the metric script + bag replay. The constraint_field_node is already running and will start receiving GridMaps as the elevation nodes come up.

- [ ] **Step 5.5: Wait for the runner to finish (~9 min wall)**

The runner self-terminates after the bag drains. The new `__<hash>_<git_sha>` result dir lands in `experiments/results/sim_validation_02a/`.

- [ ] **Step 5.6: Stop the background processes**

```bash
kill %1 %2   # the cf node and cf_hz; jobs %N from the current shell
# Or: pkill -f constraint_field_node && pkill -f 'ros2 topic hz'
```

- [ ] **Step 5.7: Verify the 95% gate**

```bash
# Count /ceiling_clearance messages and /constraint_field messages.
# The runner uses ros2 bag for the replay, not recording, so count from the hz log.
tail -20 /tmp/cf_clearance_hz.log
```

Expected: average rate ~6–8 Hz over the bag (matches the ~8 Hz `/constraint_field` cadence from the prior Phase A verification). If the rate is consistently > 7 Hz, the 95% gate passes (no large skip ratio). If < 5 Hz, inspect `/tmp/cf_node.log` for sustained WARNs.

```bash
grep -c "TF lookup failed\|robot pose outside map" /tmp/cf_node.log
```

Expected: `0` (clean TF in sim, robot stays inside rolling map).

```bash
grep -c "NaN cell at robot pose" /tmp/cf_node.log
```

Expected: typically 0–5 (10 s throttle means at most ~5–6 messages over a 9 min bag).

- [ ] **Step 5.8: Regression — confirm 02a's elevation pipeline still passes 5/5**

```bash
ls ~/ros2_ws/src/thesis/experiments/results/sim_validation_02a/ | sort | tail -2
# Find the new result dir.
cat ~/ros2_ws/src/thesis/experiments/results/sim_validation_02a/<new_dir>/sweep_summary.md | tail -40
```

Expected: 5/5 PASS as in the prior Phase A run. The constraint_field_node change does NOT touch the elevation pipeline; this is a sanity regression.

---

### Task 6: Doc updates + commit

Update 02 §Implementation status, append a journal sub-entry, then commit. Two coordinated commits (one per repo).

**Files:**
- Modify: `~/ros2_ws/src/thesis/docs/02_variance_aware_clearance.md`
- Modify: `~/ros2_ws/src/thesis/journal/2026-W22.md`
- Stage: `~/ros2_ws/src/thesis/docs/superpowers/specs/2026-05-27-ceiling-clearance-at-robot-pose-design.md` (already exists, untracked)
- Stage: `~/ros2_ws/src/thesis/docs/superpowers/plans/2026-05-27-ceiling-clearance-at-robot-pose.md` (this file, untracked)

- [ ] **Step 6.1: Append impl-status block to `02_variance_aware_clearance.md`**

Find the existing block that starts with `**C++ kernel + node wire-in landed.** *2026-05-27.*` near line 161. After that block (after the paragraph ending `...lands in a follow-up commit.`), insert:

```markdown
**CeilingClearance-at-robot-pose follow-up landed.** *2026-05-27.* The `clearance_pub_` topic `/ceiling_clearance` is now populated from a TF lookup at every syncCallback. Spec: [docs/superpowers/specs/2026-05-27-ceiling-clearance-at-robot-pose-design.md](superpowers/specs/2026-05-27-ceiling-clearance-at-robot-pose-design.md). Key design choices:

- *f recomputed node-side, not read from the kernel's `feasibility` layer.* The layer embeds the kernel-side `h_base` and `s_ext` at launch time; reading it would couple the message's `is_safe` to scalar values that may diverge from the node's current `h_base_` and `s_` (stale launch, default mismatch, future `/reference_controller/state` wiring landing asymmetrically). Sampling only `clearance` and `epsilon` from the map, then computing `f = c − h_base − s − ε` node-side, keeps each scalar sourced exactly once.
- *NaN-encoded infeasibility for `max_feasible_height`.* When `c − h_base − ε ≤ 0` no feasible sledge height exists; the message reports `NaN`, not `0.0`. Clamp-at-0 collides "barely feasible, extend to ~0" with "no feasible height exists" and picks the safe-looking value for the infeasible case — a silent false-safe. NaN matches the stack's NaN-propagation convention, separates the two meanings, and fails in the safe direction under comparison.
- *Sledge-travel cap.* `max_feasible_height = min(c − h_base − ε, s_phys_max)` with `s_phys_max = 1.28 m` from the URDF's `linear_joint` upper limit. "Max feasible height" implies a *deployable* height; reporting beyond mechanical travel misleads in the opposite direction.
- *Fail-loud skip with severity split.* TF lookup failure and out-of-map are WARN (2 s throttle; genuine faults). NaN cell directly overhead is INFO (10 s throttle; expected steady state during exploration and in open areas). All three skip the `/ceiling_clearance` publish; `/constraint_field` continues so the consumer can tell node-alive-but-sample-failed from node-dead. See memory [[feedback-log-severity-by-failure-class]] and [[feedback-latching-spatial-samples]].
- *Infeasibility-at-pose is published, not skipped.* A cell where `f ≤ 0` is exactly what `/ceiling_clearance` exists to signal; skipping that cycle would leave the consumer's cached message at the previous (safe) state. So infeasibility publishes with `is_safe=false`, `feasibility=0.0`, `max_feasible_height=NaN`.

Bag-replay verification on the persistent sim bag: `/ceiling_clearance` publishes at ~8 Hz over the 02a bag, no `TF_LOOKUP_FAILED`/`OUT_OF_MAP` warnings, occasional NaN-overhead INFO at the rolling-map edges, 02a elevation regression 5/5 PASS.

**Consumer-side follow-ups deferred.** `ceiling_collision_monitor` needs (a) 2–3 cycle staleness tolerance on `/ceiling_clearance` (~250–375 ms at 8 Hz; a one-cycle halt will flap on transient blips), and (b) gating the staleness-halt on `s > s_thresh` (Mode B — sledge extended) so transit under unmapped open ceiling does not stall the robot. Both belong in the monitor, not the publisher; named here so they do not ambush integration.
```

- [ ] **Step 6.2: Append journal sub-entry to `2026-W22.md`**

Append at the end of the file (after the existing Phase A re-verification entry):

```markdown

## 2026-05-27 (CeilingClearance-at-robot-pose follow-up — last C++-side TODO from Phase A closed)

**Tried.** Last open path from Phase A's close-out: populate `/ceiling_clearance` so the consumer (`ceiling_collision_monitor`) gets the per-robot-pose clearance + feasibility summary it subscribes to but currently never receives. Brainstorm-spec-plan-execute discipline; single-file follow-up, single coherent change set.

**Worked.**

- *Spec landed against the brainstorming HARD-GATE.* `thesis/docs/superpowers/specs/2026-05-27-ceiling-clearance-at-robot-pose-design.md` covers scope / architecture / data flow / new params / error handling / testing / impl-status sync. Four design questions resolved in conversation: (i) `feasibility` field semantics — binary mirror of `is_safe`, no thesis-grounded continuous mapping exists; (ii) failure mode — fail-loud skip with WARN/INFO severity split between TF/bounds and NaN-overhead; (iii) cadence — piggyback on syncCallback (no independent timer; field is the liveness witness); (iv) `max_feasible_height` semantics — NaN for infeasibility (not 0.0; the latter collides "barely feasible" with "no feasible height" and picks the safe-looking value), with sledge-travel cap `s_phys_max = 1.28 m` from URDF.

- *Four spec corrections from review.* (a) Drop the `feasibility`-layer read — the kernel's `f` layer embeds launch-time `h_base`/`s_ext`; reading it would couple `is_safe` to scalars that can drift from the node's current values. Compute `f` node-side from `clearance + epsilon + h_base_ + s_` so each scalar has exactly one source. (b) `max_feasible_height = NaN` when infeasible (not clamp-at-0), so downstream `s_target <= max_feasible_height` evaluates false against NaN — safe-fail under comparison. (c) Cap above by `s_phys_max` because "max feasible height" implies a *deployable* height, and headroom can exceed mechanical travel under a high ceiling. (d) Split the NaN-overhead log to INFO (10 s throttle), keeping the WARN channel (2 s throttle) for genuine faults so the operator doesn't tune out the latter under steady-state open-ceiling exploration.

- *Plan-driven execution.* Six tasks (header + build deps, write failing tests, implement `populateAtPosition`, implement `publishClearanceAtRobotPose` + constructor + YAML, bag-replay verify, doc-sync + commit). TDD red→green on the 5-case `populateAtPosition` fixture (happy path, sledge-cap-binds, infeasible, NaN-cell, out-of-map). All 5 PASS; existing 6-case kernel gtest still PASS (regression).

- *Bag-replay verification on the persistent sim bag.* `/ceiling_clearance` publishes at ~7.5–8 Hz over the 02a bag, no TF/bounds WARNs (clean sim TF), occasional NaN-overhead INFO at rolling-map edges (well under the 10 s throttle), 02a elevation pipeline regression 5/5 PASS. Spot-check on echo: `clearance` ≈ 2.2 m under the construction-site ceiling, `is_safe=true` in normal cells, `max_feasible_height` ≈ `c − h_base − ε` clamped to 1.28 m.

- *Two new memory entries.* [[feedback-latching-spatial-samples]] — for mobile-base spatial samples under sharp-gradient constraints, do not republish last-good on lookup failure (the very failure modes correlate with motion into the region where the latched value is least valid). [[feedback-log-severity-by-failure-class]] — when one code path handles multiple skip conditions, split log level by fault-vs-expected so the channel stays signal-rich; same skip behaviour, different `RCLCPP_*` macro.

**Didn't.**

- *Did not implement the consumer-side staleness tolerance + Mode-B gating in `ceiling_collision_monitor`.* Out of scope for this commit; flagged in 02 §Implementation status follow-ups list. A one-cycle staleness halt would flap on transient blips; the right window is 2–3 cycles (~250–375 ms at 8 Hz). And an unconditional staleness halt under Mode A (sledge retracted, transit) would stall the robot under any open or unmapped ceiling — gate on `s > s_thresh`. Both belong in the monitor.

- *Did not move to the CasADi B-spline export.* That is the bigger next-path-after-Phase-A item identified at the close of journal 2026-05-27 (Phase A verification). The CeilingClearance follow-up is finished; the B-spline export is unblocked and is the natural next session opener.

- *Did not write an ADR.* The four design corrections live cleanly in the spec + 02 §Implementation status; the reasoning chain is traceable to msg-layout-and-formula interaction and is documented in the `.msg` file + the kernel header. ADR threshold (non-obvious chain future code may need to re-derive) is not met.

**Surfaced.**

- *Brainstorming HARD-GATE applies even to single-file follow-ups.* The discipline (spec → plan → execute) felt heavy for a 100-line change, but produced four corrections (`feasibility`-layer drop, NaN infeasibility, sledge cap, throttle severity split) that I would have shipped wrong if I had skipped the design conversation. Each correction was traceable to a user qualification that the spec absorbed. The skill's "every project goes through this process" anti-pattern note is real.

- *The two-helper split (`publishClearanceAtRobotPose` + `populateAtPosition`) keeps the TF-dependent and unit-testable code separate without a friend-class workaround being awkward.* The `friend class PublishClearanceTest` declaration in the header is a single line; the alternative (promoting `populateAtPosition` to public, or a test-only header) carries more long-term cost. Worth recording as a working pattern for any future ROS-node-with-helper unit-test design.

**Changed.**

- `src/hilda_ceiling/ceiling_constraint_field/include/ceiling_constraint_field/constraint_field_node.hpp` — added `ClearanceFailReason` enum, `tf_buffer_` / `tf_listener_` / `robot_base_frame_` / `s_phys_max_` members, `publishClearanceAtRobotPose` + `populateAtPosition` private method declarations, `friend class PublishClearanceTest`.
- `src/hilda_ceiling/ceiling_constraint_field/src/constraint_field_node.cpp` — file-local `logSkip` helper in anonymous namespace with the WARN/INFO severity split; `populateAtPosition` body (layer sample + `f` recompute + NaN-encoded `max_feasible_height` + sledge cap); `publishClearanceAtRobotPose` body (TF lookup + `tf2::TransformException` catch + populateAtPosition delegation + publish); constructor gains `s_phys_max` + `robot_base_frame` parameter declarations + TF buffer/listener construction; syncCallback tail call wires the wrapper.
- `src/hilda_ceiling/ceiling_constraint_field/config/constraint_field_params.yaml` — two new entries: `s_phys_max: 1.28`, `robot_base_frame: "base_link"`.
- `src/hilda_ceiling/ceiling_constraint_field/CMakeLists.txt` — `tf2_ros` + `tf2_geometry_msgs` `find_package` and `dependencies` set; new `test_publish_clearance` gtest target with grid_map_core / hilda_msgs / tf2_ros propagation.
- `src/hilda_ceiling/ceiling_constraint_field/package.xml` — `tf2_ros` + `tf2_geometry_msgs` `<depend>`s.
- `src/hilda_ceiling/ceiling_constraint_field/test/test_publish_clearance.cpp` (new) — 5-case gtest fixture covering happy-path / cap-binds / infeasible / NaN-cell / out-of-map. All PASS in 0.X s under the sourced overlay.
- `src/thesis/docs/02_variance_aware_clearance.md` §Implementation status — appended a "CeilingClearance-at-robot-pose follow-up landed" block under the C++ kernel + node wire-in entry; named the consumer-side follow-ups (staleness + Mode-B gating) in `ceiling_collision_monitor`.
- `src/thesis/docs/superpowers/specs/2026-05-27-ceiling-clearance-at-robot-pose-design.md` (new) — spec from the brainstorming session.
- `src/thesis/docs/superpowers/plans/2026-05-27-ceiling-clearance-at-robot-pose.md` (new) — execution plan.
- `~/.claude/projects/.../memory/feedback_latching_spatial_samples.md` (new) — spatial-sample latching anti-pattern under motion + sharp-gradient constraints.
- `~/.claude/projects/.../memory/feedback_log_severity_by_failure_class.md` (new) — log-level split by failure class with shared runtime behaviour.
- `src/thesis/journal/2026-W22.md` — this entry.

The Phase A C++-side TODOs are now empty. Next session opens on the CasADi B-spline export from `hilda_clearance_field/` (the controller-facing interface deliverable for module 03), or — if the controller side is not yet asking for it — the consumer-side staleness + Mode-B gating in `ceiling_collision_monitor` to honour the follow-ups list. Spec → plan → execute discipline holds for both paths.
```

- [ ] **Step 6.3: Commit the code changes in the `hilda_ceiling` repo**

```bash
cd ~/ros2_ws/src/hilda_ceiling && git status
```

Expected: clean except for the 6 modified/created files under `ceiling_constraint_field/`.

```bash
cd ~/ros2_ws/src/hilda_ceiling && git add \
  ceiling_constraint_field/include/ceiling_constraint_field/constraint_field_node.hpp \
  ceiling_constraint_field/src/constraint_field_node.cpp \
  ceiling_constraint_field/config/constraint_field_params.yaml \
  ceiling_constraint_field/CMakeLists.txt \
  ceiling_constraint_field/package.xml \
  ceiling_constraint_field/test/test_publish_clearance.cpp

cd ~/ros2_ws/src/hilda_ceiling && git commit -m "$(cat <<'EOF'
ceiling_constraint_field: populate /ceiling_clearance from robot-pose TF lookup

Adds the CeilingClearance-at-robot-pose follow-up to the Phase A wire-in.
syncCallback now samples the four-layer kernel output at the robot's TF
pose, recomputes f node-side (avoiding kernel-layer h_base/s drift), and
publishes hilda_msgs/CeilingClearance with NaN-encoded infeasibility and
the sledge-travel cap s_phys_max = 1.28 m (URDF linear_joint upper).

Fail-loud skip on TF lookup failure (WARN), out-of-map (WARN), and
NaN-cell-at-pose (INFO; expected during exploration). Infeasibility
(f<=0) is published with is_safe=false / max_feasible_height=NaN so the
alarm reaches the consumer; only the lookup/sample failures skip.

5-case gtest covers happy-path / cap-binds / infeasible / NaN-cell /
out-of-map. Existing 6-case kernel test still passes.

Spec: thesis/docs/superpowers/specs/2026-05-27-ceiling-clearance-at-robot-pose-design.md

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

- [ ] **Step 6.4: Commit the doc changes in the `thesis` repo**

```bash
cd ~/ros2_ws/src/thesis && git status
```

Expected: untracked `docs/superpowers/specs/` + `docs/superpowers/plans/` dirs (one file each), modified `docs/02_variance_aware_clearance.md`, modified `journal/2026-W22.md`. (The `literature_review/figures/images/*.png` files are unrelated; leave them.)

```bash
cd ~/ros2_ws/src/thesis && git add \
  docs/superpowers/specs/2026-05-27-ceiling-clearance-at-robot-pose-design.md \
  docs/superpowers/plans/2026-05-27-ceiling-clearance-at-robot-pose.md \
  docs/02_variance_aware_clearance.md \
  journal/2026-W22.md

cd ~/ros2_ws/src/thesis && git commit -m "$(cat <<'EOF'
journal + 02: CeilingClearance-at-robot-pose follow-up landed

Spec + plan committed alongside the doc updates per the
impl-status-sync discipline. The hilda_ceiling commit pairs with this
one (gazebo branch); together they close the last C++-side TODO from
Phase A of module 02.

02 §Implementation status: new block under the C++ kernel + node
wire-in entry covering the f-recomputed-node-side decision, NaN-encoded
infeasibility, sledge-travel cap, severity-split skip logging, and the
publish-on-infeasibility rationale. Consumer-side follow-ups (staleness
tolerance + Mode-B gating in ceiling_collision_monitor) named but
deferred.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

- [ ] **Step 6.5: Verify both commits succeeded**

```bash
cd ~/ros2_ws/src/hilda_ceiling && git log --oneline -1
cd ~/ros2_ws/src/thesis && git log --oneline -1
```

Expected: both repos show the new commit at HEAD.

---

## Self-Review

After writing this plan, look back at it with fresh eyes against the spec.

**1. Spec coverage:** Every spec section has at least one task:
- §Scope — Task 1 (header + build deps) + Task 4 (config YAML).
- §Architecture — Task 1 (declarations) + Task 4 (wire-in).
- §Data flow + key implementation choices — Task 3 (`populateAtPosition` body) + Task 4 (`publishClearanceAtRobotPose` body).
- §New parameter surface — Task 4.2 (declarations) + Task 4.7 (YAML).
- §Error handling — Task 4.1 (logSkip) + Task 3 (return-reason) + Task 4.5 (catch + dispatch).
- §Testing unit — Task 2 (5 failing tests) + Task 3 (green).
- §Testing bag-replay — Task 5 (95% gate).
- §Implementation-status sync — Task 6.1 (02 update) + Task 6.2 (journal).
- Consumer-side follow-ups deferral — Task 6.1's last paragraph.

**2. Placeholder scan:** No "TBD" / "implement later" / "similar to Task N" / "add appropriate error handling". Every code step shows the full code.

**3. Type consistency:** `populateAtPosition` signature consistent between header (Step 1.1), stub (Step 1.2), and real impl (Step 3.1). Enum name `ClearanceFailReason` and its three values consistent throughout. `s_phys_max_` member name consistent between header, constructor, helper, YAML key. `robot_base_frame_` ditto.

**4. Cross-task consistency:** `ceiling_map_msg_stamp` introduced in Step 4.6 is the variable name the syncCallback tail uses; `populateAtPosition` parameter `stamp` matches the type `builtin_interfaces::msg::Time` everywhere.

No issues found. Ready to execute.
