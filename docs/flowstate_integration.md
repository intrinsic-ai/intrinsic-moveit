# Flowstate integration

## Build & Deploy to Flowstate (Sideloading)

For licensed Flowstate users, the following commands build and deploy the services and skills to a Flowstate cluster:

### Environment Setup

Create a `.env` file based on `.env.template` containing target Intrinsic SDK versions, cluster, and organization info:

```bash
cp .env.template .env
# Edit .env with your environment configuration (SDK_VERSION, INTRINSIC_ORGANIZATION, INTRINSIC_CLUSTER, ROS_DISTRO)
```

Download required build tools (`inctl` and `inbuild`):

```bash
make download_inbuild_and_inctl
```

Authenticate with `inctl`:

```bash
export INTRINSIC_ORGANIZATION=...
./bin/inctl auth login --org $INTRINSIC_ORGANIZATION
```

### Build & Deploy Services

Deploy the unified `moveit_planning_service`:

```bash
cd ~/ws_intrinsic_moveit/src/intrinsic-moveit/

# 1. Build & install MoveIt Planning Service bundle
make install_moveit_planning_service

# 2. Add instance of MoveIt Planning Service to active solution
make add_moveit_planning_service
```

> [!NOTE]
> **Upstream Bridge Requirement**: The active cluster solution requires a running `flowstate_ros_bridge` service configured for the workcell. For instructions on applying the configuration for Flowstate (via the Flowstate platform UI) or Intrinsic Core (via `inctl` or Bazel), see [`docs/flowstate_ros_bridge_configuration.md`](./flowstate_ros_bridge_configuration.md).

### Selecting the Workcell Robot Configuration

A single `moveit_planning_service` bundle packages both supported workcell configurations (`omts` with UR5e and `lab_bb_01` with UR3e). At service startup, `service.launch.py` and `moveit_planning_node` resolve `robot_config` from `/etc/intrinsic/runtime_config.pb` (`MoveItPlanningServiceConfig.robot_config`, defaulting to `"omts"`):

* **OMTS (UR5e, default)**: Uses [`moveit_planning_service_default_config.pbtxt`](../moveit_planning_service/moveit_planning_service_default_config.pbtxt) (`robot_config: "omts"`). No extra flags are required when running `make add_moveit_planning_service`.
* **`lab_bb_01` (UR3e)**: Set `robot_config: "lab_bb_01"` on the `moveit_planning_service` instance configuration:
  * **Flowstate UI**: Edit the `moveit_planning_service` instance configuration in your solution and set `robot_config` to `"lab_bb_01"`.
  * **`inctl` / `Makefile` CLI**: Because `inctl service add --config` replaces the full `MoveItPlanningServiceConfig` message rather than merging individual fields, encode the complete reference configuration at [`configs/moveit_planning_service_lab_bb_01_config.pbtxt`](../configs/moveit_planning_service_lab_bb_01_config.pbtxt) (which preserves the in-cluster Ingress and Zenoh DNS endpoints) into a `google.protobuf.Any` binary protobuf (`.binarypb`):

    ```bash
    # Using descriptor sets extracted by `make moveit_planning_service_proto_desc`
    # (or from <ros2-ws>/install/{moveit_planning_service,intrinsic_sdk_cmake}/share/.../*.desc):
    protoc --encode=google.protobuf.Any \
      --descriptor_set_in=./images/moveit_planning_service_protos.desc:./images/intrinsic_proto.desc \
      < ./configs/moveit_planning_service_lab_bb_01_config.pbtxt \
      > ./configs/moveit_planning_service_lab_bb_01_config.binarypb

    # Flowstate cluster (via Makefile):
    make add_moveit_planning_service \
      SERVICE_CONFIG_FLAGS="--config=./configs/moveit_planning_service_lab_bb_01_config.binarypb"

    # Or Intrinsic Core cluster (delete any existing instance first):
    # inctl service delete --address localhost:17080 moveit_planning_service
    # inctl service add --address localhost:17080 ai.intrinsic.moveit_planning_service \
    #   --name=moveit_planning_service \
    #   --config=./configs/moveit_planning_service_lab_bb_01_config.binarypb
    ```

Clean up when done:

```bash
make delete_moveit_planning_service
make uninstall_moveit_planning_service
```

### Build & Deploy Planning Skills

```bash
# Build & install the motion planning skill bundle
make install_moveit_plan_motion_skill

# Build & install the grasp planning skill bundle
make install_moveit_plan_grasp_skill
```

Uninstall skills when done:

```bash
make uninstall_moveit_plan_motion_skill
make uninstall_moveit_plan_grasp_skill
```
