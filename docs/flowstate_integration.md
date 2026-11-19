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
