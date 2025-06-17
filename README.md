# figure8

## 🚀 Quick Start: Building and Running

Follow these steps to build the project and launch the `figure8` node.

### 1. Build the Workspace

Navigate to the root of your ROS 2 workspace and build the package using `colcon`.

```bash
colcon build --packages-select figure8
```
> **Note:** If you are building the entire workspace for the first time, you can omit the `--packages-select` argument.

### 2. Source the Environment

After a successful build, source the setup script to make the new executables available in your terminal environment.

```bash
source install/setup.bash
```

### 3. Launch the Node

You are now ready to run the figure-8 trajectory node.

```bash
ros2 run figure8 figure8
```

The drone should now be ready to execute its pre-programmed figure-8 flight pattern.
