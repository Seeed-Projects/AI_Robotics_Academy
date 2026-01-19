# 1.5 - Common ROS 2 Commands

In a robotic system, there may be several to dozens of nodes running simultaneously. Each node has a unique name and communicates through Topics, Services, Parameters, and more.

ROS 2 provides a unified entry command, `ros2`, to manage the entire system. Common command categories are as follows:

- **ros2 node**: Manage nodes
- **ros2 topic**: Manage topics
- **ros2 service**: Manage services
- **ros2 interface**: Manage interfaces (replaces ROS 1's `rosmsg` and `rossrv`)
- **ros2 param**: Manage parameters
- **ros2 pkg**: Manage packages
- **ros2 bag**: Data recording and playback

---

## 1. ros2 node (Node Management)
`ros2 node` is used to retrieve information about ROS 2 nodes.

- `ros2 node list`: Lists all active nodes.
- `ros2 node info <node_name>`: Prints detailed information about a node (including its subscribed/published topics, provided services, etc.).

**Example:**
```bash
# List all nodes
ros2 node list

# View detailed information of the /turtlesim node
ros2 node info /turtlesim
```

---

## 2. ros2 topic (Topic Management)
`ros2 topic` is the most commonly used debugging tool for inspecting streaming data.

- `ros2 topic list`: Lists currently active topics.
    - Common argument: `-t` (displays message types).
- `ros2 topic echo <topic_name>`: Prints topic data to the screen.
- `ros2 topic info <topic_name>`: Displays detailed information about a topic (type, number of publishers/subscribers).
- `ros2 topic pub <topic_name> <msg_type> <args>`: Manually publishes data to a topic.
- `ros2 topic hz <topic_name>`: Displays the publishing frequency of a topic.
- `ros2 topic bw <topic_name>`: Displays the bandwidth usage of a topic.

**Example:**

1.  **View topic list and types:**
    ```bash
    ros2 topic list -t
    ```

2.  **Manually publish a message:**
    *Note: Publishing messages via the ROS 2 command line requires YAML format, usually wrapped in double quotes with the internal data structure in curly braces.*
    ```bash
    # Format: ros2 topic pub --once <topic> <type> "<data>"
    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
    ```

---

## 3. ros2 interface (Interface Management - Core Change)
In ROS 1, you used `rosmsg` to view `.msg` files and `rossrv` for `.srv` files.
**In ROS 2, these are collectively called "Interfaces" and are managed using the unified `ros2 interface` command.**

- `ros2 interface list`: Lists all available msg, srv, and action types in the system.
- `ros2 interface show <interface_name>`: Displays the definition of an interface (field structure).
- `ros2 interface package <package_name>`: Lists all interfaces under a specific package.

**Example:**

1.  **View the structure of a standard string message:**
    ```bash
    ros2 interface show std_msgs/msg/String
    # Output: string data
    ```

2.  **View the structure of a Twist velocity message:**
    ```bash
    ros2 interface show geometry_msgs/msg/Twist
    ```

---

## 4. ros2 service (Service Management)
`ros2 service` is used to call and inspect "request-response" style services.

- `ros2 service list`: Lists all active services.
    - Common argument: `-t` (displays service types).
- `ros2 service type <service_name>`: Views the type of a service.
- `ros2 service find <type_name>`: Finds all services using a specified type.
- `ros2 service call <service_name> <service_type> <args>`: Manually calls a service.

**Example:**

**Call the turtlesim spawn service:**
```bash
# Format: ros2 service call <service_name> <type> "<data>"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.2, name: 'turtle2'}"
```

---

## 5. ros2 param (Parameter Management)
In ROS 2, parameters belong to specific nodes; there is no global parameter server.

- `ros2 param list`: Lists all nodes and their parameters.
- `ros2 param get <node_name> <param_name>`: Gets the parameter value of a specific node.
- `ros2 param set <node_name> <param_name> <value>`: Dynamically modifies a parameter value.
- `ros2 param dump <node_name>`: Saves a node's parameters to a YAML file.
- `ros2 param load <node_name> <file_path>`: Loads parameters from a YAML file into a node.

**Example:**

1.  **Modify the turtlesim background color:**
    ```bash
    # View parameter list for /turtlesim
    ros2 param list /turtlesim
    
    # Get the current red component of the background
    ros2 param get /turtlesim background_r
    
    # Change it to 150 (the color changes immediately)
    ros2 param set /turtlesim background_r 150
    ```

---

## 6. Other Utility Commands

### ros2 pkg (Package Tools)
- `ros2 pkg create`: Creates a new package.
- `ros2 pkg list`: Lists all installed packages.
- `ros2 pkg executable <package_name>`: Lists the executables (nodes) within a package.

### ros2 doctor (Environment Diagnosis)
This is a unique tool in ROS 2. If the system is not running correctly, you can use it to check your environment configuration.

```bash
ros2 doctor
```
If everything is correct, it will output confirmation messages such as `All required rosdeps installed successfully`.

---

## Summary: ROS 1 vs. ROS 2 Command Comparison

To help you migrate quickly, here is a comparison table of common commands:

| Feature | ROS 1 Command | **ROS 2 Command** |
| :--- | :--- | :--- |
| **Run Node** | `rosrun <pkg> <exe>` | `ros2 run <pkg> <exe>` |
| **Node List** | `rosnode list` | `ros2 node list` |
| **Node Info** | `rosnode info` | `ros2 node info` |
| **Topic List** | `rostopic list` | `ros2 topic list` |
| **Publish Message** | `rostopic pub` | `ros2 topic pub` |
| **View Message** | `rostopic echo` | `ros2 topic echo` |
| **Service List** | `rosservice list` | `ros2 service list` |
| **Call Service** | `rosservice call` | `ros2 service call` |
| **Parameter List** | `rosparam list` | `ros2 param list` |
| **Set Parameter** | `rosparam set` | `ros2 param set` |
| **View Msg** | `rosmsg show` | **`ros2 interface show`** |
| **View Srv** | `rossrv show` | **`ros2 interface show`** |
| **Create Package** | `catkin_create_pkg` | `ros2 pkg create` |