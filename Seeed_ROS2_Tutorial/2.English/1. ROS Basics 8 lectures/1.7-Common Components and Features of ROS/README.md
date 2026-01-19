# 1.7 - Common ROS 2 Components and Features

## ROS 2 Distributed Communication

ROS 2 is designed from the ground up for distributed systems. Unlike ROS 1, ROS 2 removes the centralized Master node (`roscore`) and instead uses middleware based on DDS (Data Distribution Service) to achieve automatic discovery and communication between nodes.

This means: **As long as two computers are on the same local area network (LAN) and are configured with the same Domain ID, they will automatically see each other and communicate.**

### Core Concept: Domain ID

In ROS 1, we decided which network to connect to by specifying the `ROS_MASTER_URI`.
In ROS 2, we use the **`ROS_DOMAIN_ID`**.

*   **Isolation**: `ROS_DOMAIN_ID` is an integer (default is 0). Only nodes with the same ID can communicate with each other.
*   **Use Cases**: If two groups in a lab are debugging different robots under the same Wi-Fi, Group A can set `ID=1` and Group B can set `ID=2`. This completely isolates the data and prevents interference.

### Implementation Steps

#### 1. Preparation

*   **Network Connection**: Ensure all computers (Host PC, Raspberry Pi, Jetson, etc.) are connected to the same router (via Wi-Fi or Ethernet).
*   **Network Settings**: ROS 2 relies heavily on **UDP Multicast**.
    *   If using a Virtual Machine, it must be set to **"Bridged Adapter"** mode. NAT mode usually does not support multi-machine communication.
    *   Check the firewall to ensure UDP communication is allowed.

#### 2. Configure Domain ID

Set the same `ROS_DOMAIN_ID` on all machines that need to communicate.

**On Machine A (e.g., PC):**
Open the terminal or modify `~/.bashrc`:
```bash
# It is recommended to choose an integer between 0 and 101
export ROS_DOMAIN_ID=5
```

**On Machine B (e.g., Robot/Jetson):**
Set the ID to 5 as well:
```bash
export ROS_DOMAIN_ID=5
```

> **Note:** If this variable is not set, ROS 2 defaults to ID `0`. However, in multi-machine environments, it is recommended to explicitly set a non-zero ID to avoid conflicts.

#### 3. Check Localhost Restriction (Common Pitfall)

ROS 2 has a specific environment variable `ROS_LOCALHOST_ONLY`. If it is set to `1`, nodes will only communicate locally and cannot connect to external machines.

**Check on all machines:**
```bash
echo $ROS_LOCALHOST_ONLY
```
If the output is `1`, set it to `0` or unset it in `.bashrc`:
```bash
export ROS_LOCALHOST_ONLY=0
```

### Testing the Setup

ROS 2 provides a very convenient demo package `demo_nodes_cpp` to test communication.

#### 1. Test from Machine A to Machine B

*   **Machine A (Listener):**
    ```bash
    source ~/.bashrc  # Ensure DOMAIN_ID is loaded
    ros2 run demo_nodes_cpp listener
    ```

*   **Machine B (Talker):**
    ```bash
    source ~/.bashrc
    ros2 run demo_nodes_cpp talker
    ```

**Expected Result:** The terminal on Machine A should start printing `I heard: [Hello World: ...]`. If you see this, the distributed communication is working.

#### 2. Verify with CLI Tools

Run the following on any machine:
```bash
ros2 node list
ros2 topic list
```
You should be able to see the nodes and topics running on the other machine.

### Troubleshooting

If both machines are on the same network with the same ID but still cannot communicate, troubleshoot in the following order:

1.  **Firewall**:
    Ubuntu's default firewall might block DDS packets.
    ```bash
    sudo ufw disable
    ```
    *(Or configure ufw to allow UDP ports 7400-7600)*

2.  **Multicast Support**:
    ROS 2 discovery mechanism relies on multicast. Some corporate or campus networks disable multicast.
    **Test if multicast is working:**
    Run `ros2 multicast receive` on one machine and `ros2 multicast send` on the other. If no message is received, the router does not support multicast, and you might need to use the *FastDDS Discovery Server* (advanced configuration).

3.  **Multiple Network Interfaces**:
    If a computer is connected to both Wi-Fi and Ethernet simultaneously, or has a Docker virtual network interface, DDS might select the wrong interface.
    You can bind to a specific interface (like `wlan0` or `eth0`) via an XML configuration file.

### Comparison: ROS 1 vs ROS 2 Distributed Communication

| Feature | ROS 1 | **ROS 2** |
| :--- | :--- | :--- |
| **Core Mechanism** | Master Node (`roscore`) | DDS (Decentralized) |
| **Network Identifier** | `ROS_MASTER_URI` (IP Address) | `ROS_DOMAIN_ID` (Integer) |
| **Dependency** | Master must be started first | Nodes can start anytime; auto-discovery |
| **Address Resolution** | Must modify `/etc/hosts` | Usually not required; relies on UDP discovery |
| **Major Pain Point** | If Master crashes, the whole network fails | Router doesn't support multicast |

---

## ROS 2 Time Synchronization

In a distributed system, the time on multiple machines must be consistent; otherwise, TF coordinate transformations and sensor data fusion will fail (e.g., if a LiDAR data timestamp is 5 seconds later than the current time, the data will be discarded).

**ROS 2 does not automatically synchronize system time**; you need to use operating system-level tools.

### Recommended Solution: Chrony

We install `chrony` on all machines, selecting one as the time server (Master) and others as clients.

1.  **Install Chrony (All machines):**
    ```bash
    sudo apt install chrony
    ```

2.  **Configure the Server (Robot/Main Host):**
    Edit `/etc/chrony/chrony.conf` and add:
    ```text
    allow 192.168.1.0/24  # Allow machines in the LAN to synchronize
    local stratum 10      # Act as a local time source even without internet
    ```
    Restart the service: `sudo service chrony restart`

3.  **Configure Clients (Other computers):**
    Edit `/etc/chrony/chrony.conf`, comment out the `pool ...` lines, and add:
    ```text
    server <Server_IP_Address> minpoll 0 maxpoll 5 iburst
    ```
    Restart the service: `sudo service chrony restart`

4.  **Verification:**
    Run `chronyc sources` on the client. If you see a `*` symbol, the synchronization is successful.