# RMW Zenoh-Pico implementation

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

The rmw_zenoh_pico is the implementation of the rmw layer with zenoh_pico.  
This package is generated using the [Micro-ROS project](https://micro.ros.org/) stack.  
This package provides the same functionality as the [RMW Micro XRCE-DDS implementation](https://github.com/micro-ROS/rmw_microxrcedds).  
This package is able to connect the [rmw_zenoh](https://github.com/ros2/rmw_zenoh) layer, whose implementation is based on Zenoh and is written using the zenoh-c bindings by the ROS community.  

<!--
 rmw_zenoh                rmw_zenoh_pico
 +-------------+          +------------------+
 |   ROS APP   |          |     ROS APP      |
 +-------------+          +------------------+
 |   rclcpp    |          |      rclc        |
 +-------------+          +------------------+
 |     rcl     |          |  rcl by microros |
 +-------------+          +------------------+
 |  rmw_zenoh  |          |  rmw_zenoh_pico  |
 +-------------+          +------------------+
 |   zenoh     +----------+    zenoh pico    |
 +-------------+          +------------------+
 |  Posix API  |          | Posix/RTOS API   |
 +-------------+          +------------------+
 |Linux/Windows|          |    RTOS/Linux    |
 +-------------+          +------------------+
-->

![stack_diag.svg](./design/overview.svg)

## Packages

### What is rmw_zenoh_pico ?

- Implementation of the ROS 2 middleware abstraction interface written in C
- Generate micro-ROS applications using Zenoh-Pico
- A middleware implementation for [Zenoh-Pico](https://github.com/eclipse-zenoh/zenoh-pico) package
- Replacement XRCE-DDS on micro-ROS with Zenoh-Pico
- Wrapper of Zenoh-Pico API and Zenoh-Pico internal utilities(z_sting, z_mutex, etc.)
- Called this reference library implementation for the Zenoh protocol from the upper layers in the ROS 2 stack

### Support rmw Functions

The rmw API for rmw_zenoh_pico is not currently as well supported as rmw_zenoh and rmw_microxrcedds_c.  
This [list](./rmw_zenoh_pico_rmw_list.md) is support of rmw_zenoh_pico.

> [!IMPORTANT]
> The prototype implementation of rmw_zenoh_pico does not yet support graph-cache information (gid).  
> For rmw_zenoh_pico to support gid, several [issues](#known-issueslimitations) should be considered.  
> The rmw_zenoh_pico would like to support zenoh_pico and rmw_zenoh along with future updates from the ROS community.  

### Related repositories

Table: Related repositories

| Repository      | Branch | SID                                        | URL                                               |
|-----------------|--------|--------------------------------------------|---------------------------------------------------|
| micro_ros_setup | jazzy  | `d60bb3ae889d3617a7a408ae78765e472eda7af9` | <https://github.com/micro-ROS/micro_ros_setup>    |
| zenoh-pico      | 1.4.0  | `d08c096944807a00853892ea45696152308350a2` | <https://github.com/eclipse-zenoh/zenoh-pico.git> |
| rmw_zenoh       | jazzy  | `aae224e449f8f364f4a8025fe85899ce06f5381b` | <https://github.com/ros2/rmw_zenoh.git>           |

### Configuration

This package can be configured via CMake arguments.

| Name                         | Description                                               | Default         |
|------------------------------|-----------------------------------------------------------|-----------------|
| RMW_ZENOH_PICO_TRANSPORT     | Sets Zenoh-Pico transport to use (unicast, mcast, serial) | unicast         |
| RMW_ZENOH_PICO_CONNECT       | Sets the scout address.                                   | 127.0.0.1       |
| RMW_ZENOH_PICO_CONNECT_PORT  | Sets the scout port.                                      | 7447            |
| RMW_ZENOH_PICO_LISTEN        | Sets the listen address.                                  | 127.0.0.1       |
| RMW_ZENOH_PICO_LISTEN_PORT   | Sets the listen port.                                     | -1              |
| RMW_ZENOH_PICO_MCAST         | Sets the multicast address,                               | 224.0.0.224     |
| RMW_ZENOH_PICO_MCAST_PORT    | Sets the multicast port.                                  | 7446            |
| RMW_ZENOH_PICO_MCAST_DEV     | Sets the multicast device.                                | eth0            |
| RMW_ZENOH_PICO_SERIAL_DEVICE | Sets the agent serial port. *1                            | serial/usb      |
| RMW_ZENOH_PICO_SERIAL_PARAM  | Sets the agent serial parameter *1                        | baudrate=115200 |

*1 Will support in the future

These configurations can be overridden by the MCU program using the rmw_zenoh_pico API.

|              Function              |                                                      Configuration                                                      |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| rmw_zenoh_pico_set_unicast()       | RMW_ZENOH_PICO_CONNECT,</br>RMW_ZENOH_PICO_CONNECT_PORT,</br>RMW_ZENOH_PICO_LISTEN,</br>RMW_ZENOH_PICO_LISTEN_PORT</br> |
| rmw_zenoh_pico_set_mcast()         | RMW_ZENOH_PICO_MCAST,</br>RMW_ZENOH_PICO_MCAST_PORT,</br>RMW_ZENOH_PICO_MCAST_DEV</br>                                  |
| rmw_zenoh_pico_set_serial_device() | RMW_ZENOH_PICO_SERIAL_DEVICE, </br>RMW_ZENOH_PICO_SERIAL_PARAM</br>                                                     |
| rmw_zenoh_pico_set_mode()          | None                                                                                                                    |

Each configuration pair value must be consistent between the connecting Zenoh peer/router.

| type           | place of modifys                                           |
|----------------|------------------------------------------------------------|
| rmw_zenoh      | configuration with ZENOH_SESSION_CONFIG_URI                |
| rmw_zneoh_pico | into application by function(use rmw_zenoh_pico_set_xxx()) |

<!--
Case unicast:
 node1                                       node2
+----------------------------------+       +---------------------------------+
|  +----------------------------+  |       |  +----------------------------+ |
|  | RMW_ZENOH_PICO_LISTEN_XXX  +--|-------|--+ RMW_ZENOH_PICO_CONNECT_XXX | |
|  +------------+---------------+  |       |  +----------------------------+ |
+---------------|------------------+       +---------------------------------+
 |                                         node3
 |                          +---------------------------------+
 |                          |  +----------------------------+ |
 +-----------------------------+ RMW_ZENOH_PICO_CONNECT_XXX | |
 |  +----------------------------+ |
 +---------------------------------+

Case multicast:
 node1                                         node2
+----------------------------------+       +---------------------------------+
|  +----------------------------+  |       |  +----------------------------+ |
|  | RMW_ZENOH_PICO_MCAST_XXX   +--|-------|--+ RMW_ZENOH_PICO_MCAST_XXX   | |
|  +----------------------------+  |       |  +----------------------------+ |
+----------------------------------+       +---------------------------------+
-->

![stack_diag.svg](./design/config.svg)

- The RMW_ZENOH_PICO_LISTEN specifies listening on a connection address.(tcp, udp, serial,,,,)
- The RMW_ZENOH_PICO_LISTEN PORT can be set to -1 when allocated automatically, which reserves the port.
- The RMW_ZENOH_PICO_CONNECT_XXX specifies the address of the connection destination.
- The RMW_ZENOH_PICO_MCAST_XXX uses the same address on all nodes.

> [!IMPORTANT]
>
> - The value of RMW_ZENOH_PICO_TRANSPORT can only be set when creating a build image.
> - Each of these functions must be called before rclc_support_init() is executed.
> - The current implementation does not support any serial.

## Installing and build for rmw_zenoh_pico with the micro-ROS system

The rmw_zenoh_pico is used instead of the XRCE-DDS layer in the micro-ROS product.  
The rmw_zenoh_pico has to install the micro-ROS product before building it.

> [!NOTE]
> The rmw_zenoh_pico needs part of any library to get the hash value in the fastdds library.  
> Therefore, the value of RMW_IMPLEMENTATION should not be set before building rmw_zenoh_pico if you use customize rmw_middleware (ex. cyclonedds).

### Network Topology

The rmw_zenoh_pico uses the standard Zenoh protocol; no special conversion processing is required to communicate between the rmw_zenoh nodes.  
For this purpose, the same configuration as the Zenoh network topology can be used.  

> [!NOTE]
> The implementation of Zenoh-Pico does not implement the route function, so it cannot become zenohd.

#### Type1 (connect Zenoh router)

All ROS nodes connect to zenohd, and their communication is carried out using the router function.

<!--
 +------------+                  +----------------+
 | rclcpp app |                  |    rclc app    |
 +------------+    +--------+    +----------------+
 | rmw_zenoh  +----+ zenohd +----+ rmw_zenoh_pico |
 +------------+    +--------+    +----------------+
-->

![network_diag.svg](./design/topology_type1.svg)

- Every ROS node can explicitly select a zenohd to connect to using unicast/multicast communication.
- ROS nodes using the rmw_zenoh can discover zenohd to connect to automatically using multicast. (but, the rmw_zenoh_pico does not impliment this future, yet.)
- The topic of the rmw_zenoh node is the same protocol used by rmw_zenoh, so the topic can be serialized/deserialized to the rmw_zenoh layer itself.
- All ROS topics are sent via zenohd to the destination ROS node.
- Due to the Zenoh specification, it is possible to continue using the network automatically when zenohd is restarted.

> [!IMPORTANT]
> Zenohd generated by the rmw_zenoh build can be replaced with the daemon generated by the original Zenoh product build or zenoh_plugin_ros2dds. However, the zenoh_plugin_ros2dds cannot bridge between (cyclone) DDS.  
> see: [on-interoperability-with-eclipse-zenohzenoh-plugin-ros2dds-and-zenoh-bridge-ros2dds](https://github.com/ros2/rmw_zenoh#on-interoperability-with-eclipse-zenohzenoh-plugin-ros2dds-and-zenoh-bridge-ros2dds)  

The rmw_zenoh_pico can connect to zenohd via unicast TCP/IP.
This makes it relatively easy to set up a ROS communication environment in a network that uses a firewall or NAT. (STUN server)

<!--
 +------------+                              +----------------+
 | rclcpp app |                              |    rclc app    |
 +------------+  +----+  +--------+  +----+  +----------------+
 | rmw_zenoh  +--+ FW +--+ zenohd +--+ FW +--+ rmw_zenoh_pico |
 +------------+  +----+  +--------+  +----+  +----------------+
-->

STUN service example 1:

- Deploying zenohd on the internet

![network_diag.svg](./design/topology_type1_fw_ex1.svg)

#### Type2 (connect ros node with rmw_zenoh)

The rmw_zenoh_pico node connects to the rmw_zenoh node directly.

<!--
 +------------+    +-----------------+
 | rclcpp app |    |    rclc app     |
 +--------+    +------------+    +-----------------+
 | zenohd +----+ rmw_zenoh  +----+ rmw_zenoh_pico  |
 +--------+    +------------+    +-----------------+
-->

![network_diag.svg](./design/topology_type2.svg)

- the rmw_zenoh_pico node connect to rmw_zenoh node by peer mode. And rmw_zenoh connects to zenohd by peer/multicast mode.
- In this case, topic data of rmw_zenoh/rmw_zneoh_pico does not pass through zenohd. This is expected to simplify the network topology and improve communication performance.
- The rmw_zenoh needs to connect to zenohd on startup, but does not require zenohd to handle the ROS topic communication itself after it has started. In this case, local communication will continue even if zenohd stops for some reason.

> [!IMPORTANT]
> This is Experimental Mode.
> Zenoh's peer mode can reconnect automatically when the remote node reboots.  
> However, the client mode of Zenoh cannot reconnect in rmw_zenoh with Zenoh 1.4.0.  
> When rmw_zenoh restarts, its information data will be sent to zenohd.  
> Currently, the rmw_zenoh implementation does not send its own information to client nodes.  
> In this case, if the Zenoh-Pico node loses connection with rmw_zenoh, it will attempt to reconnect using _z_reopen() on its own lease task.  
> However, since no information is sent to rmw_zenoh_pico after reconnecting from the rmw_zenoh, the rmw_zenoh_pico will not resume communication.  
> This feature can be turned off in the Zenoh-Pico configuration (Z_FEATURE_INTEREST).  
> The rmw_zenoh_pico is built with this configuration disabled by default.  

#### Type3 (connect direct peer)

Due to its specifications, a Zenoh communication environment can be created if at least one peer node exists.  
The rmw_zenoh_pico does not support all the features of ROS (network graph, etc.), so connecting to Zenoh is not required.  
In this case, it is possible to directly connect multiple rmw_zenoh_pico devices using the simple peer function of zenoh_pico.  
This network topology is a variation of the type 2 described above.

<!--
 +----------------+     +----------------+
 |    rclc app    |     |    rclc app    |
 +----------------+-----+----------------|
 | rmw_zenoh_pico |     | rmw_zenoh_pico |
 +----------------+     +----------------+
-->

![network_diag.svg](./design/topology_type3.svg)

- minimally connected environment
- The zenohd is not required for connection
- You can build your environment by selecting multicast or unicast.
- Management functions such as "ros node list" cannot be used.
- Since communication parameters are not adjusted during communication, they must be adjusted manually in advance. (ex, [troubleshooting](https://github.com/eclipse-zenoh/zenoh-pico#troubleshooting))

> [!IMPORTANT]
> This is Experimental Mode.
> Currently, we are unable to confirm the operation of some desired features in zenoh_pico peer mode.
> For details, see the [issue](#known-issueslimitations) section.

### Example/Demo target information

The rmw_zenoh_pico has a few demo programs.  
This section introduces a sample communication example between rmw_zenoh_pico and zenoh_pico.  
For example, if you want to generate a host environment for rmw_zenoh_pico, please refer to the [first_application_linux](https://micro.ros.org/docs/tutorials/core/first_application_linux/) document to install the host environment.  

This chapter explains the procedure for generating an MCU image for this example program.

Target connection summary:

Target environments:

| Type of rmw    | ROS application name | Node name      |
|----------------|----------------------|----------------|
| rmw_zenoh      | talker               | /talker        |
| rmw_zenoh_pico | listener             | /listener_node |

| use token |
|-----------|
| /chatter  |

| Target                                        | IP address | Listen Port |
|-----------------------------------------------|------------|-------------|
| Linux target  / zenohd (on rmw_zenoh)         | 10.0.10.30 | 7447        |
| Linux target  / ros application(on rmw_zenoh) | 10.0.10.30 | 8000        |
| MCU_1 or raspberrypi target                   | 10.0.10.50 | 8000        |
| MCU_2                                         | 10.0.10.51 | 8000        |

### [FORMAL/Manual] Creating a new firmware workspace

The official method for generating an ROS application image using rmw_zenoh_pico involves steps similar to creating a microROS image.

1. Create a workspace and download the micro-ROS tools
2. build image

The current patch for rmw_zenoh_pico to micro_ros_setup makes it possible to create the following Linux image.

- Linux host target
- Raspberry Pi OS 32/64 target

> [!NOTE]
>
> The rmw_zenoh_pico have to add patch to product of [micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup).
> If you need the patched product, you can clone the branch of rmw_zenoh_pico from [mirror of micro_ros_setup](https://github.com/esol-community/micro_ros_setup).

#### 1. Create a workspace and download the micro-ROS tools

``` bash
unset RMW_IMPLEMENTATION
mkdir uros_ws && cd uros_ws
git clone -b rmw_zenoh_pico https://github.com/esol-community/micro_ros_setup src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
```

#### 2.1 Build Linux target

``` bash
# cd uros_ws
source install/local_setup.bash
ros2 run micro_ros_setup create_firmware_ws.sh zenoh host
source install/local_setup.bash
ros2 run micro_ros_setup build_firmware.sh
file ./install/rmw_zenoh_pico/lib/rmw_zenoh_demos_rclc/listener/listener
# ./install/rmw_zenoh_pico/lib/rmw_zenoh_demos_rclc/listener/listener: ELF 64-bit LSB pie executable, x86-64, version 1 (SYSV), dynamically linked, interpreter /lib64/ld-linux-x86-64.so.2, BuildID[sha1]=2c0119cf4620f58fafa736594d3bba038e56e13a, for GNU/Linux 3.2.0, not stripped
```

#### 2.2 Build Raspberry Pi target

``` bash
# cd uros_ws
source install/local_setup.bash
ros2 run micro_ros_setup create_firmware_ws.sh zenoh raspbian bookworm_v12
ros2 run micro_ros_setup configure_firmware.sh listener -t unicast -i <ip> -p <port>
source install/local_setup.bash
ros2 run micro_ros_setup build_firmware.sh
file ./firmware/bin/listener
# ./firmware/bin/listener: ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), dynamically linked, interpreter /lib/ld-linux-armhf.so.3, for GNU/Linux 3.2.0, with debug_info, not stripped
```

The zenoh_pico in rmw_zenoh_pico is executing in client mode, and the connection has to be set to Zenoh and its router.
The Raspberry Pi OS configuration for rmw_zenoh_pico has to set its IP address and port when executing configure_firmware.

| Name | Mean                         | Example       |
|------|------------------------------|---------------|
| ip   | address connect zenoh/zenohd | -i 10.0.10.30 |
| port | port connect zenoh           | -p 7447       |

The Raspberry Pi OS configuration on this patch on the rmw_zeno_pico package used Raspberry Pi 1 and Raspberry Pi Pico toolchains (ELF32bit/EABI5).
If you want to use another Raspberry Pi target, which is a 64-bit environment, you have to change part of the toolchain URL in create.sh manually.

``` shell-session
$ cat micro_ros_setup/config/zenoh/raspbian/create.sh
#! /bin/bash

pushd $FW_TARGETDIR/$DEV_WS_DIR >/dev/null
 if [ $OPTION == "bookworm_v12" ]; then
 TOOLCHAIN_URL="https://..."              /* Change URL of cross-compile toolchain */
 /* For match target Raspberry Pi OS environment */
 :
 :
 else
 echo "Platform not supported."
 exit 1
 fi
 :
```

See: <https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/>

### [Makefile] Creating a new firmware workspace

The release repository of rmw_zenoh pico includes a patch for the micro_ros_setup product.
A Makefile is installed that can apply those patches and create an image of the MCU.

Images that can be generated with this Makefile:

- Linux host target
- Raspberry Pi OS 32/64 target

#### 1. Create a workspace and download

``` bash
git clone -b 1.4.0 https://github.com/esol-community/rmw_zenoh_pico uros_ws
```

#### 2. Download and apply the patch to the micro-ROS tools

``` bash
cd uros_ws
make setup
rosdep update && rosdep install --from-paths src --ignore-src -y
```

#### 3.1 Build Linux target

Manually select Makefile options(include) for the image you want to generate.

``` shell-session
$ head -10 Makefile

include microros/Makefile.host

# include microros/Makefile.raspbian_listener
# include microros/Makefile.raspbian_talker
# include microros/Makefile.raspbian_server
# include microros/Makefile.raspbian_client

#################################################
```

``` bash
make create
make build
```

``` shell-session
$ file ./install/rmw_zenoh_pico/lib/rmw_zenoh_demos_rclc/listener/listener
./install/rmw_zenoh_pico/lib/rmw_zenoh_demos_rclc/listener/listener: ELF 64-bit LSB pie executable, x86-64, version 1 (SYSV), dynamically linked, interpreter /lib64/ld-linux-x86-64.so.2, BuildID[sha1]=e25777f46b0e5a9d9f2fff85ac815411e9dd06c7, for GNU/Linux 3.2.0, not stripped
```

#### 3.2 Build Raspberry Pi target (Raspberry Pi 1 Model B+)

Manually select Makefile options(include) for the image you want to generate.

``` shell-session
$ head -10 Makefile

# include microros/Makefile.host

include microros/Makefile.raspbian_listener
# include microros/Makefile.raspbian_talker
# include microros/Makefile.raspbian_server
# include microros/Makefile.raspbian_client

#################################################
```

``` shell-session
$ cat microros/Makefile.raspbian_listener
ROS_DISTR     = jazzy
TARGET_PLATFORM = raspbian
TARGET_TARGET   = bookworm_v12

CONNECT_TYPE    = unicast
 :
CONNECT_IP      = 10.0.10.30
CONNECT_PORT    = 7447
 :
LISTEN_IP       = 10.0.10.50
LISTEN_PORT     = 8000
 :
TARGET_APP     = listener
```

``` bash
make create
make configure
make build
```

``` shell-session
$ file firmware/bin/listener
firmware/bin/listener: ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), dynamically linked, interpreter /lib/ld-linux-armhf.so.3, for GNU/Linux 3.2.0, with debug_info, not stripped
```

## Running the micro-ROS app

For sample details, see [Setup](https://github.com/ros2/rmw_zenoh#setup) and [Test](https://github.com/ros2/rmw_zenoh#test) in the rmw_zenoh repository.

### Running Zenoh router and ROS node on Linux

The following commands are executed on another terminal, respectively.

1. Start the Zenoh router on rmw_zenoh

 ``` bash
    # cd <directory in install rmw_zenoh>
    source install/setup.bash
    export ZENOH_CONFIG_OVERRIDE='transport/link/tx/lease=10000'
    ros2 run rmw_zenoh_cpp rmw_zenohd
 ```

2. Run the talker on rmw_zenoh

 ``` bash
    # cd <directory in install rmw_zenoh>
    sudo apt-get install ros-$ROS_DISTRO-demo-nodes-py
    sudo apt-get install ros-$ROS_DISTRO-demo-nodes-cpp
    source /opt/ros/$ROS_DISTRO/local_setup.bash
    source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    ros2 run demo_nodes_cpp talker
 ```

### Running target

#### Running Linux target

1. Run the listener on rmw_zenoh_pico in microros

 ``` bash
    # cd uros_ws
    source install/local_setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_pico
    ros2 run rmw_zenoh_demos_rclc listener
 ```

#### Running Raspberry Pi target (on Raspberry Pi H/W)

1. Copy the micro-ROS application to Raspberry Pi (on host)

 ``` bash
    # cd uros_ws
    scp firmware/bin/listener <target raspberry pi>:~/.
 ```

2. Run listener

 ``` bash
    ssh <target raspberry pi>
    ./listener
 ```

#### Running RTOS target

1. upload MCU target image

 ``` bash
    pio run -t upload
 ```

### Name of node/topic on ROS 2

The ROS 2 CLI command starts a new ROS 2 daemon task, and the daemon caches data.
If the ROS 2 daemon is already running, it must be stopped before executing ROS 2 CLI commands.

1. Get the node name if the listener node is executed

 ``` bash
    ros2 daemon stop
    # The daemon has been stopped
    source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    ros2 node list
    # [INFO] [1728446929.812251213] [rmw_zenoh_cpp]: Successfully connected to a Zenoh router with id aebe653d2ff57b4ba1ef7b43c59b547.
    # [WARN] [1728446931.353213030] [rmw_zenoh_cpp]: Received liveliness token to remove node /_ros2cli_361692 from the graph before all pub/subs/clients/services for this node have been removed. Removing all entities first...
    # /listener_node
 ```

2. Get the topic name if the listener node is executed

 ``` bash
    ros2 daemon stop
    # The daemon has been stopped
    source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    ros2 topic list
    # [INFO] [1728446957.797293950] [rmw_zenoh_cpp]: Successfully connected to a Zenoh router with id aebe653d2ff57b4ba1ef7b43c59b547.
    # [WARN] [1728446959.338124246] [rmw_zenoh_cpp]: Received liveliness token to remove node /_ros2cli_361735 from the graph before all pub/subs/clients/services for this node have been removed. Removing all entities first...
    # /parameter_events
    # /rosout
    # /chatter
 ```

The new ROS 2 daemon started by ROS 2 CLI exits when this command exits.
At this time, the rmw_zenoh middleware outputs a few messages.
This message will change depending on your build/execute environment.

## Purpose of the Project

This software is not ready for production use.
It has neither been developed nor tested for a specific use case.
However, the license conditions of the applicable open-source licenses allow you to adapt the software to your needs.
Before using it in a safety-relevant setting, ensure the software fulfills your requirements and adjusts them according to applicable safety standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in this repository, see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

1. The rmw_zenoh_pico uses malloc() system functions when there is a new memory region.
The XRCE-DDS implementation has simple memory futures in its layer.
This memory-future is designed to run ROS 2 communication for small resource systems.
The rmw_zenoh layer can execute on a Linux system with a memory subsystem for a rich system with Zenoh.
The Zenoh-Pico is designed for small resource systems.
However, the Zenoh-Pico memory is compiling into its system, and it is not designed to use the same data area as an upper layer, which is rmw_zenoh_pico.
If you use rmw_zeno_pico on a small resource system with any RTOS, you might need to add some customizations for Zenoh-Pico and rmw_zenoh_pico.

1. This implementation of rmw_zenoh_pico does not support graph caching.
The graph cache is a key component in maintaining the overall structure of ROS2.
However, we feel that it will be used less in micro-ROS for microcontroller targets.
Moreover, rmw_zenoh_pico needs to allocate memory to hold the graph cache dynamically.
On the other hand, the graph cache is closely related to the Qos feature of ROS2.
Since Zenoh is based on TCP/IP, the OS basically guarantees packet reliability, so there are few problems.
However, this can be an issue when communicating directly with ROS using the Zenoh plugin, etc.
We feel it is necessary to add limited OoS functionality to the link layer used by the microcontroller.
The ROS QoS implementation works in conjunction with ROS event processing.
The current implementation of rmw_zenoh_pico only implements the event framework (compile only).

1. The Zenoh-Pico peer mode cannot connect to zenohd.
The peer mode of Zenoh-Pico cannot connect to zenohd.
We have not yet determined whether this is a feature or a problem with our environment.
For this reason, the peer mode of rmw_zenoh_pico is currently only used when connecting each node directly.
If you request to switch the connection mode of rmw_zenoh_pico, a restart or other process is required.
This phenomenon does not occur in client mode. For this reason, if you use rmw_zenoh_pico, we recommend connecting to zenohd or the desired rmw_zenoh node in client mode.
