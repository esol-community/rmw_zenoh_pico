
include microros/Makefile.host

# include microros/Makefile.raspbian_listener
# include microros/Makefile.raspbian_talker
# include microros/Makefile.raspbian_server
# include microros/Makefile.raspbian_client

#################################################

SHELL           = /bin/bash
TARGET          := firmware/bin/${TARGET_APP}

SETUP_PATCH_DIR := ${CURDIR}/patches/$(ROS_DISTRO)/micro_ros_setup

SRC_DIR         := ${CURDIR}/src

.PHONY: setup
setup :
	if [ -d ${CURDIR}/rmw_zenoh_pico ]; then \
	  touch ${CURDIR}/rmw_zenoh_pico/COLCON_IGNORE; \
	fi

	if [ ! -d "$(SRC_DIR)" ]; then \
	  echo 'clone develop git repo' \
	  mkdir -p "$(SRC_DIR)"; \
	  git clone -b rmw_zenoh_pico https://github.com/esol-community/micro_ros_setup $(SRC_DIR)/micro_ros_setup ; \
	  colcon build; \
	fi

.PHONY: create
create :
ifeq ($(TARGET_PLATFORM), host)
	export ROS_DISTRO=${ROS_DISTRO} ; \
	source install/local_setup.sh ; \
	ros2 run micro_ros_setup create_firmware_ws.sh zenoh host
else
	source install/local_setup.sh ; \
	ros2 run micro_ros_setup create_firmware_ws.sh zenoh ${TARGET_PLATFORM} ${TARGET_TARGET}
endif

.PHONY: configure
configure :
ifneq ($(TARGET_PLATFORM), host)
ifeq ($(CONNECT_TYPE), unicast)
	export ROS_DISTRO=${ROS_DISTRO} ; \
	export FEATURE_INTEREST=${FEATURE_INTEREST}; \
	source install/local_setup.sh ; \
	ros2 run micro_ros_setup configure_firmware.sh ${TARGET_APP} -t ${CONNECT_TYPE} -i ${CONNECT_IP} -p ${CONNECT_PORT}
else ifeq ($(CONNECT_TYPE), mcast)
	export ROS_DISTRO=${ROS_DISTRO} ; \
	source install/local_setup.sh ; \
	ros2 run micro_ros_setup configure_firmware.sh ${TARGET_APP} -t ${CONNECT_TYPE} -i ${MCAST_IP} -p ${MCAST_PORT}
endif
endif

.PHONY: build
build :
ifeq ($(TARGET_PLATFORM), host)
	unset RMW_IMPLEMENTATION ; \
	source install/local_setup.sh ; \
	export CONNECT_TYPE=${CONNECT_TYPE}; \
	export CONNECT_IP=${CONNECT_IP}; \
	export CONNECT_PORT=${CONNECT_PORT}; \
	export LISTEN_IP=${LISTEN_IP}; \
	export LISTEN_PORT=${LISTEN_PORT}; \
	export MCAST_IP=${MCAST_IP}; \
	export MCAST_PORT=${MCAST_PORT}; \
	export MCAST_DEV=${MCAST_DEV}; \
	export FEATURE_INTEREST=${FEATURE_INTEREST}; \
	ros2 run micro_ros_setup build_firmware.sh
else
	unset RMW_IMPLEMENTATION ; \
	source install/local_setup.sh ; \
	ros2 run micro_ros_setup build_firmware.sh
endif

.PHONY: clean
clean :
	rm -rf build firmware install log
ifeq ($(TARGET_PLATFORM), host)
	if [ -d "$(SRC_DIR)" ]; then \
	  cd ./src; \
	  rm -rf colcon.meta eProsima ros2 repos uros; \
	  cd ${CURDIR}; \
	fi
endif
	colcon build

.PHONY: dist-clean
dist-clean :
	@rm -rf build firmware install log src
