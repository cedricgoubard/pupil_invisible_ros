####################################################################################################
######################################### USEFUL VARIABLES #########################################
####################################################################################################
THIS_FILE := $(lastword $(MAKEFILE_LIST))
.MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
CURRENT_DIR_SHORT := $(notdir $(patsubst %/,%,$(dir $(.MKFILE_PATH))))
CURRENT_DIR := $(dir $(.MKFILE_PATH))

# The names of the container and imager are set here; default to the name of the directory
TAG_NAME := pupil-invisible
CONTAINER_NAME := pupil-invisible

####################################################################################################
############################################# COMMANDS #############################################
####################################################################################################

# In case we have files with the same names as our commands, this avoids an error 
.PHONY: .pull .build start stop push test


# This is called when you run "make" with no arguments
default:
	@$(MAKE) -s .pull
	@$(MAKE) -s .build

.start_container_if_not_running:
	@if [ ! $$(docker ps -a | grep ${CONTAINER_NAME}) ]; then $(MAKE) start; fi	

# Create the ROS workspace ON THE HOST (not in the container; it will be mounted from the host to
# the container at runtime)
.pull:
	mkdir -p ${CURRENT_DIR}/volumes/ros_ws/src

# Build the docker image and the ROS workspace inside it
.build: stop
	docker build  --tag=personalroboticsimperial/prl:${TAG_NAME} .
	@docker run \
		--rm \
		--detach \
		--net host \
		-e ROS_IP \
		-e ROS_MASTER_URI='http://127.0.0.1:18452' \
		-v ${CURRENT_DIR}/volumes/ros_ws:/ros_ws:rw \
		-v ${CURRENT_DIR}:/ros_ws/src/${CURRENT_DIR_SHORT}:rw \
		-v /ros_ws/src/${CURRENT_DIR_SHORT}/ros_ws/ \
		-v ${CURRENT_DIR}/volumes/.cache/:/root/.cache/ \
		-v ${CURRENT_DIR}/volumes/.bash_history:/root/.bash_history \
		--privileged \
		--name ${CONTAINER_NAME} \
		personalroboticsimperial/prl:${TAG_NAME} bash -c "source /opt/ros/noetic/setup.bash; roscore -p 64258"
	@sleep 5
	@docker exec -it ${CONTAINER_NAME} bash -c "source /opt/ros/noetic/setup.bash; cd /ros_ws; apt update && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src --rosdistro noetic -y; catkin build"
	@$(MAKE) -s -f $(THIS_FILE) stop

# Starts the container with ROSCORE in the background
start: stop
	@docker run \
		--rm \
		--net host \
		--privileged \
		--runtime=nvidia \
		-e DISPLAY \
		-e ROS_MASTER_URI \
		-e ROS_IP \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v ${CURRENT_DIR}/volumes/ros_ws:/ros_ws:rw \
		-v ${CURRENT_DIR}:/ros_ws/src/${CURRENT_DIR_SHORT}:rw \
		-v /ros_ws/src/${CURRENT_DIR_SHORT}/ros_ws/ \
		-v ${CURRENT_DIR}/volumes/.cache/:/root/.cache/ \
		-v ${CURRENT_DIR}/volumes/.bash_history:/root/.bash_history \
		-it \
		--name ${CONTAINER_NAME} \
		personalroboticsimperial/prl:${TAG_NAME} bash 


stop:
	@docker stop ${CONTAINER_NAME} || true

push:
	@docker push personalroboticsimperial/prl:${TAG_NAME}

exec: .start_container_if_not_running
	@docker exec -it ${CONTAINER_NAME} bash
