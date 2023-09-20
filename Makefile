export NODES_IMAGE=nodes
export ROBOT_CONTROLLER_IMAGE=robot-controller
export NODES_CONTAINER=nodes
export ROBOT_CONTROLLER_CONTAINER=robot-controller

build-nodes:
	docker build --tag ${NODES_IMAGE} --platform linux/amd64 .

build-robot-controller:
	docker build --tag ${ROBOT_CONTROLLER_IMAGE} --platform linux/amd64 --file robot.Dockerfile .

run-nodes:
	docker run -it -d \
		--name ${NODES_CONTAINER} \
		--user nao \
		--network host \
		--device /dev/snd \
		-v ./src:/home/nao/NAOHW-Boxjelly/src \
		-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=${DISPLAY} \
		${NODES_IMAGE}

run-robot-controller:
	docker run -it \
		--name ${ROBOT_CONTROLLER_CONTAINER} \
		--user nao \
		--network="host" \
		-v ./src/controller:/home/nao/controller \
		${ROBOT_CONTROLLER_IMAGE} 

start-cowriter:
	docker exec -it ${NODES_CONTAINER} \
	bash -c "source ./install/setup.bash && /opt/ros/humble/bin/ros2 launch letter_learning_interaction cowriter.launch.py"

start-adaptive-words:
	docker exec -it ${NODES_CONTAINER} \
	bash -c "source ./install/setup.bash && /opt/ros/humble/bin/ros2 launch choose_adaptive_words adaptive.launch.py"

start-trajectory-following:
	docker exec -it ${NODES_CONTAINER} \
	bash -c "source ./install/setup.bash && /opt/ros/humble/bin/ros2 launch nao_trajectory_following trajectory.launch.py"

rm-nodes:
	docker rm -f ${NODES_CONTAINER}

rm-robot-controller:
	docker rm -f ${ROBOT_CONTROLLER_CONTAINER}