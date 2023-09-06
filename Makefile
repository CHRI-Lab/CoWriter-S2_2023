export DOCKER_REPO=aplaire
export IMAGE=${DOCKER_REPO}/nao-redback
export CONTAINER=nao-redback

build:
	docker build --tag ${IMAGE} . 
push:
	docker push ${IMAGE}
pull:
	docker pull ${IMAGE}


run-gui:
	docker compose -f ./resources/NAOHW-RedBack/src/nao_drawing_board/docker-compose.dev.yml up

run-gui-build:
	docker compose -f ./resources/NAOHW-RedBack/src/nao_drawing_board/docker-compose.dev.yml up --build

run-test:
	docker run --rm -it \
		--network="host" \
		--user nao \
		-e DISPLAY=unix${DISPLAY} \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		--name ${CONTAINER} \
		${IMAGE} \
		bash -c "python2 ./tests/Naoqi2robot/nao_bot_controller.py"

run-audio-chat:
	docker exec -it \
		--user nao \
		${CONTAINER} \
		bash -c "cd ./src/nao_ros2_ws && bash"
