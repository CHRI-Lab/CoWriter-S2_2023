export DOCKER_REPO=aplaire
export BASE_IMAGE=${DOCKER_REPO}/nao-base:bluering
export IMAGE=${DOCKER_REPO}/nao-bluering
export IMAGE_LITE=chienpul/nao-bluering-lite:v4
export CONTAINER=nao-bluering
export GOOGLE_APPLICATION_CREDENTIALS=/home/nao/credentials/comp90082.json

build-base:
	docker build --tag ${BASE_IMAGE} -f base.Dockerfile .
push-base:
	docker build --tag ${BASE_IMAGE}
pull-base:
	docker build --tag ${BASE_IMAGE}

build:
	docker build --tag ${IMAGE} --platform linux/amd64 . 
push:
	docker push ${IMAGE}
pull:
	docker pull ${IMAGE}

run-robot:
	docker run --rm -it \
		--network="host" \
		--user nao \
		-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=${DISPLAY} \
		--name ${CONTAINER} \
		${IMAGE} \
		bash
	
run-gui:
	docker exec -it \
		--user nao \
		${CONTAINER} \
		bash -c "python3 ./src/choose_adaptive_words/nodes/activity.py"

run-roscore:
	docker run --rm -it \
		--network host \
		--device /dev/snd \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /home/ubuntu/credentials:/home/nao/credentials \
		-e DISPLAY=unix${DISPLAY} \
		-e OPENAI_KEY=${OPENAI_KEY} \
		-e GOOGLE_APPLICATION_CREDENTIALS=${GOOGLE_APPLICATION_CREDENTIALS} \
		--user nao \
		--name ${CONTAINER} \
		${IMAGE_LITE} \
		roscore

run-ui:
	docker exec -it \
		--user nao \
		--workdir /home/nao/catkin_ws/src/choose_adaptive_words/nodes \
		${CONTAINER} \
		bash -c "python3 ./activity.py"

run-cowriter:
	docker exec -it \
		--user nao \
		${CONTAINER} \
		bash