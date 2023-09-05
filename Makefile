export DOCKER_REPO=aplaire
export BASE_IMAGE=${DOCKER_REPO}/nao-base:bluering
export IMAGE=${DOCKER_REPO}/nao-bluering
export CONTAINER=nao-bluering

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

