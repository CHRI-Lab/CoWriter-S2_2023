build-nodes-production:
	docker build --tag ${NODES_IMAGE} --platform linux/amd64 --file ${FILE_PATH}/nodes.Dockerfile .

build-controller-production:
	docker build --tag ${CONTROLLER_IMAGE} --platform linux/amd64 --file ${FILE_PATH}/controller.Dockerfile .

run-nodes-production:
	docker run -it --rm \
		--name ${NODES_CONTAINER} \
		--network host \
		--device /dev/snd \
		-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=${DISPLAY} \
		-v ./.env:/home/nao/.env \
		${NODES_IMAGE}

run-controller-production:
	docker run -it --rm \
		--name ${CONTROLLER_CONTAINER} \
		--user nao \
		--network="host" \
		${CONTROLLER_IMAGE}

rm-nodes-production:
	docker rm -f ${NODES_CONTAINER}

rm-controller-production:
	docker rm -f ${CONTROLLER_CONTAINER}
