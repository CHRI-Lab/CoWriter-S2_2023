build-nodes-development:
	docker build --tag ${NODES_IMAGE} --platform linux/amd64 --file ${FILE_PATH}/nodes.Dockerfile .

build-controller-development:
	docker build --tag ${CONTROLLER_IMAGE} --platform linux/amd64 --file ${FILE_PATH}/controller.Dockerfile .

run-nodes-development:
	docker run -it \
		--name ${NODES_CONTAINER} \
		--user nao \
		--network host \
		-v ./src:/home/nao/src \
		-v ./share:/home/nao/share \
		-v ./.env:/home/nao/.env \
		-v ./share:/home/nao/share \
		${NODES_IMAGE}

run-controller-development:
	docker run -it \
		--name ${CONTROLLER_CONTAINER} \
		--user nao \
		--network="host" \
		-v ./src/controller:/home/nao/controller \
		${CONTROLLER_IMAGE}

start-nodes-development:
	docker start ${NODES_CONTAINER}

start-controller-development:
	docker start ${CONTROLLER_CONTAINER}

rm-nodes-development:
	docker rm -f ${NODES_CONTAINER}

rm-controller-development:
	docker rm -f ${CONTROLLER_CONTAINER}
	