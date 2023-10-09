# With Docker Compose
compose-build-development:
	docker-compose \
		--file ${FILE_PATH}/docker-compose.yml \
		--env-file ./docker/docker.env \
		build

compose-up-development:
	docker-compose \
		--file ${FILE_PATH}/docker-compose.yml \
		up

# Without Docker Compose
build-nodes-development:
	docker build \
		--tag ${NODES_IMAGE} \
		--file ${FILE_PATH}/nodes.Dockerfile .

build-controller-development:
	docker build \
		--tag ${CONTROLLER_IMAGE} \
		--file ${FILE_PATH}/controller.Dockerfile .

build-frontend-development:
	docker build \
		--tag ${FRONTEND_IMAGE} \
		--file ${FILE_PATH}/frontend.Dockerfile .

run-nodes-development:
	docker run -it \
		--name ${NODES_CONTAINER} \
		--user nao \
		--network host \
		-v ./src:/home/nao/src \
		-v ./.env:/home/nao/.env \
		-v ./share:/home/nao/share \
		${NODES_IMAGE}

run-controller-development:
	docker run -it \
		--name ${CONTROLLER_CONTAINER} \
		-p 3000:3000 \
		-v ./src/controller:/home/nao/controller \
		${CONTROLLER_IMAGE}

run-frontend-development:
	docker run -it \
		--name ${FRONTEND_CONTAINER} \
		-v ./src/frontend/app:/usr/src/app \
        -v /usr/src/app/node_modules \
		${FRONTEND_IMAGE}

start-nodes-development:
	docker start ${NODES_CONTAINER}

start-controller-development:
	docker start ${CONTROLLER_CONTAINER}

start-frontend-development:
	docker start ${FRONTEND_CONTAINER}

rm-nodes-development:
	docker rm -f ${NODES_CONTAINER}

rm-controller-development:
	docker rm -f ${CONTROLLER_CONTAINER}
	
rm-frontend-development:
	docker rm -f ${FRONTEND_CONTAINER}
