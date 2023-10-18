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

build-frontend-development:
	docker build \
		--tag ${FRONTEND_IMAGE} \
		--file ${FILE_PATH}/frontend.Dockerfile .

run-nodes-development:
	docker run -it \
		--name ${NODES_CONTAINER} \
		--user nao \
		--network host \
		--device /dev/snd \
		-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=${DISPLAY} \
		-v ./src:/home/nao/src \
		-v ./.env:/home/nao/.env \
		${NODES_IMAGE}

run-frontend-development:
	docker run -it \
		--name ${FRONTEND_CONTAINER} \
		-v ./src/frontend/app:/usr/src/app \
        -v /usr/src/app/node_modules \
		${FRONTEND_IMAGE}

start-nodes-development:
	docker start ${NODES_CONTAINER}

start-frontend-development:
	docker start ${FRONTEND_CONTAINER}

rm-nodes-development:
	docker rm -f ${NODES_CONTAINER}
	
rm-frontend-development:
	docker rm -f ${FRONTEND_CONTAINER}
