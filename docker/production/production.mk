# Using Docker Compose
compose-build-production:
	docker compose \
		--file ${FILE_PATH}/docker-compose.yml \
		build

compose-up-production:
	docker compose \
		--file ${FILE_PATH}/docker-compose.yml \
		up

# Without Docker Compose
build-nodes-production:
	docker build \
		--tag ${NODES_IMAGE} \
		--file ${FILE_PATH}/nodes.Dockerfile .

build-frontend-production:
	docker build \
		--tag ${FRONTEND_IMAGE} \
		--file ${FILE_PATH}/frontend.Dockerfile .

run-nodes-production:
	docker run -it --rm \
		--name ${NODES_CONTAINER} \
		--network host \
		--device /dev/snd \
		-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=${DISPLAY} \
		-v ./.env:/home/nao/.env \
		-v ./share:/home/nao/share \
		-v ./strugg_letter_data:/home/nao/strugg_letter_data \
		-v ./credentials:/home/nao/credentials \
		${NODES_IMAGE}

run-frontend-production:
	docker run -it \
		--name ${FRONTEND_CONTAINER} \
		-p 80:80 \
		-v ./src/frontend/app:/usr/src/app \
        -v /usr/src/app/node_modules \
		${FRONTEND_IMAGE}

rm-nodes-production:
	docker rm -f ${NODES_CONTAINER}

rm-frontend-production:
	docker rm -f ${FRONTEND_CONTAINER}
