build:
	docker build --tag boxjelly-ros2 --platform linux/amd64 .
build-controller:
	docker build --tag boxjelly-controller --platform linux/amd64 --file robot.Dockerfile .
run:
	docker run -it \
		--name boxjelly-ros2 \
		--user nao \
		--network="host" \
		-v ./src:/home/nao/NAOHW-RedBack/src \
		boxjelly-ros2 bash

run-controller:
	docker run -it \
		--name boxjelly-controller \
		--user nao \
		--network="host" \
		-v ./src/controller:/home/nao/controller \
		boxjelly-controller bash
rm-container:
	docker rm -f boxjelly-ros2