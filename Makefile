build:
	docker build --tag boxjelly-ros2 --platform linux/amd64 .
run:
	docker run -it \
		--name boxjelly-ros2 \
		--user nao \
		-v ./src:/home/nao/NAOHW-RedBack/src \
		boxjelly-ros2 bash
rm-container:
	docker rm -f boxjelly-ros2