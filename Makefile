include ./docker/docker.env
include ./docker/production/production.mk
include ./docker/development/development.mk

# ensure that the ENV variable is set to either "production" or "development"
# before running any of the targets below
TARGETS = $(filter-out $@,$(MAKECMDGOALS))
$(eval $(TARGETS): check-env)

check-env:
	@if [ -z "$(ENV)" ]; then \
		echo "Set the ENV variable either to \"production\" or \"development\".\nexport ENV=..."; \
		exit 1; \
	elif [ "$(ENV)" != "production" ] && [ "$(ENV)" != "development" ]; then \
		echo "Error: ENV must be set to either \"production\" or \"development\"."; \
		exit 1; \
	else \
		echo "ENV is set to $(ENV)"; \
	fi

# Using Docker Compose
compose-build:
	docker compose \
		--file docker-compose.${ENV}.yml \
		--env-file ./docker/docker.env \
		build
compose-up:
	docker compose \
		--file docker-compose.${ENV}.yml \
		--env-file ./docker/docker.env \
		up

# Without Docker Compose
build-nodes: build-nodes-${ENV}
build-frontend: build-frontend-${ENV}
build: build-nodes build-frontend

run-nodes: run-nodes-${ENV}
run-frontend: run-frontend-${ENV}

start-nodes: start-nodes-${ENV}
start-frontend: start-frontend-${ENV}

rm-nodes: rm-nodes-${ENV}
rm-frontend: rm-frontend-${ENV}
rm: rm-nodes rm-frontend