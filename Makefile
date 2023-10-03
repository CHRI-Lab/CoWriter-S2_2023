export FILE_PATH=./docker/${ENV}

export NODES_IMAGE=nodes-${ENV}
export CONTROLLER_IMAGE=controller-${ENV}

export NODES_CONTAINER=nodes-${ENV}
export CONTROLLER_CONTAINER=controller-${ENV}


include ./docker/production/production.mk
include ./docker/development/development.mk


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

build-nodes: build-nodes-${ENV}
build-controller: build-controller-${ENV}
build: build-nodes build-controller

run-nodes: run-nodes-${ENV}
run-controller: run-controller-${ENV}

start-nodes: start-nodes-${ENV}
start-controller: start-controller-${ENV}

rm-nodes: rm-nodes-${ENV}
rm-controller: rm-controller-${ENV}
rm: rm-nodes rm-controller