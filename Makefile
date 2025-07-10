PYSQUARED_VERSION ?= mike-252-bring-back-big-data
PYSQUARED ?= git+https://github.com/proveskit/pysquared@$(PYSQUARED_VERSION)

.PHONY: all
all: .venv download-libraries pre-commit-install help

.PHONY: help
help: ## Display this help.
	@awk 'BEGIN {FS = ":.*##"; printf "\nUsage:\n  make \033[36m<target>\033[0m\n"} /^[a-zA-Z_0-9-]+:.*?##/ { printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)

##@ Development

.venv: ## Create a virtual environment
	@echo "Creating virtual environment..."
	@$(MAKE) uv
	@$(UV) venv
	@$(UV) pip install --requirement pyproject.toml

.PHONY: download-libraries
download-libraries: uv .venv ## Download the required libraries
	@echo "Downloading libraries..."
	@$(UV) pip install --requirement lib/requirements.txt --target lib --no-deps --upgrade --quiet
	@$(UV) pip --no-cache install $(PYSQUARED) --target lib --no-deps --upgrade --quiet

	@rm -rf lib/*.dist-info
	@rm -rf lib/.lock

.PHONY: pre-commit-install
pre-commit-install: uv
	@echo "Installing pre-commit hooks..."
	@$(UVX) pre-commit install > /dev/null

.PHONY: sync-time
sync-time: uv ## Syncs the time from your computer to the PROVES Kit board
	$(UVX) --from git+https://github.com/proveskit/sync-time@1.0.1 sync-time

.PHONY: fmt
fmt: pre-commit-install ## Lint and format files
	$(UVX) pre-commit run --all-files

BOARD_MOUNT_POINT ?= ""
VERSION ?= $(shell git tag --points-at HEAD --sort=-creatordate < /dev/null | head -n 1)

.PHONY: install
install: build ## Install the project onto a connected PROVES Kit use `make install BOARD_MOUNT_POINT=/my_board_destination/` to specify the mount point
ifeq ($(OS),Windows_NT)
	rm -rf $(BOARD_MOUNT_POINT)
	cp -r artifacts/proves/* $(BOARD_MOUNT_POINT)
else
	@rm $(BOARD_MOUNT_POINT)/code.py > /dev/null 2>&1 || true
	$(call rsync_to_dest,artifacts/proves,$(BOARD_MOUNT_POINT))
endif

# install-firmware
.PHONY: install-firmware
install-firmware: uv ## Install the board firmware onto a connected PROVES Kit
	@$(UVX) --from git+https://github.com/proveskit/install-firmware@1.0.2 install-firmware v5b

.PHONY: clean
clean: ## Remove all gitignored files such as downloaded libraries and artifacts
	git clean -dfX

##@ Build

.PHONY: build
build: download-libraries mpy-cross ## Build the project, store the result in the artifacts directory
	@echo "Creating artifacts/proves"
	@mkdir -p artifacts/proves
	@echo "__version__ = '$(VERSION)'" > artifacts/proves/version.py
	$(call compile_mpy)
	$(call rsync_to_dest,.,artifacts/proves/)
	@$(UV) run python -c "import os; [os.remove(os.path.join(root, file)) for root, _, files in os.walk('artifacts/proves/lib') for file in files if file.endswith('.py')]"
	@echo "Creating artifacts/proves.zip"
	@zip -r artifacts/proves.zip artifacts/proves > /dev/null

define rsync_to_dest
	@if [ -z "$(1)" ]; then \
		echo "Issue with Make target, rsync source is not specified. Stopping."; \
		exit 1; \
	fi

	@if [ -z "$(2)" ]; then \
		echo "Issue with Make target, rsync destination is not specified. Stopping."; \
		exit 1; \
	fi

	@rsync -avh $(1)/config.json artifacts/proves/version.py $(1)/*.py $(1)/lib --exclude=".*" --exclude='requirements.txt' --exclude='__pycache__' $(2) --delete --times --checksum
endef

##@ Build Tools
TOOLS_DIR ?= tools
$(TOOLS_DIR):
	mkdir -p $(TOOLS_DIR)

### Tool Versions
UV_VERSION ?= 0.5.24
MPY_CROSS_VERSION ?= 9.0.5

UV_DIR ?= $(TOOLS_DIR)/uv-$(UV_VERSION)
UV ?= $(UV_DIR)/uv
UVX ?= $(UV_DIR)/uvx
.PHONY: uv
uv: $(UV) ## Download uv
$(UV): $(TOOLS_DIR)
	@test -s $(UV) || { mkdir -p $(UV_DIR); curl -LsSf https://astral.sh/uv/$(UV_VERSION)/install.sh | UV_INSTALL_DIR=$(UV_DIR) sh > /dev/null; }

UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)

MPY_S3_PREFIX ?= https://adafruit-circuit-python.s3.amazonaws.com/bin/mpy-cross
MPY_CROSS ?= $(TOOLS_DIR)/mpy-cross-$(MPY_CROSS_VERSION)
.PHONY: mpy-cross
mpy-cross: $(MPY_CROSS) ## Download mpy-cross
$(MPY_CROSS): $(TOOLS_DIR)
	@echo "Downloading mpy-cross $(MPY_CROSS_VERSION)..."
	@mkdir -p $(dir $@)
ifeq ($(OS),Windows_NT)
	@curl -LsSf $(MPY_S3_PREFIX)/windows/mpy-cross-windows-$(MPY_CROSS_VERSION).static.exe -o $@
else
ifeq ($(UNAME_S),Linux)
ifeq ($(or $(filter x86_64,$(UNAME_M)),$(filter amd64,$(UNAME_M))),$(UNAME_M))
	@curl -LsSf $(MPY_S3_PREFIX)/linux-amd64/mpy-cross-linux-amd64-$(MPY_CROSS_VERSION).static -o $@
	@chmod +x $@
else
	@echo "Pre-built mpy-cross not available for Linux machine: $(UNAME_M)"
endif
else ifeq ($(UNAME_S),Darwin)
	@curl -LsSf $(MPY_S3_PREFIX)/macos-11/mpy-cross-macos-11-$(MPY_CROSS_VERSION)-universal -o $@
	@chmod +x $@
else
	@echo "Pre-built mpy-cross not available for system: $(UNAME_S)"
endif
endif

define compile_mpy
	@$(UV) run python -c "import os, subprocess; [subprocess.run(['$(MPY_CROSS)', os.path.join(root, file)]) for root, _, files in os.walk('lib') for file in files if file.endswith('.py')]" || exit 1
endef
