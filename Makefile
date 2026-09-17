# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

-include .env
export

CACHE_FLAGS ?=

.PHONY: clean check-intrinsic-env format format-check \
	moveit_planning_service moveit_planning_service_proto_desc moveit_planning_service.bundle.tar \
	install_moveit_planning_service add_moveit_planning_service delete_moveit_planning_service uninstall_moveit_planning_service \
	moveit_plan_motion_skill moveit_plan_motion_skill_proto_desc moveit_plan_motion_skill.bundle.tar install_moveit_plan_motion_skill uninstall_moveit_plan_motion_skill \
	moveit_plan_grasp_skill moveit_plan_grasp_skill_proto_desc moveit_plan_grasp_skill.bundle.tar install_moveit_plan_grasp_skill uninstall_moveit_plan_grasp_skill \
	install_motion_planning_skill uninstall_motion_planning_skill install_grasp_planning_skill uninstall_grasp_planning_skill

FORMAT_EXCLUDED_DIRS := build install log .git third_party
FORMAT_FILE_EXTENSIONS := "*.cpp" "*.hpp" "*.h" "*.cc" "*.proto"

FORMAT_EXCLUDED_DIRS_EXPRESSION := \
	$(foreach dir,$(FORMAT_EXCLUDED_DIRS),-name $(dir) -o) -false

FORMAT_FILE_EXTENSIONS_EXPRESSION := \
	$(foreach extension,$(FORMAT_FILE_EXTENSIONS),-name $(extension) -o) -false

FIND_FORMAT_SOURCES := find . \
	-type d \( $(FORMAT_EXCLUDED_DIRS_EXPRESSION) \) -prune \
	-o -type f \( $(FORMAT_FILE_EXTENSIONS_EXPRESSION) \) -print0

format:
	@$(FIND_FORMAT_SOURCES) | xargs -0 -r clang-format -i

format-check:
	@$(FIND_FORMAT_SOURCES) | xargs -0 -r clang-format --dry-run --Werror

check-intrinsic-env:
ifndef SDK_VERSION
	$(error SDK_VERSION must be defined)
endif
ifndef INTRINSIC_ORGANIZATION
	$(error INTRINSIC_ORGANIZATION must be defined)
endif
ifndef INTRINSIC_CLUSTER
	$(error INTRINSIC_CLUSTER must be defined)
endif
ifndef ROS_DISTRO
	$(error ROS_DISTRO must be defined)
endif

download_inbuild_and_inctl: check-intrinsic-env
	@mkdir -p bin/
	@if [ -f ./bin/inbuild ] && ./bin/inbuild --help >/dev/null 2>&1 && \
	   [ -f ./bin/inctl ] && ./bin/inctl --help >/dev/null 2>&1; then \
		echo "Intrinsic CLI tools are already installed and functional in ./bin/. Skipping download."; \
	else \
		echo "Downloading Intrinsic CLI tools (version ${SDK_VERSION})..."; \
		wget -q -O ./bin/inbuild \
			https://github.com/intrinsic-ai/sdk/releases/download/${SDK_VERSION}/inbuild-linux-amd64; \
		wget -q -O ./bin/inctl \
			https://github.com/intrinsic-ai/sdk/releases/download/${SDK_VERSION}/inctl-linux-amd64; \
		chmod +x ./bin/inbuild ./bin/inctl; \
		if ./bin/inbuild --help >/dev/null 2>&1 && \
		   ./bin/inctl --help >/dev/null 2>&1; then \
			echo "Download complete! Tools verified successfully."; \
		else \
			echo "Error: Downloaded tools are corrupted or incompatible."; \
			rm -f ./bin/inbuild ./bin/inctl; \
			exit 1; \
		fi; \
	fi

docker_setup:
	mkdir -p images/
	docker buildx inspect --builder container-builder || \
		docker buildx create --name="container-builder" --driver="docker-container"

# ==============================================================================
# ==============================================================================
# Service Targets (Consolidated: moveit_planning_service)
# ==============================================================================

# MoveIt Planning Service (Unified planning, scene bridge, and TF streaming)
moveit_planning_service: docker_setup check-intrinsic-env
	docker buildx build $(CACHE_FLAGS) -t moveit_planning_service:latest \
		--builder="container-builder" \
		--output="\
			type=docker,\
			dest=./images/moveit_planning_service.tar,\
			compression=zstd,\
			push=false,\
			name=moveit_planning_service:latest" \
		--build-arg ROS_PACKAGE_NAME_ARG=moveit_planning_service \
		--build-arg ROS_DISTRO=${ROS_DISTRO} \
		--file ./Dockerfile.service \
		.

moveit_planning_service_proto_desc: moveit_planning_service
	docker load -i ./images/moveit_planning_service.tar
	docker create --name temp_container "moveit_planning_service:latest"
	docker cp "temp_container:/opt/service_workspace/install/share/moveit_planning_service/moveit_planning_service_protos.desc" "./images/moveit_planning_service_protos.desc"
	docker cp "temp_container:/opt/intrinsic/intrinsic_sdk_cmake/install/share/intrinsic_sdk_cmake/intrinsic_proto.desc" "./images/intrinsic_proto.desc"
	docker rm -f "temp_container"
	docker rmi "moveit_planning_service:latest"

moveit_planning_service.bundle.tar: moveit_planning_service_proto_desc download_inbuild_and_inctl
	./bin/inbuild service bundle \
		--manifest ./moveit_planning_service/moveit_planning_service.manifest.textproto \
		--oci_image ./images/moveit_planning_service.tar \
		--default_config ./moveit_planning_service/moveit_planning_service_default_config.pbtxt \
		--file_descriptor_set ./images/moveit_planning_service_protos.desc \
		--file_descriptor_set ./images/intrinsic_proto.desc \
		--output ./images/moveit_planning_service.bundle.tar

# Planning Service Installation Targets
install_moveit_planning_service: check-intrinsic-env moveit_planning_service.bundle.tar
	./bin/inctl asset install \
		--org ${INTRINSIC_ORGANIZATION} \
		--cluster ${INTRINSIC_CLUSTER} \
		./images/moveit_planning_service.bundle.tar

add_moveit_planning_service: check-intrinsic-env
	./bin/inctl service add \
		--org ${INTRINSIC_ORGANIZATION} \
		--cluster ${INTRINSIC_CLUSTER} \
		ai.intrinsic.moveit_planning_service --name=moveit_planning_service

delete_moveit_planning_service: check-intrinsic-env
	./bin/inctl service delete \
		--org ${INTRINSIC_ORGANIZATION} \
		--cluster ${INTRINSIC_CLUSTER} \
		moveit_planning_service

uninstall_moveit_planning_service: check-intrinsic-env
	./bin/inctl asset uninstall \
		--org ${INTRINSIC_ORGANIZATION} \
		--cluster ${INTRINSIC_CLUSTER} \
		ai.intrinsic.moveit_planning_service

# ==============================================================================
# Skill Targets
# ==============================================================================

# MoveIt Plan Motion Skill
moveit_plan_motion_skill: docker_setup check-intrinsic-env
	docker buildx build $(CACHE_FLAGS) -t moveit_plan_motion_skill:latest \
		--builder="container-builder" \
		--output="type=docker,dest=./images/moveit_plan_motion_skill.tar,compression=zstd,push=false,name=moveit_plan_motion_skill:latest" \
		--build-arg ROS_DISTRO=${ROS_DISTRO} \
		--build-arg SKILL_PACKAGE=moveit_plan_motion_skill \
		--build-arg SKILL_EXECUTABLE_NAME=moveit_plan_motion_skill_main \
		--build-arg SKILL_CONFIG=share/moveit_plan_motion_skill/moveit_plan_motion_skill_config.pbbin \
		--file ./Dockerfile.skill \
		.

moveit_plan_motion_skill_proto_desc: moveit_plan_motion_skill
	docker load -i ./images/moveit_plan_motion_skill.tar
	docker create --name temp_container_skills "moveit_plan_motion_skill:latest"
	docker cp "temp_container_skills:/opt/intrinsic/intrinsic_sdk_cmake/install/share/intrinsic_sdk_cmake/intrinsic_proto.desc" "./images/intrinsic_proto.desc"
	docker rm -f "temp_container_skills"
	docker run --rm -v $(CURDIR):/ws -w /ws "moveit_plan_motion_skill:latest" \
		protoc -I./moveit_plan_motion_skill/src/moveit_plan_motion_skill \
		--descriptor_set_in=./images/intrinsic_proto.desc \
		--descriptor_set_out=./images/moveit_plan_motion_skill_protos.desc \
		./moveit_plan_motion_skill/src/moveit_plan_motion_skill/moveit_plan_motion_skill.proto
	docker rmi "moveit_plan_motion_skill:latest"

moveit_plan_motion_skill.bundle.tar: moveit_plan_motion_skill_proto_desc download_inbuild_and_inctl
	./bin/inbuild skill manifest \
		--manifest ./moveit_plan_motion_skill/src/moveit_plan_motion_skill/moveit_plan_motion_skill.manifest.textproto \
		--file_descriptor_sets ./images/moveit_plan_motion_skill_protos.desc,./images/intrinsic_proto.desc \
		--output ./images/moveit_plan_motion_skill_augmented_manifest.pbbin \
		--file_descriptor_set_out ./images/moveit_plan_motion_skill_augmented_protos.desc
	./bin/inbuild skill bundle \
		--augmented_manifest ./images/moveit_plan_motion_skill_augmented_manifest.pbbin \
		--augmented_file_descriptor_set ./images/moveit_plan_motion_skill_augmented_protos.desc \
		--oci_image ./images/moveit_plan_motion_skill.tar \
		--output ./images/moveit_plan_motion_skill.bundle.tar

# MoveIt Plan Grasp Skill
moveit_plan_grasp_skill: docker_setup check-intrinsic-env
	docker buildx build $(CACHE_FLAGS) -t moveit_plan_grasp_skill:latest \
		--builder="container-builder" \
		--output="type=docker,dest=./images/moveit_plan_grasp_skill.tar,compression=zstd,push=false,name=moveit_plan_grasp_skill:latest" \
		--build-arg ROS_DISTRO=${ROS_DISTRO} \
		--build-arg SKILL_PACKAGE=moveit_plan_grasp_skill \
		--build-arg SKILL_EXECUTABLE_NAME=moveit_plan_grasp_skill_main \
		--build-arg SKILL_CONFIG=share/moveit_plan_grasp_skill/moveit_plan_grasp_skill_config.pbbin \
		--file ./Dockerfile.skill \
		.

moveit_plan_grasp_skill_proto_desc: moveit_plan_grasp_skill
	docker load -i ./images/moveit_plan_grasp_skill.tar
	docker create --name temp_container_skills "moveit_plan_grasp_skill:latest"
	docker cp "temp_container_skills:/opt/intrinsic/intrinsic_sdk_cmake/install/share/intrinsic_sdk_cmake/intrinsic_proto.desc" "./images/intrinsic_proto.desc"
	docker rm -f "temp_container_skills"
	docker run --rm -v $(CURDIR):/ws -w /ws "moveit_plan_grasp_skill:latest" \
		protoc -I./moveit_plan_grasp_skill/src/moveit_plan_grasp_skill \
		--descriptor_set_in=./images/intrinsic_proto.desc \
		--descriptor_set_out=./images/moveit_plan_grasp_skill_protos.desc \
		./moveit_plan_grasp_skill/src/moveit_plan_grasp_skill/moveit_plan_grasp_skill.proto
	docker rmi "moveit_plan_grasp_skill:latest"

moveit_plan_grasp_skill.bundle.tar: moveit_plan_grasp_skill_proto_desc download_inbuild_and_inctl
	./bin/inbuild skill manifest \
		--manifest ./moveit_plan_grasp_skill/src/moveit_plan_grasp_skill/moveit_plan_grasp_skill.manifest.textproto \
		--file_descriptor_sets ./images/moveit_plan_grasp_skill_protos.desc,./images/intrinsic_proto.desc \
		--output ./images/moveit_plan_grasp_skill_augmented_manifest.pbbin \
		--file_descriptor_set_out ./images/moveit_plan_grasp_skill_augmented_protos.desc
	./bin/inbuild skill bundle \
		--augmented_manifest ./images/moveit_plan_grasp_skill_augmented_manifest.pbbin \
		--augmented_file_descriptor_set ./images/moveit_plan_grasp_skill_augmented_protos.desc \
		--oci_image ./images/moveit_plan_grasp_skill.tar \
		--output ./images/moveit_plan_grasp_skill.bundle.tar

# Skill Installation Targets
install_moveit_plan_motion_skill: check-intrinsic-env moveit_plan_motion_skill.bundle.tar
	./bin/inctl asset install \
		--org ${INTRINSIC_ORGANIZATION} \
		--cluster ${INTRINSIC_CLUSTER} \
		./images/moveit_plan_motion_skill.bundle.tar

uninstall_moveit_plan_motion_skill: check-intrinsic-env
	./bin/inctl asset uninstall \
		--org ${INTRINSIC_ORGANIZATION} \
		--cluster ${INTRINSIC_CLUSTER} \
		ai.intrinsic.moveit_plan_motion_skill

install_moveit_plan_grasp_skill: check-intrinsic-env moveit_plan_grasp_skill.bundle.tar
	./bin/inctl asset install \
		--org ${INTRINSIC_ORGANIZATION} \
		--cluster ${INTRINSIC_CLUSTER} \
		./images/moveit_plan_grasp_skill.bundle.tar

uninstall_moveit_plan_grasp_skill: check-intrinsic-env
	./bin/inctl asset uninstall \
		--org ${INTRINSIC_ORGANIZATION} \
		--cluster ${INTRINSIC_CLUSTER} \
		ai.intrinsic.moveit_plan_grasp_skill

# Backward-compatibility aliases
install_motion_planning_skill: install_moveit_plan_motion_skill
uninstall_motion_planning_skill: uninstall_moveit_plan_motion_skill
install_grasp_planning_skill: install_moveit_plan_grasp_skill
uninstall_grasp_planning_skill: uninstall_moveit_plan_grasp_skill

clean:
	rm -rf bin/ images/
