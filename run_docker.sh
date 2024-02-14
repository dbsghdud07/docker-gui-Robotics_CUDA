#!/bin/bash
# 첫 번째 인자로부터 이미지 이름을 받음
DOCKER_IMAGE=$1

# 두 번째 인자부터 나머지 모든 인자를 추가 옵션으로 처리
shift  # 첫 번째 인자 제거
DOCKER_OPTIONS="$@"

echo "docker run -d $DOCKER_OPTIONS $DOCKER_IMAGE bash"
docker run -d $DOCKER_OPTIONS $DOCKER_IMAGE bash