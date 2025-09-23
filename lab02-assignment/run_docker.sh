docker run -it --privileged \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev:rw \
    --env=LOCAL_USER_ID="$(id -u)" \
    --hostname $(hostname) \
    --network host \
    --name lab02_container kweontj/lab02:assignment bash
    # 수정 이유 : 도커 파일을 사용되는 컴퓨터의 이름, 폴더명으로 변경
    # 추가적으로 수업 과제와 겹치지 않게 하기위해 스크립트 파일 수정