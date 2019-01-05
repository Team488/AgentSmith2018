#!/bin/bash

#TODO: The service configuration requires that you have the following line in /etc/sudoers:
# nvidia ALL=(ALL) NOPASSWD: /bin/systemctl
# nvidia ALL=(ALL) NOPASSWD: /bin/cp 
# and that key-based SSH is configured.

TARGET_HOST="$1"
TARGET_USER_NAME=nvidia
TARGET_PASSWORD=nvidia
SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DEST_DIR=/home/$TARGET_USER_NAME/catkin_ws/src/robot-odom/
BUILD_DIR=/home/$TARGET_USER_NAME/catkin_ws/

# TODO: Add logic to do a production-ready build, and maybe configure the service 

ENABLE_SERVICE=false
for i in "$@"
do
    case $i in
        -s|--enable-service)
            ENABLE_SERVICE=true
            shift
        ;;
        *)
            # unknown option    
        ;;
    esac
done
DATE=$(date -R)

# TODO: Ping to confirm 
echo "Setting remote date to $DATE"
ssh $TARGET_USER_NAME@$TARGET_HOST << EOF
    sudo date --set "$DATE"
EOF

#exit 0

ssh $TARGET_USER_NAME@$TARGET_HOST mkdir -p "$DEST_DIR"
rsync -azvv -e ssh \
    --exclude=.git \
    --exclude=build \
    --exclude='Visual Studio Solution' \
    --exclude='*~' \
    --exclude='.gitignore' \
    $SOURCE_DIR/ $TARGET_USER_NAME@$TARGET_HOST:$DEST_DIR

ssh $TARGET_USER_NAME@$TARGET_HOST -X << EOF
    export DEBIAN_FRONTEND=noninteractive
    set -e
    
    cd "$BUILD_DIR"
    source "$BUILD_DIR/devel/setup.bash"
    catkin_make
    #-DCMAKE_BUILD_TYPE=Debug
    
    if [ "$ENABLE_SERVICE" = true ]; then
        if [ -f /etc/systemd/system/vision.service ]; then
            sudo systemctl stop vision.service
        fi
        sudo cp "$DEST_DIR/vision.service" "/etc/systemd/system/"
        sudo systemctl enable vision.service
        echo "Enabled systemd job"
    else
        if [ -f /etc/systemd/system/vision.service ]; then
            #sudo systemctl stop vision.service
            sudo systemctl disable vision.service
            echo "Disabled systemd job"
        fi
    fi

    echo "Build succeeded!"
EOF

