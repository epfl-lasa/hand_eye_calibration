    
    
# Setup a shared volume between docker and host. Volume is named "my volume"
# Setup a shared volume between docker and host. Volume is named "calibration"
docker volume rm calibration  # First delete any volume with that name

# Then create a new volume
docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/../src/" \
    --opt o="bind" \
    "calibration"
        
FWD_ARGS+=(--volume="calibration:/home/ros/ros_ws/src:rw")
FWD_ARGS+=(--privileged)

# network for ros
FWD_ARGS+=(--net host)
FWD_ARGS+=(--env ROS_HOSTNAME="$(hostname)")
#HOST_IP=$(hostname -I | cut -d' ' -f1)
FWD_ARGS+=(--env ROS_IP="$ROS_IP")

aica-docker interactive calibration:${ROS_DISTRO} "${FWD_ARGS[@]}"