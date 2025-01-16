

# Build the image:
```
docker login nvcr.io
docker build --pull -t \
  isaac-sim:4.2.0-ubuntu22.04 \
  --build-arg ISAACSIM_VERSION=4.2.0 \
  --file Dockerfile .
```

# Run container 
## windowed, docker compose
```
docker compose up
```

## Alternatives #1 headless app:
```
docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
  -e "PRIVACY_CONSENT=Y" -e "PRIVACY_USERID=<email>" \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
	isaac-sim:4.2.0-ubuntu22.04 \
	./isaac-sim.headless.native.sh --allow-root
```

## Alternatives #2 windowed app:
```
xhost +localhost:
docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
  -e "PRIVACY_CONSENT=Y" \
  -e "EXTENSIONS=omni.isaac.ros2_bridge" \
  -e "ROS_DISTRO=humble" \
  -e "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  -e "LD_LIBRARY_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib:$LD_LIBRARY_PATH" \
  -e "AMENT_PREFIX_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/humble" \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -e DISPLAY \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  -v ./workspace:/workspace:rw \
  isaac-sim:4.2.0-ubuntu22.04 \
  ./runapp.sh
```