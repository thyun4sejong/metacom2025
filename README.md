

# Build image based on isaac-sim 4.2.0 image provided by nvidia nvcr.io:
```
docker login nvcr.io
docker build --pull -t isaac-sim:4.2.0-ubuntu22.04 \
  --build-arg ISAACSIM_VERSION=4.2.0 \
  --file Dockerfile .
```

# Write simulation app

## simulation code

./simulation_app/codes is for simulation codes. run_simulation_app.py will be called when the container startup time.


## user's custom models 

usd files can be stored on ./simulation_app/models.
you can access models with the path /simulation_app/models under docker container environment.

## user's other files

./simulation_app directory is binded to /simulation_app under docker container environment. use this directory to store user's data, configuration, etc.


# Run container 

below command will launch isaac-sim and execute /simulation_app/codes/run_simulation_app.py
```
docker compose up
```




# Just FYI ...

## launch headless isaac-sim using docker image which is built by ./Dockerfile
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

## launch windowed isaac-sim using docker image which is built by ./Dockerfile
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