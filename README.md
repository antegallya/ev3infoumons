Build the EV3DEV Cross-Compiling image
======================================
We use an qemu armel image to cross-compile. It's damn slow, but it's easier.
Go to docker/ev3dev-ros-armel-cross/
docker build -t ev3dev-ros-armel-cross .

EV3 target image
================
The docker recipe is in "docker/ev3dev-ros-ev3/Dockerfile". Follow the ev3dev
brickstrap instructions to build the target image using this docker recipe..
