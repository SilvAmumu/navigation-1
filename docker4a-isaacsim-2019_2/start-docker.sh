#docker run --rm -it --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix randaz81/r1slam:isaac-nvidia 
#docker run --rm -it --gpus all -e QT_X11_NO_MITSHM=1 -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e TZ=Europe/Rome  --hostname dockerpc randaz81/r1slam:isaac-nvidia 
nvidia-docker run --rm -it --gpus all -e QT_X11_NO_MITSHM=1 -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics --hostname dockerpc randaz81/isaac:src-2019_2
