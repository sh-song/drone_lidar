docker run --rm -it \
-v $(pwd)/workspace:/workspace \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=unix$DISPLAY \
--net=host \
--privileged \
--name obs_tracker \
obs_tracker 
