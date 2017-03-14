# Joystick publishes commands to named pipes. Motion controller listens for
# movement commands, and Rexarm listens for "pick up object" commands
# (when you run rexarm_listener.sh).

mkfifo a=rw /tmp/pickup_pipe
mkfifo a=rw /tmp/bot_move_pipe

./joystick/joystick
./motion_controller/motion_controller < /tmp/bot_move_pipe

rm /tmp/pickup_pipe
rm /tmp/bot_move_pipe
