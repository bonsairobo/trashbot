# Kinect publishes data about best grippable object to named pipe.
# With feedback from Kinect, Rexarm attempts to grab object upon request
# from Bluetooth Joystick.

if [-e /tmp/pickup_pipe] then
	mkfifo a=rw /tmp/object_pipe

	./kinect 0 > /tmp/object_pipe
	./rexarm

	rm /tmp/object_pipe
fi
