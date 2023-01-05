# edge-ai-ros-dlp

DLP Controller code: to automatically update the DLP while running the inventory demo.
For details on how it all works, check the in-code documentation.

To test the DLP on your robot, run:
```
rosrun d3_dlp dlp_ros.py
rosrun d3_dlp dlp_test.py
```
Then enter either W,A,S,D to send direction commands, or Q/E to send SCANNING/STOP.
The DLP should update immediately after typing a letter and pressing enter.

