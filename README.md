```
roslaunch cultureid_devel turtlebot_bringup_live.launch
roslaunch cultureid_devel avanti_live.launch
roslaunch cultureid_waypoints_following avanti.launch game_id:=0 map_name:=csal_karto
roslaunch cultureid_tiff22_game avanti.launch game_id:=a
roslaunch cultureid_waypoint_navigation_plugin rviz.launch
```
