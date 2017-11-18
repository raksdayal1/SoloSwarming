std_msgs/Header       header
float64               Time
utc_time	      UtcTime
uint16                Week
uint8                 GpsFix
uint8                 NumSats
swarm_control/lla     LLA
swarm_control/ned     NED
geometry_msgs/Vector3 NedVel
geometry_msgs/Vector3 NedAcc
float32               SpeedAcc
float32               TimeAcc


uint8 GPSFIX_NO_FIX=0
uint8 GPSFIX_TIME_ONLY=1
uint8 GPSFIX_2D_FIX=2
uint8 GPSFIX_3D_FIX=3
