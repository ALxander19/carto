# carto

# generamos el pbstream a partir del bag
roslaunch testcarto offline_2d.launch bag_filenames:=${HOME}/sub.bag

# generamos el ply a partir del bag y el pbstream
roslaunch testcarto writer_2d.launch bag_filenames:=${HOME}/subset.bag pose_graph_filename:=${HOME}/subset.bag.pbstream

######### HUSKY ##########

roslaunch husky_gazebo husky_playpen.launch

rosbag record -O subset /velodyne_points /imu /odometry_filtered

rosrun xacro xacro husky.urdf.xacro > husky.urdf

# generamos el pbstream a partir del bag
roslaunch husky_carto offline_3d.launch bag_filenames:=${HOME}/subset.bag

# generamos el ply a partir del bag y el pbstream
roslaunch husky_carto writer_3d.launch bag_filenames:=${HOME}/subset.bag pose_graph_filename:=${HOME}/subset.bag.pbstream

######### HECTOR DRONE ##########

roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch

rosbag record -O subset /scan /raw_imu /fix

roscd hector_quadrotor_description/urdf
rosrun xacro xacro quadrotor_hokuyo_utm30lx.gazebo.xacro > quadrotor_hokuyo_utm30lx.urdf

# generamos el pbstream a partir del bag
roslaunch hector_carto offline_2d.launch bag_filenames:=${HOME}/subset.bag

# generamos el ply a partir del bag y el pbstream
roslaunch hector_carto writer_2d.launch bag_filenames:=${HOME}/subset.bag pose_graph_filename:=${HOME}/subset.bag.pbstream

######### OS1-64 ##########

roscd ouster_description/urdf
rosrun xacro xacro example.urdf.xacro > os1_sensor.urdf

# generamos el pbstream a partir del bag
roslaunch testcarto offline_cart_3d.launch bag_filenames:=${HOME}/primervuelo.bag

# generamos el ply a partir del bag y el pbstream
roslaunch testcarto assets_writer_cart_3d.launch bag_filenames:=${HOME}/primervuelo.bag pose_graph_filename:=${HOME}/primervuelo.bag.pbstream

########## Revisar bag ###########

cartographer_rosbag_validate -bag_filename primervuelo.bag
