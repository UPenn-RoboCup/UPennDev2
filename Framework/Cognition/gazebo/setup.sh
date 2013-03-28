export GAZEBO_MASTER_URI=http://localhost:11345
export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-1.5:/usr/share/gazebo_models
export GAZEBO_PLUGIN_PATH=/usr/lib/gazebo-1.5/plugins:${PWD}/plugins
export LD_LIBRARY_PATH=/usr/lib/gazebo-1.5/plugins:${LD_LIBRARY_PATH}
export OGRE_RESOURCE_PATH=/usr/lib/i386-linux-gnu/OGRE-1.7.4

# This line is needed while we're relying on ROS's urdfdom library
export LD_LIBRARY_PATH=/opt/ros/fuerte/lib:${LD_LIBRARY_PATH}
