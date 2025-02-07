source /ros_entrypoint.sh
source ./install/setup.sh

TOPIC_NAME="/dtc/simulation_start"
echo "Waiting for topic '$TOPIC_NAME' to exist..."
while ! ros2 topic list | grep -q "^$TOPIC_NAME\$"; do
    sleep 0.5
done

sleep 1
ros2 topic pub $TOPIC_NAME std_msgs/msg/Empty -1

ros2 run ardupilot_controller controller & 


. ~/.profile
sim_vehicle.py --no-mavproxy -v ArduCopter -f JSON:127.0.0.1 -l $ARDUPILOT_ORIGIN_LAT,$ARDUPILOT_ORIGIN_LON,$ARDUPILOT_STARTING_ALT,$ARDUPILOT_STARTING_YAW --auto-sysid &

sleep 5
/home/dtcbuild/QGroundControl.AppImage &

sleep 10
ros2 topic list
wait