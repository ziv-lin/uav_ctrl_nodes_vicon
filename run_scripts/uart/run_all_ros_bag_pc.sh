export scripts_wait_time=3

echo "==== Run 1_mocap.sh ===="
./1_mocap.sh  &> log_mocap.txt &
sleep ${scripts_wait_time}

echo "==== Run 2_pos_vel.sh ===="
./2_pos_vel.sh &> log_pos_vel.txt &
sleep ${scripts_wait_time}

echo "==== Run 6_ctrl.sh ===="
./6_uart_odom.sh  &> log_uart_odom.txt &
sleep ${scripts_wait_time}
