export scripts_wait_time=3

echo "==== Run 4_djiros.sh ===="
#./4_djiros.sh &> log_djiros.txt &
./4_djiros.sh &
sleep ${scripts_wait_time}

echo "==== Run 1_mocap.sh ===="
./1_mocap.sh &> log_mocap.txt &
sleep ${scripts_wait_time}

echo "==== Run 3_ctrl.sh ===="
./3_ctrl.sh &> log_ctrl.txt &
sleep ${scripts_wait_time}

echo "==== Run 6_ctrl.sh ===="
./6_uart_odom.sh  &> log_uart_odom.txt &
sleep ${scripts_wait_time}
