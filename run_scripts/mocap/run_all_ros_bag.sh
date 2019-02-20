export scripts_wait_time=3

echo "==== Run 4_djiros.sh ===="
./4_djiros.sh &
sleep ${scripts_wait_time}

echo "==== Run 1_gps.sh ===="
./1_mocap.sh &> log_mocap.txt &
sleep ${scripts_wait_time}

echo "==== Run 1_gps.sh ===="
./2_pos_vel.sh &> log_pos_vel.txt &
sleep ${scripts_wait_time}

echo "==== Run 3_ctrl.sh ===="
./3_ctrl.sh >& log_ctrl_md.txt #&
#sleep ${scripts_wait_time}

