# uav_ctrl_nodes_vicon

基于老司机@刘天博 的基础上改进的 n1ctrl & dji_ros
1. 添加角速度环，突破dji N3 35度的限制(兼容原来的n3姿态角控制，通过修改yaml中switch_angle的大小可以切换控制模式)
2. 解决姿态转换出现的singular point的问题
3. 限制控制器的输入误差
4. 可以通过ros消息设置控制器的角度限制

uart_odom
为了规避无线通信使用wifi时出现的干扰问题，做的一个使用无线桥接vicon message的一个方案, 实测PC和TX2之间的收发vicon的通信频率能达到 260Hz
通信链路：
vicon -> PC -> uart -> UWB (or 933Mhz)
UWB (or 933Mhz) -> uart -> Onboard computer

mocap_optitrack & pos_vel_mocap
1. 修复vicon频率高于50hz 速度差分不准确的问题
