# UAV_GIMBAL开发

## CMAKE.txt

include_directories(${includes} User/Framework User/Framework/ADRC
User/Framework/Debug User/Framework/Filters User/Framework/Gimbal User/Framework/IMU User/Framework/IWDG
User/Framework/LED User/Framework/PID
User/Framework/RomteC User/Framework/ShootC User/Framework/Vision User/LOG User/MCUDriver/CAN User/MCUDriver/LASER
User/MCUDriver/LED User/MCUDriver/POWER User/Framework/PID_stm32 User/Framework/PosPID_stm32 User/Framework/slprj
User/MCUDriver/REMOTEC User/MCUDriver/SERVO User/Tasks User/ User/MCUDriver/ User/Framework/RGB
User/Framework/Vision User/Framework/IMU/algorithm User/Framework/IMU/bsp User/Framework/IMU/controller User/Framework/IMU/devices User/Framework/IMU/task User/Framework/IMU/Other
User/Framework/Yaw_Matlab_PID User/Framework/Yaw_Matlab_PID/spdLoop User/Framework/Yaw_Matlab_PID/posLoop)
## 
## C板

### **SWD**

![image-20230915164321231](C:\Users\ShiF\AppData\Roaming\Typora\typora-user-images\image-20230915164321231.png)

### **USART**

![image-20230915164210004](C:\Users\ShiF\AppData\Roaming\Typora\typora-user-images\image-20230915164210004.png)

### **CAN**

![image-20230915164355811](C:\Users\ShiF\AppData\Roaming\Typora\typora-user-images\image-20230915164355811.png)

### **PWM**

![image-20230915164543496](C:\Users\ShiF\AppData\Roaming\Typora\typora-user-images\image-20230915164543496.png)

![image-20230915164551012](C:\Users\ShiF\AppData\Roaming\Typora\typora-user-images\image-20230915164551012.png)

### **DBUS**

![image-20230915164616528](C:\Users\ShiF\AppData\Roaming\Typora\typora-user-images\image-20230915164616528.png)

### **IMU 六轴惯性测量单元**

**BMIO88**