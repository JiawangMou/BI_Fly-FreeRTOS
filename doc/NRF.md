# atkp_t 数据格式
>  uint8_t msgID;  
>  uint8_t dataLen;  
>  uint8_t data[30];  

_**以下介绍均为data数据段的解释说明**_  

## 上行  
>**上行发送时会对数据包加上*包头*和*校验标志***   
>* 包头UP_BYTE1 (0xAA)和UP_BYTE2(0xAA)  
>* 校验标志为所有数据的数据的和(包括数据包头)  

### msgID:UP_REMOTER(0x50)(上传各模块状态)

| [0]| [1]| [2]| [3]| [4]| [5]| [6]| [7]-[10] | [11]-[14] |  
|---|---|---|---|---|---|---|---|---|  
|ACK_MSG(0x01)|硬件版本|MPU自检状态|气压计自检状态|是否校准完成|是否低电量|模块ID|roll微调|pitch微调|

### msgID:UP_STATUS(0x01)(上传飞行器状态)
|[0]-[1]|[2]-[3]|[4]-[5]|[6]-[9]|[10]|[11]|  
|---|---|---|---|---|---|  
|roll|pitch|yaw|气压计数据|0(fly_mode)|false(flyable)|  
>数据调整说明：
>* roll,pitch,yaw均*100  
>* 所有数据进行了一次大小端转换  

### msgID:UP_SENSER(0x02)(上传传感器原始数据)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |[18]-[19]|  
|---|---|---|---|---|---|---|---|---|---|  
|加速度计.X|加速度计.Y|加速度计.Z|陀螺仪.X|陀螺仪.Y|陀螺仪.Z|磁力计.X|磁力计.Y|磁力计.Z| 0 |  
>数据调整说明:
>* 所有数据进行了大小端转换  

### msgID:UP_USER_DATA1(0xF1)(上传用户数据)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|acc.x|acc.y|acc.z|vel.x|vel.y|vel.z|pos.x|pox.y|pox.z|  
>数据调整说明:
>* 所有数据进行了大小端转换   

### msgID:UP_USER_DATA2(0xF2)(上传用户数据)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|光流测速.X|光流测速.Y|光流累计位移.X|光流累计位移.Y|0|融合后高度|激光器测量高度|100*光流数据置信度|基础油门|  
>数据调整说明:  
>* 所有数据进行了大小端转换  
>* 注意光流数据是在发送函数位做的处理  
>* 基础油门也是在传参前做了一次0.1* configParam.thrustBase  

### msgID:UP_RCDATA(0x03)(上传遥控器控制数据)   
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |[18]-[19]|  
|---|---|---|---|---|---|---|---|---|---|  
|油门|yaw|roll|pitch|0|0|0|0|0|0|  
>数据调整说明:
>* 所有数据进行了大小端转换  

### msgID:UP_POWER(0x05)(上传电池状态)  
|[0]-[1]|[2]-[3]|   
|---|---|  
|电池电压*100|500|  
>数据调整说明:
>* 所有数据进行了大小端转换  
>* 注意电压在发送函数传参时做了*100调整  
>* 500为固定参数传入  

### msgID:UP_MOTOR(0x06)(上传电机的PWM占空比)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |
|---|---|---|---|---|---|---|---|
|M1-PWM|M2-PWM|M3-PWM|M4-PWM|0|0|0|0| 
>数据调整说明:  
>* 所有数据进行了大小端转换    
>* 传输前,占空比传参为 pwm/65535*1000    

### msgID:UP_SENSER2(0x07)(上传气压计数据)  
|[0]-[3]|[4]-[5]| 
|---|---|  
|u32的气压计数据|0|  
>数据调整说明:
>* 所有数据进行了大小端转换  

### msgID:UP_PID1(0x10)(上传姿态角速率PID参数)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |  
|---|---|---|---|---|---|---|---|---|
|rollRate.kp|rollRate.ki|rollRate.kd|ptichRate.kp|ptichRate.ki|  
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |   |  
|ptichRate.kd|yawRate.kp|yawRate.ki|yawRate.kd|  |  
>数据调整说明:  
>* 所有数据进行了大小端转换  
>* 所有数据*10扩大  

### msgID:UP_PID2(0x11)(上传姿态角度环PID参数)  
 |[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9]|    
|---|---|---|---|---|---|---|---|---|
|rollAngle.kp|rollAngle.ki|rollAngle.kd|ptichAngle.kp|ptichAngle.ki|  
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  |  
| ptichAngle.kd|yawAngle.kp|yawAngle.ki|yawAngle.kd|  
>数据调整说明:  
>* 所有数据进行了大小端转换  
>* 所有数据*10扩大  

### msgID:UP_PID3(0x12)(上传位置环PID参数)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|pidVZ.kp|pidVZ.ki|pidVZ.kd|pidZ.kp|pidZ.ki|pidZ.kd|pidVX.kp|pidVX.ki|pidVX.kd|  
>数据调整说明:  
>* 所有数据进行了大小端转换  
>* 所有数据*10扩大  

### msgID:UP_PID4(0x13)(上传位置环PID参数)  
 |[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|pidX.kp|pidX.ki|pidX.kd|0|0|0|0|0|0|  
>数据调整说明:  
>* 所有数据进行了大小端转换  
>* 所有数据*10扩大  

### msgID:UP_CHECK(0xEF)(上传数据接收的检查结果)  
| [0] |[1]|  
|---|---|  
|接受的atpk包的msgID|检查结果|   

## 下行  
>**下行发送时会对数据包加上*包头*和*校验标志***   
>* 包头UP_BYTE1 (0xAA)和UP_BYTE2(0xAF)  
>* 校验标志为所有数据的数据的和(包括数据包头)  

### msgID:DOWN_POWER(0x05)(发送电池状态给STM32)  
|[0]:2|[0]:1|[1]-[4]|  
|---|---|---|    
|是否USB供电|充电状态|电池电量|  

### msgID:DOWN_REMOTOR(0x50)(发送遥控数据)  
|[0]|[1]-[4]|[5]-[8]|[9]-[12]|[13]-[16]|    
|---|---|  
|REMOTOR_DATA(0x01)数据标志|float roll|float pitch|float yaw|float 油门|  
|[17]-[20]|[21]-[24]|[25]|[26]|[27]|  
|float trimPitch|float trimRoll|ctrlMode控制模式|flightMode飞行模式|RCLock|  

### msgID:DOWN_REMOTOR(0x50)(发送遥控指令)  
|[0]|[1]|[2]|    
|---|---|---|  
|REMOTOR_CMD(0x00)|指令|data|  
>指令定义如下:
>* #define  CMD_GET_MSG		    0x01	/*获取四轴信息（自检）*/  
>* #define  CMD_GET_CANFLY		0x02	/*获取四轴是否能飞*/  
>* #define  CMD_FLIGHT_LAND	    0x03	/*起飞、降落*/  
>* #define  CMD_EMER_STOP		0x04	/*紧急停机*/  
>* #define  CMD_FLIP			0x05	/*4D翻滚*/  
>* #define  CMD_POWER_MODULE	0x06	/*打开关闭扩展模块电源*/  
>* #define  CMD_LEDRING_EFFECT	0x07	/*设置RGB灯环效果*/  
>* #define  CMD_POWER_VL53LXX	0x08	/*打开关闭激光*/  

### msgID:DOWN_RCDATA(0x03) (发送遥感数据)  
|[0]-[1]|[2]-[3]|[4]-[5]|[6]-[7]|  
|---|---|---|---|  
|roll|pitch|yaw|thrust|  
>和上行不同,没有进行大小端调整  

### msgID:DOWN_CONMAND(0x01) (发送指令)  
|[0]|  
|---|  
|一堆无效指令|  
>指令定义如下
>* #define  D_COMMAND_ACC_CALIB		    0x01
>* #define  D_COMMAND_GYRO_CALIB		0x02
>* #define  D_COMMAND_MAG_CALIB		    0x04
>* #define  D_COMMAND_BARO_CALIB		0x05
>* #define  D_COMMAND_ACC_CALIB_EXIT	0x20
>* #define  D_COMMAND_ACC_CALIB_STEP1	0x21
>* #define  D_COMMAND_ACC_CALIB_STEP2	0x22
>* #define  D_COMMAND_ACC_CALIB_STEP3	0x23
>* #define  D_COMMAND_ACC_CALIB_STEP4	0x24
>* #define  D_COMMAND_ACC_CALIB_STEP5	0x25
>* #define  D_COMMAND_ACC_CALIB_STEP6	0x26
>* #define  D_COMMAND_FLIGHT_LOCK		0xA0    (这两个是有用的)
>* #define  D_COMMAND_FLIGHT_ULOCK		0xA1    (这两个是有用的)

###　msgID:DOWN_ACK(0x02)  
|[0]|  
|---|  
|指令|  
>指令定义如下    
>* #define  D_ACK_READ_PID				0x01  
>* #define  D_ACK_READ_VERSION			0xA0  
>* #define  D_ACK_RESET_PARAM			0xA1  

### 　msgID:DOWN_PID1(0x10)(发送姿态角速度环PID参数)
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |  
|---|---|---|---|---|---|---|---|---|
|rollRate.kp|rollRate.ki|rollRate.kd|ptichRate.kp|ptichRate.ki|  
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |   |  
|ptichRate.kd|yawRate.kp|yawRate.ki|yawRate.kd|  |  

### msgID:DOWN_PID2(0x11)(发送姿态角度环PID参数)  
 |[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9]|    
|---|---|---|---|---|---|---|---|---|
|rollAngle.kp|rollAngle.ki|rollAngle.kd|ptichAngle.kp|ptichAngle.ki|  
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  |  
| ptichAngle.kd|yawAngle.kp|yawAngle.ki|yawAngle.kd|  


### msgID:DOWN_PID3(0x12)(发送位置环PID参数)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|pidVZ.kp|pidVZ.ki|pidVZ.kd|pidZ.kp|pidZ.ki|pidZ.kd|pidVX.kp|pidVX.ki|pidVX.kd|  


### msgID:DOWN_PID4(0x13)(发送位置环PID参数)  
 |[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|pidX.kp|pidX.ki|pidX.kd|0|0|0|0|0|0|  

### msgID:DOWN_PID5(0x14)(只要回传,发送全0)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|0|0|0|0|0|0|0|0|0|  

### msgID:DOWN_PID6(0x15)(只要回传,发送全0)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|0|0|0|MOTOR.Enable|M1.PWM|M2.PWM|M3.PWM|M4.PWM|0|  
