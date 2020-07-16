## 上行  
>**上行数据包格式为: *包头*(2字节)+*ID*(1字节)+*长度*(1字节)+*数据段*(*长度*字节)+*校验标志*(1字节)**   
>* 包头UP_BYTE1 (0xAA)和UP_BYTE2(0xAA)  
>* 校验标志为所有数据的数据的和(包括数据包头)  

### msgID:UP_STATUS(0x01)(上传飞行器状态)
|[0]-[1]|[2]-[3]|[4]-[5]|[6]-[9]|[10]|[11]|  
|---|---|---|---|---|---|  
|(int16)Roll|(int16)Pitch|(int16)Yaw|(int32)海拔高度|(u8)FLY_MODE|(u8)IS_ARMED|
> **数据调整说明：**
>* roll,pitch,yaw均*100(角度制)
>* 以mm为单位
>* 发送顺序高八位在前

### msgID:UP_SENSER(0x02)(上传传感器原始数据)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |
|---|---|---|---|---|---|---|---|---|---|  
|(int16)acc.X|(int16)acc.Y|(int16)acc.Z|(int16)gyro.X|(int16)gyro.Y|(int16)gyro.Z|(int16)mag.X|(int16)mag.Y|(int16)mag.Z|
> **数据调整说明：**
>* 发送顺序高八位在前

### msgID:UP_RCDATA(0x03)(上传遥控器控制数据)
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |
|---|---|---|---|
|(int16)thrust|(int16)yaw|(int16)roll|(int16)pitch|
>数据调整说明:
>* 发送顺序高八位在前
>* 四通道×500扩大

### msgID:UP_POWER(0x05)(上传电池状态)  
|[0]-[1]|[2]-[3]|   
|---|---|  
|(uint16)电压|(uint16)电流|
>数据调整说明:
>* 发送顺序高八位在前
>* 电压×100扩大  
>* 电流原协议没有

### msgID:UP_MOTOR(0x06)(上传电机的PWM占空比)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |
|---|---|---|---|---|---|---|---|
|(uint16)左电机|(uint16)右电机|(uint16)左舵机|(uint16)右舵机|
>数据调整说明:
>* 发送顺序高八位在前
>* 发送数据调整pwm/65535*1000

### msgID:UP_SENSER2(0x07)(上传气压计数据)  
|[0]-[3]|[4]-[5]| 
|---|---|  
|(uint32)气压计数据|0|  
>数据调整说明:
>* 发送顺序高八位在前

### msgID:UP_USER_DATA2(0xF2)(光流调试数据)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|光流测速.X|光流测速.Y|光流累计位移.X|光流累计位移.Y|0|融合后高度|激光器测量高度|100*光流数据置信度|基础油门|  
>数据调整说明:  
>* 发送顺序高八位在前
>* 注意光流数据是在发送函数位做的处理
>* 基础油门也是在传参前做了一次0.1* configParam.thrustBase

### msgID:UP_PID1(0x10)(上传姿态角速率PID参数)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |  
|---|---|---|---|---|---|---|---|---|
|(int16)rollRate.kp|(int16)rollRate.ki|(int16)rollRate.kd|(int16)pitchRate.kp|(int16)pitchRate.ki|  
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |   |  
|(int16)pitchRate.kd|(int16)yawRate.kp|(int16)yawRate.ki|(int16)yawRate.kd|  |  
>数据调整说明:  
>* 发送顺序高八位在前  
>* 所有数据*10扩大  

### msgID:UP_PID2(0x11)(上传姿态角度环PID参数)  
 |[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9]|    
|---|---|---|---|---|---|---|---|---|
|(int16)rollAngle.kp|(int16)rollAngle.ki|(int16)rollAngle.kd|(int16)pitchAngle.kp|(int16)pitchAngle.ki|  
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  |  
|(int16)pitchAngle.kd|(int16)yawAngle.kp|(int16)yawAngle.ki|(int16)yawAngle.kd|  
>数据调整说明:  
>* 发送顺序高八位在前
>* 所有数据*10扩大

### msgID:UP_PID5(0x14)(上传姿态角速度环PID上下限)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |
|---|---|---|---|---|---|---|---|---|
|(int16)rollRate.ub|(int16)rollRate.lb|(int16)pitchRate.ub|(int16)pitchRate.ub|(int16)yawRate.ub|
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |
|(int16)yawRate.lb|0|0|0|
>数据调整说明:
>* 发送顺序高八位在前
>* 所有数据*10扩大

### msgID:UP_PID6(0x15)(上传姿态角度环PID上下限)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |
|---|---|---|---|---|---|---|---|---|
|(int16)rollAngle.ub|(int16)rollAngle.lb|(int16)pitchAngle.ub|(int16)pitchAngle.ub|(int16)yawAngle.ub|
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |
|(int16)yawAngle.lb|0|0|0|
>数据调整说明:  
>* 发送顺序高八位在前
>* 所有数据*10扩大

### msgID:UP_PID3(0x12)(上传位置环PID参数)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|(int16)pidVZ.kp|(int16)pidVZ.ki|(int16)pidVZ.kd|(int16)pidZ.kp|(int16)pidZ.ki|(int16)pidZ.kd|(int16)pidVX.kp|(int16)pidVX.ki|(int16)pidVX.kd|  
>数据调整说明:  
>* 所有数据进行了大小端转换  
>* 所有数据*10扩大  

### msgID:UP_PID4(0x13)(上传位置环PID参数)  
 |[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|(int16)pidX.kp|(int16)pidX.ki|(int16)pidX.kd|0|0|0|0|0|0|  
>数据调整说明:  
>* 所有数据进行了大小端转换  
>* 所有数据*10扩大  

### msgID:UP_CHECK(0xEF)(上传数据接收的检查结果)  
| [0] |[1]|  
|---|---|  
|(u8)收到的msgID|(u8)计算的校验和|   

## 下行  
>**上行数据包格式为: *包头*(2字节)+*ID*(1字节)+*长度*(1字节)+*数据段*(*长度*字节)+*校验标志*(1字节)**   
>* 包头UP_BYTE1 (0xAA)和UP_BYTE2(0xAF)
>* 校验标志为所有数据的数据的和(包括数据包头)

### msgID:DOWN_RCDATA(0x03) (发送遥控数据)  
|[0]-[1]|[2]-[3]|[4]-[5]|[6]-[7]|  
|---|---|---|---|  
|(int16)roll|(int16)pitch|(int16)yaw|(int16)thrust|

### msgID:DOWN_CONMAND(0x01) (发送指令)  
|[0]|  
|---|  
|指令|  
>指令定义如下
>* D_COMMAND_ACC_CALIB		    **0x01**	(校准Acc)
>* D_COMMAND_GYRO_CALIB			**0x02**	(校准Acc)
>* D_COMMAND_MAG_CALIB		    **0x04**	(校准Acc)
>* D_COMMAND_BARO_CALIB			**0x05**	(校准Acc)
>* D_COMMAND_ACC_CALIB_EXIT		**0x20**	(退出6面校准)
>* D_COMMAND_ACC_CALIB_STEP1	**0x21**	(6面校准1)
>* D_COMMAND_ACC_CALIB_STEP2	**0x22**	(6面校准2)
>* D_COMMAND_ACC_CALIB_STEP3	**0x23**	(6面校准3)
>* D_COMMAND_ACC_CALIB_STEP4	**0x24**	(6面校准4)
>* D_COMMAND_ACC_CALIB_STEP5	**0x25**	(6面校准5)
>* D_COMMAND_ACC_CALIB_STEP6	**0x26**	(6面校准6)
>* D_COMMAND_FLIGHT_LOCK		**0xA0**    (解锁)
>* D_COMMAND_FLIGHT_ULOCK		**0xA1**    (锁定)

### msgID:DOWN_ACK(0x02)  
|[0]|  
|---|  
|(u8)指令|  
>指令定义如下    
>* D_ACK_READ_PID				0x01  (读PID请求)
>* D_ACK_READ_VERSION			0xA0  (读版本请求)
>* D_ACK_RESET_PARAM			0xA1  (重置默认参数)

### msgID:DOWN_PID1(0x10)(发送姿态角速度环PID参数)
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |  
|---|---|---|---|---|---|---|---|---|
|(int16)rollRate.kp|(int16)rollRate.ki|(int16)rollRate.kd|(int16)pitchRate.kp|(int16)pitchRate.ki|  
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |   |  
|(int16)pitchRate.kd|(int16)yawRate.kp|(int16)yawRate.ki|(int16)yawRate.kd|  |
>数据调整说明:  
>* 发送顺序高八位在前  
>* 所有数据*10扩大  

### msgID:DOWN_PID2(0x11)(发送姿态角度环PID参数)  
 |[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9]|
|---|---|---|---|---|---|---|---|---|
|(int16)rollAngle.kp|(int16)rollAngle.ki|(int16)rollAngle.kd|(int16)pitchAngle.kp|(int16)pitchAngle.ki|
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  |  
|(int16)pitchAngle.kd|(int16)yawAngle.kp|(int16)yawAngle.ki|(int16)yawAngle.kd|
>数据调整说明:  
>* 发送顺序高八位在前
>* 所有数据*10扩大

### msgID:DOWN_PID5(0x14)(发送姿态角速度环PID上下限)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |
|---|---|---|---|---|---|---|---|---|
|(int16)rollRate.ub|(int16)rollRate.lb|(int16)pitchRate.ub|(int16)pitchRate.ub|(int16)yawRate.ub|
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |
|(int16)yawRate.lb|0|0|0|
>数据调整说明:
>* 发送顺序高八位在前
>* 所有数据*10扩大

### msgID:DOWN_PID6(0x15)(发送姿态角度环PID上下限)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |
|---|---|---|---|---|---|---|---|---|
|(int16)rollAngle.ub|(int16)rollAngle.lb|(int16)pitchAngle.ub|(int16)pitchAngle.ub|(int16)yawAngle.ub|
|[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |
|(int16)yawAngle.lb|0|0|0|
>数据调整说明:  
>* 发送顺序高八位在前
>* 所有数据*10扩大

### msgID:DOWN_PID3(0x12)(发送位置环PID参数)  
|[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|pidVZ.kp|pidVZ.ki|pidVZ.kd|pidZ.kp|pidZ.ki|pidZ.kd|pidVX.kp|pidVX.ki|pidVX.kd|
>数据调整说明:  
>* 发送顺序高八位在前  
>* 所有数据*10扩大  

### msgID:DOWN_PID4(0x13)(发送位置环PID参数)  
 |[0]-[1]|[2]-[3] |[4]-[5] |[6]-[7] |[8]-[9] |[10]-[11] |[12]-[13] |[14]-[15] |[16]-[17] |  
|---|---|---|---|---|---|---|---|---|
|pidX.kp|pidX.ki|pidX.kd|0|0|0|0|0|0|
>数据调整说明:  
>* 发送顺序高八位在前  
>* 所有数据*10扩大  