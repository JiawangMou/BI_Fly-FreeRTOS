#ifndef __SENSFUSION6_H
#define __SENSFUSION6_H
#include "stabilizer_types.h"

typedef struct{
    Axis3f	acc_beforefusion;
    bool    useAcc;
}Acc_Send;

void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt);	/*数据融合 互补滤波*/
bool getIsCalibrated(void);
void imuTransformVectorBodyToEarth(Axis3f * v);	/*机体到地球*/
void imuTransformVectorEarthToBody(Axis3f * v);	/*地球到机体*/
void getAcc_SendData(Acc_Send *acc);            /*发送融合之前加速度的数据*/

#endif

