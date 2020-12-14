#ifndef __SENSFUSION6_H
#define __SENSFUSION6_H
#include "stabilizer_types.h"

typedef struct{
    Axis3f	acc_beforefusion;
    bool    useAcc;
}Acc_Send;

void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt);	/*�����ں� �����˲�*/
bool getIsCalibrated(void);
void imuTransformVectorBodyToEarth(Axis3f * v);	/*���嵽����*/
void imuTransformVectorEarthToBody(Axis3f * v);	/*���򵽻���*/
void getAcc_SendData(Acc_Send *acc);            /*�����ں�֮ǰ���ٶȵ�����*/

#endif

