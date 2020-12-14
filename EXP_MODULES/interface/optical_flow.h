#ifndef __OPTICAL_FLOW_H
#define __OPTICAL_FLOW_H
#include "sys.h"
#include <stdbool.h>
#include "spi.h"
#include "stabilizer_types.h"
#include "module_mgt.h"
#include "vl53lxx.h"

typedef struct opFlow_s 
{
	float pixSum[2];		/*累积像素*/
	float pixComp[2];		/*像素补偿*/
	float pixValid[2];		/*有效像素*/
	float pixValidLast[2];	/*上一次有效像素*/
	
	float deltaPos[2];		/*2帧之间的位移 单位cm*/
	float deltaVel[2];		/*速度 单位cm/s*/
	float posSum[2];		/*累积位移 单位cm*/
	float velLpf[2];		/*速度低通 单位cm/s*/
	
	bool isOpFlowOk;		/*光流状态*/
	bool isDataValid;		/*数据有效*/

} opFlow_t;

extern opFlow_t opFlow;

void opticalFlowPowerControl(bool state);	//光流电源控制
bool getOpFlowData(state_t *state, float dt);	//读取光流数据
void opticalFlowInit(void);		/*初始化光流模块*/
bool getOpDataState(void);		/*光流数据状态*/
bool getopFlowState(void);		/*光流模块状态*/
void opticalFlowTask(void *param);

#endif
