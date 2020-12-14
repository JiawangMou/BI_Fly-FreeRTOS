#ifndef __EXP_MODULE_H
#define __EXP_MODULE_H
#include "sys.h"
#include "module_detect.h"

enum expModuleID getModuleID(void);
void expModulePower(bool state);
void expModuleMgtTask(void* param);


#endif /* __EXP_MODULE_H */

