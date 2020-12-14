#include "comm.h"
#include "config.h"
#include "console.h"
#include "radiolink.h"
#include "usblink.h"

static bool isInit;

void commInit(void)
{
	if (isInit) return;
	radiolinkInit();	/*无线通信初始化*/
//  usblinkInit();		/*USB通信初始化*/
	isInit = true;
}

bool commTest(void)
{
  bool pass=isInit;
  
  pass &= consoleTest();
  
  return pass;
}

