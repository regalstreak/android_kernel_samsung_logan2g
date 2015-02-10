 /*******************************************************************************

by jbass.choi@samsung.com 15/JUL/2013

 *******************************************************************************/
 
 /*
by jbass.choi@samsung.com 15/JUL/2013
  */
 #ifndef __LINUX_TPS61158_BL_H
 #define __LINUX_TPS61158_BL_H
extern int is_poweron;
extern int wakeup_brightness;
void ktd_backlight_set_brightness(int level);//temporary added by jbass.choi@samsung.com
static int tps_backlight_set_brightness(int level);
 #endif
