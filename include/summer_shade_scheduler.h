#ifndef SUMMER_SHADE_SCHEDULER_H
#define SUMMER_SHADE_SCHEDULER_H

#include <user_config.h>

#if defined(SUMMER_SHADE_AUTOMATION)
void initSummerShadeScheduler();
#else
inline void initSummerShadeScheduler() {}
#endif

#endif // SUMMER_SHADE_SCHEDULER_H
