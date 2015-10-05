#ifndef __PC_DEBUG_H_
#define __PC_DEBUG_H_

#include <cstdio>

#define PDEBUG(fmt, args...) fprintf (stderr, fmt , ## args)

#endif
