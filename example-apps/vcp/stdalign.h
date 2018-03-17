#ifndef STDALIGN_H_
#define STDALIGN_H_

/* workaround for missing stdalign.h */

#define alignas(x) __attribute__ ((aligned (x)))

#endif
