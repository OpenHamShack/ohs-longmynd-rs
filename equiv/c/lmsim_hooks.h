/* Forced into every file of the reference build (-include): the C's two
 * calls into time, usleep() and clock_gettime(), go to the simulator's
 * virtual clock instead. The system headers are included first, under
 * their real names, so only longmynd's own calls are renamed. */
#ifndef LMSIM_HOOKS_H
#define LMSIM_HOOKS_H
#include <unistd.h>
#include <time.h>

int lmsim_usleep(useconds_t us);
int lmsim_clock_gettime(clockid_t clk, struct timespec *tp);

#define usleep lmsim_usleep
#define clock_gettime lmsim_clock_gettime
#endif
