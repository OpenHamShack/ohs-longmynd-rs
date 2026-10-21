/* The C side of the equivalence harness: longmynd's own main.c and ts.c,
 * compiled into this file so their static functions and state are
 * reachable, and three ways of running them against the simulator.
 *
 *   mode 0: the real main(), renamed lm_c_main(). Used for runs that end
 *           before main() starts its threads (argument errors, no device,
 *           a missing FIFO), where it is deterministic as it stands.
 *   mode 1: main() up to its threads, then loop_i2c() - the receiver
 *           thread - run here, on this thread, against the virtual clock.
 *           main()'s loop body runs at each of the receiver's sleeps
 *           (main_poll below), which is where the real main would have
 *           run between them.
 *   mode 2: main() up to its threads, then loop_ts() run here, fed canned
 *           TS, with the parse thread's work done before each TS read.
 *
 * Only main_poll() is not longmynd's own code: it is main()'s loop body,
 * copied, without the 10ms sleep. The Rust side runs the port's equivalent
 * (app::MainLoop::poll) at the same points, so the two are compared doing
 * the same thing at the same virtual times. Keep it in step with main.c.
 */

#define main lm_c_main
#include "main.c"
#undef main
#include "ts.c"
#include <libusb-1.0/libusb.h>

extern void lmsim_set_sleep_hook(void (*hook)(void));
extern void lmsim_set_ts_hook(void (*hook)(void));
extern int lmsim_terminate_due(void);
extern uint32_t lmsim_ts_flow(void);
extern void lmsim_set_option(libusb_context *ctx, int option);

/* libusb_set_option is variadic, which Rust cannot define */
int libusb_set_option(libusb_context *ctx, enum libusb_option option, ...)
{
    lmsim_set_option(ctx, (int)option);
    return 0;
}

/* beep.c needs ALSA; the harness never enables the beep, and this is the
 * C's loop with the tone left out. */
void *loop_beep(void *arg)
{
    thread_vars_t *thread_vars = (thread_vars_t *)arg;
    uint8_t *err = &thread_vars->thread_err;
    while (*err == ERROR_NONE && *thread_vars->main_err_ptr == ERROR_NONE) {
        usleep(10 * 1000);
    }
    return NULL;
}

/* main()'s locals, as statics so the hooks can reach them */
static uint8_t harness_err = ERROR_NONE;
static uint8_t (*h_status_write)(uint8_t, uint32_t, bool *);
static uint8_t (*h_status_string_write)(uint8_t, char *, bool *);
static bool status_output_ready = true;
static uint64_t last_status_sent_monotonic = 0;
static thread_vars_t tv_i2c, tv_ts;

/* main.c, main(): from "first setup the fifos" to before the threads */
static void setup(int argc, char *argv[])
{
    printf("Flow: main\n");
    if (harness_err == ERROR_NONE) harness_err = process_command_line(argc, argv, &longmynd_config);
    if (longmynd_config.status_use_ip) {
        if (harness_err == ERROR_NONE) harness_err = udp_status_init(longmynd_config.status_ip_addr, longmynd_config.status_ip_port);
        h_status_write = udp_status_write;
        h_status_string_write = udp_status_string_write;
    } else {
        if (harness_err == ERROR_NONE) harness_err = fifo_status_init(longmynd_config.status_fifo_path, &status_output_ready);
        h_status_write = fifo_status_write;
        h_status_string_write = fifo_status_string_write;
    }
    if (harness_err == ERROR_NONE) harness_err = ftdi_init(longmynd_config.device_usb_bus, longmynd_config.device_usb_addr);
}

static void finish(void)
{
    printf("Flow: Main loop aborted, waiting for threads.\n");
    printf("Flow: All threads accounted for. Exiting cleanly.\n");
}

/* main()'s loop body (main.c lines 1165-1210), minus the 10ms sleep */
static void main_poll(void)
{
    longmynd_status_t longmynd_status_cpy;
    if (harness_err != ERROR_NONE) return;

    if (longmynd_status.last_updated_monotonic != last_status_sent_monotonic) {
        pthread_mutex_lock(&longmynd_status.mutex);
        memcpy(&longmynd_status_cpy, &longmynd_status, sizeof(longmynd_status_t));
        pthread_mutex_unlock(&longmynd_status.mutex);
        if (longmynd_config.status_use_ip || status_output_ready) {
            harness_err = status_all_write(&longmynd_status_cpy, h_status_write, h_status_string_write, &status_output_ready);
        } else if (!longmynd_config.status_use_ip && !status_output_ready) {
            harness_err = fifo_status_init(longmynd_config.status_fifo_path, &status_output_ready);
        }
        last_status_sent_monotonic = longmynd_status_cpy.last_updated_monotonic;
    }
    if (harness_err == ERROR_NONE && (tv_i2c.thread_err != ERROR_NONE || tv_ts.thread_err != ERROR_NONE)) {
        harness_err = ERROR_THREAD_ERROR;
    }
    if (longmynd_config.ts_timeout != -1
        && monotonic_ms() > (longmynd_status.last_ts_or_reinit_monotonic + longmynd_config.ts_timeout)) {
        printf("Flow: No-data timeout, re-init config.\n");
        config_reinit(true);
        pthread_mutex_lock(&longmynd_status.mutex);
        longmynd_status.last_ts_or_reinit_monotonic = monotonic_ms();
        pthread_mutex_unlock(&longmynd_status.mutex);
    }
}

/* at each of the receiver's sleeps: TS arriving (for the no-TS timer),
 * main()'s loop, and the end of the run (as SIGTERM would end it) */
static void receiver_sleep_hook(void)
{
    uint32_t add = lmsim_ts_flow();
    if (add) longmynd_status.ts_packet_count_nolock += add;
    main_poll();
    if (lmsim_terminate_due() && harness_err == ERROR_NONE) harness_err = ERROR_SIGNAL_TERMINATE;
}

static int run_receiver(int argc, char *argv[])
{
    setup(argc, argv);
    tv_i2c = (thread_vars_t){ .main_err_ptr = &harness_err, .thread_err = ERROR_NONE,
                              .config = &longmynd_config, .status = &longmynd_status };
    if (harness_err == ERROR_NONE) {
        pthread_mutex_lock(&longmynd_status.mutex);
        longmynd_status.last_ts_or_reinit_monotonic = monotonic_ms();
        pthread_mutex_unlock(&longmynd_status.mutex);
        lmsim_set_sleep_hook(receiver_sleep_hook);
        loop_i2c(&tv_i2c);
        lmsim_set_sleep_hook(NULL);
        /* main()'s next pass sees what the thread left behind */
        main_poll();
    }
    finish();
    return harness_err;
}

/* The parse thread, between TS reads: if loop_ts handed it a chunk (it
 * clears .waiting when it does), parse it, then show the result as main()
 * would; then be waiting again. */
static int ts_hook_armed = 0;
static void ts_parse_pending(void)
{
    if (ts_hook_armed && !longmynd_ts_parse_buffer.waiting) {
        longmynd_status_t cpy;
        ts_parse(longmynd_ts_parse_buffer.buffer, longmynd_ts_parse_buffer.length,
                 &ts_callback_sdt_service, &ts_callback_pmt_pids, &ts_callback_ts_stats, false);
        pthread_mutex_lock(&longmynd_status.mutex);
        memcpy(&cpy, &longmynd_status, sizeof(longmynd_status_t));
        pthread_mutex_unlock(&longmynd_status.mutex);
        if (longmynd_config.status_use_ip || status_output_ready) {
            uint8_t e = status_all_write(&cpy, h_status_write, h_status_string_write, &status_output_ready);
            if (e != ERROR_NONE && harness_err == ERROR_NONE) harness_err = e;
        }
    }
}

static void ts_hook(void)
{
    ts_parse_pending();
    longmynd_ts_parse_buffer.waiting = true;
    ts_hook_armed = 1;
    if (lmsim_terminate_due() && harness_err == ERROR_NONE) harness_err = ERROR_SIGNAL_TERMINATE;
}

static int run_ts(int argc, char *argv[])
{
    setup(argc, argv);
    tv_ts = (thread_vars_t){ .main_err_ptr = &harness_err, .thread_err = ERROR_NONE,
                             .config = &longmynd_config, .status = &longmynd_status };
    if (harness_err == ERROR_NONE) {
        /* what loop_ts_parse() sets up, and what the receiver's first
         * configuration does to the TS thread */
        ts_longmynd_status = &longmynd_status;
        longmynd_ts_parse_buffer.buffer = malloc(TS_FRAME_SIZE);
        memset(longmynd_ts_parse_buffer.buffer, 0, TS_FRAME_SIZE);
        longmynd_config.ts_reset = true;
        lmsim_set_ts_hook(ts_hook);
        loop_ts(&tv_ts);
        lmsim_set_ts_hook(NULL);
        ts_parse_pending();
        if (harness_err == ERROR_NONE && tv_ts.thread_err != ERROR_NONE) harness_err = ERROR_THREAD_ERROR;
    }
    finish();
    return harness_err;
}

int lmref_run(int mode, int argc, char *argv[])
{
    switch (mode) {
    case 0: return lm_c_main(argc, argv);
    case 1: return run_receiver(argc, argv);
    case 2: return run_ts(argc, argv);
    }
    return 255;
}

/* For cnum's tests: the reference compiler's own conversions. */
uint32_t lmref_double_to_u32(double d) { return (uint32_t)d; }
int32_t lmref_double_to_i32(double d) { return (int32_t)d; }
long lmref_strtol(const char *s) { return strtol(s, NULL, 10); }
float lmref_strtof(const char *s) { return strtof(s, NULL); }
