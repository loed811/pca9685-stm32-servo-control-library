/*
 * Minimal host-side test framework (no external dependencies).
 *
 * CHECK*       - hard assertion, a failure makes the test binary exit non-zero.
 * KNOWN_BUG*   - documents a defect that is present in the driver today.
 *                Reported as XFAIL while the bug exists and as XPASS once it is
 *                fixed; neither outcome fails the build. Build with
 *                `make STRICT=1` to turn XFAIL into failures.
 */
#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <math.h>
#include <stdio.h>

extern int tf_failures;
extern int tf_checks;
extern int tf_xfail;
extern int tf_xpass;
extern const char *tf_current;

#ifdef STRICT_KNOWN_BUGS
#define TF_XFAIL_IS_FAILURE 1
#else
#define TF_XFAIL_IS_FAILURE 0
#endif

#define TF_FAIL(fmt, ...)                                                      \
    do {                                                                       \
        tf_failures++;                                                         \
        printf("  FAIL %s (%s:%d): " fmt "\n", tf_current, __FILE__, __LINE__,  \
               __VA_ARGS__);                                                   \
    } while (0)

#define CHECK(cond)                                                            \
    do {                                                                       \
        tf_checks++;                                                           \
        if (!(cond))                                                           \
            TF_FAIL("%s", #cond);                                              \
    } while (0)

#define CHECK_EQ(actual, expected)                                             \
    do {                                                                       \
        long long a_ = (long long)(actual), e_ = (long long)(expected);        \
        tf_checks++;                                                           \
        if (a_ != e_)                                                          \
            TF_FAIL("%s == %lld (0x%llX), expected %lld (0x%llX)", #actual,    \
                    a_, a_, e_, e_);                                           \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                      \
    do {                                                                       \
        double a_ = (double)(actual), e_ = (double)(expected);                 \
        tf_checks++;                                                           \
        if (fabs(a_ - e_) > (tol))                                             \
            TF_FAIL("%s == %f, expected %f +/- %f", #actual, a_, e_,           \
                    (double)(tol));                                            \
    } while (0)

#define KNOWN_BUG(desc, cond)                                                  \
    do {                                                                       \
        tf_checks++;                                                           \
        if (cond) {                                                            \
            tf_xpass++;                                                        \
            printf("  XPASS %s: %s (bug looks fixed, convert to CHECK)\n",     \
                   tf_current, desc);                                          \
        } else {                                                               \
            tf_xfail++;                                                        \
            printf("  XFAIL %s: %s [%s:%d]\n", tf_current, desc, __FILE__,     \
                   __LINE__);                                                  \
            if (TF_XFAIL_IS_FAILURE)                                           \
                tf_failures++;                                                 \
        }                                                                      \
    } while (0)

#define RUN_TEST(fn)                                                           \
    do {                                                                       \
        int before_ = tf_failures;                                             \
        tf_current = #fn;                                                      \
        test_setup();                                                          \
        fn();                                                                  \
        printf("%s %s\n", tf_failures == before_ ? "[ OK ]" : "[FAIL]", #fn);  \
    } while (0)

#define TF_DEFINE_GLOBALS                                                      \
    int tf_failures, tf_checks, tf_xfail, tf_xpass;                            \
    const char *tf_current = ""

#define TF_SUMMARY_AND_EXIT(suite)                                             \
    do {                                                                       \
        printf("\n%s: %d checks, %d failed, %d known bugs (xfail), %d xpass\n", \
               suite, tf_checks, tf_failures, tf_xfail, tf_xpass);             \
        return tf_failures ? 1 : 0;                                            \
    } while (0)

#endif /* TEST_FRAMEWORK_H */
