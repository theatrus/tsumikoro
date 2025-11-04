/**
 * @file test_framework.h
 * @brief Simple unit test framework for Tsumikoro bus protocol
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ANSI color codes for terminal output */
#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_BOLD    "\x1b[1m"
#define COLOR_RESET   "\x1b[0m"

/* Test statistics */
static int g_assertions_run = 0;
static int g_assertions_passed = 0;
static int test_total = 0;
static int test_passed = 0;
static int test_failed = 0;
static const char *current_test_name __attribute__((unused)) = NULL;

/* Test assertion macro */
#define TEST_ASSERT(condition, message) \
    do { \
        test_total++; \
        g_assertions_run++; \
        if (!(condition)) { \
            test_failed++; \
            printf(COLOR_RED "  [FAIL] " COLOR_RESET "%s:%d: %s\n", \
                   __FILE__, __LINE__, message); \
            printf("         Condition: %s\n", #condition); \
        } else { \
            test_passed++; \
            g_assertions_passed++; \
        } \
    } while (0)

/* Test assertion with formatted message */
#define TEST_ASSERT_MSG(condition, fmt, ...) \
    do { \
        test_total++; \
        g_assertions_run++; \
        if (!(condition)) { \
            test_failed++; \
            printf(COLOR_RED "  [FAIL] " COLOR_RESET "%s:%d: ", \
                   __FILE__, __LINE__); \
            printf(fmt, ##__VA_ARGS__); \
            printf("\n"); \
            printf("         Condition: %s\n", #condition); \
        } else { \
            test_passed++; \
            g_assertions_passed++; \
        } \
    } while (0)

/* Test assertion for equality */
#define TEST_ASSERT_EQUAL(expected, actual) \
    TEST_ASSERT_MSG((expected) == (actual), \
                    "Expected %d, got %d", (int)(expected), (int)(actual))

/* Test assertion for byte array equality */
#define TEST_ASSERT_EQUAL_BYTES(expected, actual, len) \
    do { \
        test_total++; \
        g_assertions_run++; \
        if (memcmp((expected), (actual), (len)) != 0) { \
            test_failed++; \
            printf(COLOR_RED "  [FAIL] " COLOR_RESET "%s:%d: Byte arrays differ\n", \
                   __FILE__, __LINE__); \
            printf("         Expected: "); \
            for (size_t i = 0; i < (len); i++) { \
                printf("%02X ", ((uint8_t*)(expected))[i]); \
            } \
            printf("\n"); \
            printf("         Actual:   "); \
            for (size_t i = 0; i < (len); i++) { \
                printf("%02X ", ((uint8_t*)(actual))[i]); \
            } \
            printf("\n"); \
        } else { \
            test_passed++; \
            g_assertions_passed++; \
        } \
    } while (0)

/* Begin a test suite */
#define TEST_SUITE(name) \
    printf("\n" COLOR_YELLOW "=== Test Suite: %s ===" COLOR_RESET "\n", name)

/* Run a test function */
#define RUN_TEST(test_func) \
    do { \
        current_test_name = #test_func; \
        printf("\nRunning: %s\n", current_test_name); \
        test_func(); \
    } while (0)

/* Print test summary */
#define TEST_SUMMARY() \
    do { \
        printf("\n" COLOR_YELLOW "=== Test Summary ===" COLOR_RESET "\n"); \
        printf("Total:  %d\n", test_total); \
        printf(COLOR_GREEN "Passed: %d\n" COLOR_RESET, test_passed); \
        if (test_failed > 0) { \
            printf(COLOR_RED "Failed: %d\n" COLOR_RESET, test_failed); \
        } else { \
            printf("Failed: %d\n", test_failed); \
        } \
        printf("\n"); \
        if (test_failed == 0) { \
            printf(COLOR_GREEN "All tests passed!" COLOR_RESET "\n"); \
        } else { \
            printf(COLOR_RED "%d test(s) failed" COLOR_RESET "\n", test_failed); \
        } \
    } while (0)

/* Return exit code based on test results */
#define TEST_EXIT() (test_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE)

#endif /* TEST_FRAMEWORK_H */
