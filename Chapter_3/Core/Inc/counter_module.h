/**
 ******************************************************************************
 * @file    counter_module.h
 * @author  Renato Soriano
 * @brief   Chapter 3 — Exercise 3.4b: Multi-file project — `static` / `extern`
 *          Public interface of the counter module.
 *
 * @details The counter_module exposes three operations that act on an
 *          internal counter value. The counter itself is declared `static`
 *          inside counter_module.c so no other file can read or write it
 *          directly — encapsulation by linkage, the C way.
 *
 *          The header advertises the operations the rest of the project is
 *          allowed to call, and nothing else. This is the same pattern
 *          every driver in Part II of the book will follow.
 ******************************************************************************
 */

#ifndef COUNTER_MODULE_H
#define COUNTER_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  Reset the internal counter to zero.
 */
void counter_reset(void);

/**
 * @brief  Increment the internal counter by one.
 *
 * @return Counter value after increment.
 */
uint32_t counter_increment(void);

/**
 * @brief  Read the internal counter without modifying it.
 */
uint32_t counter_get(void);

#ifdef __cplusplus
}
#endif

#endif /* COUNTER_MODULE_H */
