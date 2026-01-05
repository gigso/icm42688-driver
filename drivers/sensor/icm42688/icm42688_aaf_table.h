/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM42688_ICM42688_AAF_TABLE_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM42688_ICM42688_AAF_TABLE_H_

#include <stdint.h>

/**
 * @brief Anti-Alias Filter lookup table entry
 *
 * Datasheet DS-000347-ICM-42688-P-v1.7, Section 5.2
 * Table: Anti-alias filter bandwidth programming (page 28-29)
 */
struct icm42688_aaf_entry {
	uint16_t freq_hz;      /* 3dB bandwidth in Hz */
	uint8_t delt;          /* DELT value (1-63) */
	uint16_t deltsqr;      /* DELTSQR value (1-3968) */
	uint8_t bitshift;      /* BITSHIFT value (3-15) */
};

/**
 * @brief AAF lookup table (63 entries: 42 Hz to 3979 Hz)
 *
 * Used for both Gyroscope and Accelerometer AAF configuration
 */
static const struct icm42688_aaf_entry icm42688_aaf_table[] = {
	/* freq_hz, delt, deltsqr, bitshift */
	{   42,  1,    1, 15 },
	{   84,  2,    4, 13 },
	{  126,  3,    9, 12 },
	{  170,  4,   16, 11 },
	{  213,  5,   25, 10 },
	{  258,  6,   36, 10 },
	{  303,  7,   49,  9 },
	{  348,  8,   64,  9 },
	{  394,  9,   81,  9 },
	{  441, 10,  100,  8 },
	{  488, 11,  122,  8 },
	{  536, 12,  144,  8 },
	{  585, 13,  170,  8 },
	{  634, 14,  196,  7 },
	{  684, 15,  224,  7 },
	{  734, 16,  256,  7 },
	{  785, 17,  288,  7 },
	{  837, 18,  324,  7 },
	{  890, 19,  360,  6 },
	{  943, 20,  400,  6 },
	{  997, 21,  440,  6 },
	{ 1051, 22,  488,  6 },
	{ 1107, 23,  528,  6 },
	{ 1163, 24,  576,  6 },
	{ 1220, 25,  624,  6 },
	{ 1277, 26,  680,  6 },
	{ 1336, 27,  736,  5 },
	{ 1395, 28,  784,  5 },
	{ 1454, 29,  848,  5 },
	{ 1515, 30,  896,  5 },
	{ 1577, 31,  960,  5 },
	{ 1639, 32, 1024,  5 },
	{ 1702, 33, 1088,  5 },
	{ 1766, 34, 1152,  5 },
	{ 1830, 35, 1232,  5 },
	{ 1896, 36, 1296,  5 },
	{ 1962, 37, 1376,  4 },
	{ 2029, 38, 1440,  4 },
	{ 2097, 39, 1536,  4 },
	{ 2166, 40, 1600,  4 },
	{ 2235, 41, 1696,  4 },
	{ 2306, 42, 1760,  4 },
	{ 2377, 43, 1856,  4 },
	{ 2449, 44, 1952,  4 },
	{ 2522, 45, 2016,  4 },
	{ 2596, 46, 2112,  4 },
	{ 2671, 47, 2208,  4 },
	{ 2746, 48, 2304,  4 },
	{ 2823, 49, 2400,  4 },
	{ 2900, 50, 2496,  4 },
	{ 2978, 51, 2592,  4 },
	{ 3057, 52, 2720,  4 },
	{ 3137, 53, 2816,  3 },
	{ 3217, 54, 2944,  3 },
	{ 3299, 55, 3008,  3 },
	{ 3381, 56, 3136,  3 },
	{ 3464, 57, 3264,  3 },
	{ 3548, 58, 3392,  3 },
	{ 3633, 59, 3456,  3 },
	{ 3718, 60, 3584,  3 },
	{ 3805, 61, 3712,  3 },
	{ 3892, 62, 3840,  3 },
	{ 3979, 63, 3968,  3 },
};

#define ICM42688_AAF_TABLE_SIZE 63

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM42688_ICM42688_AAF_TABLE_H_ */
