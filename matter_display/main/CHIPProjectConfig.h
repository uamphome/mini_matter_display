/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* Project-level CHIP configuration overrides. Wired in via CONFIG_CHIP_PROJECT_CONFIG
 * ("main/CHIPProjectConfig.h"); the esp32 chip component injects this header into both
 * CHIPConfig.h and SystemConfig.h. Keep it minimal and guarded so it is safe in both contexts. */

#pragma once

/* Number of CASE handshakes CHIP will keep in flight at once. The default of 2 bottlenecks boot:
 * this switch drives the CASE sessions of every bound sensor node in parallel, and overflow nodes
 * fail with CHIP_ERROR_NO_MEMORY and have to trickle in on the retry timer. */
#ifndef CHIP_CONFIG_DEVICE_MAX_ACTIVE_CASE_CLIENTS
#define CHIP_CONFIG_DEVICE_MAX_ACTIVE_CASE_CLIENTS 6
#endif
