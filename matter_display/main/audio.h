/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Bring up the ES8311 codec + I2S + speaker power amp and start the audio playback task. Must be
 *  called AFTER amoled_init() (it reuses that I2C bus and TCA9554 IO expander). Idempotent; returns
 *  ESP_OK on success. If it fails, audio_play_alert() becomes a no-op so the rest of the app is
 *  unaffected. */
esp_err_t audio_init(void);

/** Play a single-note "bell" notification chime (non-repeating), like a watch notification. Fully
 *  non-blocking and safe to call from any task (including the Matter stack thread): the sound is
 *  rendered on a dedicated audio task. No-op if audio_init() has not succeeded. */
void audio_play_alert(void);

#ifdef __cplusplus
}
#endif
