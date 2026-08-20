/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* Notification "bell" for the Waveshare ESP32-C6-Touch-AMOLED-1.8 onboard speaker.
 *
 * Audio path (from the board schematic): ESP32-C6 I2S → ES8311 codec (I2C-configured on the shared
 * bus @ 0x18) → NS4150B power amp (enabled by TCA9554 P7) → speaker.
 *   I2S:  MCLK=GPIO19  BCLK/SCLK=GPIO20  WS/LRCK=GPIO22  DOUT/DSDIN=GPIO23
 *   PA enable: TCA9554 EXIO7 (via the mutex-guarded amoled_io_expander_set_level).
 *
 * A single decaying-bell tone is synthesised once at init and played on a dedicated task, so
 * triggering it never blocks the caller (e.g. the Matter stack thread). The PA is enabled only for
 * the duration of playback to avoid idle hiss. */

#include "audio.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_io_expander.h"

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"

#include "amoled.h"

static const char *TAG = "audio";

#define AUDIO_I2S_PORT     I2S_NUM_0
#define AUDIO_MCLK_GPIO    19
#define AUDIO_BCLK_GPIO    20
#define AUDIO_WS_GPIO      22
#define AUDIO_DOUT_GPIO    23
#define AUDIO_SAMPLE_RATE  8000
#define AUDIO_PA_PIN       IO_EXPANDER_PIN_NUM_7   /* TCA9554 P7 = PA_CTRL */
#define AUDIO_VOLUME       78                      /* 0-100 */
#define AUDIO_PI           3.14159265358979323846f

/* Multi-note chime: a short ascending arpeggio of bell notes that ring into each other. ~1.7 s,
 * 16-bit MONO (the onboard speaker is a single channel) at 8 kHz — the chime's highest partial is
 * ~2.4 kHz, well under the 4 kHz Nyquist, so 8 kHz costs no audible quality and keeps the buffer
 * small (~27 KB vs ~108 KB for 16 kHz stereo). */
#define TONE_MS            1700
#define TONE_FRAMES        (AUDIO_SAMPLE_RATE * TONE_MS / 1000)
static int16_t s_tone[TONE_FRAMES];

static esp_codec_dev_handle_t s_dev      = NULL;
static SemaphoreHandle_t      s_play_sem = NULL;

/* Render one struck-bell note into s_tone starting at frame `start`, MIXED (added) into whatever is
 * already there so successive notes overlap and ring together. A fundamental plus two faster-decaying
 * partials (one inharmonic) give the metallic timbre; one exponential envelope with a 5 ms attack
 * ramp rings it out without a click. Accumulation is clamped to the int16 range.
 *
 * The C6 has no hardware FPU, so this loop is soft-float and its cost has to be managed carefully:
 *
 *  - Phases are stepped and wrapped to [0, 2*pi) rather than evaluated as sinf(2*pi*f*t). A growing
 *    argument (which reaches ~25000 rad by the end of a note) sends newlib into its multi-precision
 *    argument-reduction path (__kernel_rem_pio2f), hundreds of times slower than the fast path —
 *    slow enough that synthesis starved the whole system for tens of seconds at boot.
 *  - The exponential envelopes are stepped by a per-sample factor instead of calling expf() three
 *    times per sample.
 *
 * It still yields periodically, and runs on a low-priority task, so it can never starve the network
 * stack even if it is slow. */
static void add_note(int start, float f0, float amp)
{
    const float sr     = (float)AUDIO_SAMPLE_RATE;
    const float tau    = 0.45f;    /* per-note ring time (s) */
    const float two_pi = 2.0f * AUDIO_PI;

    /* Phase increments per sample, and the matching per-sample envelope decay factors. */
    float ph1 = 0.0f, ph2 = 0.0f, ph3 = 0.0f;
    const float d1 = two_pi * f0 / sr;
    const float d2 = two_pi * 2.0f * f0 / sr;
    const float d3 = two_pi * 2.7f * f0 / sr;
    float env = 1.0f, e2 = 1.0f, e3 = 1.0f;
    const float k_env = expf(-1.0f / (sr * tau));
    const float k2    = expf(-1.0f / (sr * tau * 0.5f));
    const float k3    = expf(-1.0f / (sr * tau * 0.35f));

    for (int n = 0; start + n < TONE_FRAMES; n++) {
        if ((n & 0xFF) == 0) {
            vTaskDelay(1);      /* let the idle task run / feed the WDT */
        }
        if (env < 0.001f) {
            break;              /* note has decayed to silence */
        }
        float t   = n / sr;
        float amp_env = env;
        if (t < 0.005f) {
            amp_env *= (t / 0.005f);   /* 5 ms attack ramp */
        }
        float s = sinf(ph1) + 0.5f * sinf(ph2) * e2 + 0.25f * sinf(ph3) * e3;
        int idx = start + n;
        int32_t mixed = (int32_t)s_tone[idx] + (int32_t)(amp_env * amp * s / 1.75f);
        if (mixed >  32767) mixed =  32767;
        if (mixed < -32768) mixed = -32768;
        s_tone[idx] = (int16_t)mixed;

        ph1 += d1; if (ph1 >= two_pi) ph1 -= two_pi;
        ph2 += d2; if (ph2 >= two_pi) ph2 -= two_pi;
        ph3 += d3; if (ph3 >= two_pi) ph3 -= two_pi;
        env *= k_env; e2 *= k2; e3 *= k3;
    }
}

/* Synthesise the notification chime: a gentle rising A-major arpeggio (A4 C#5 E5 A5), each note a
 * struck bell, spaced so the tail of one blends into the attack of the next. */
static void generate_bell(void)
{
    const float amp  = 11000.0f;   /* per-note peak; leaves headroom for overlapping notes */
    const int   step = AUDIO_SAMPLE_RATE * 190 / 1000;   /* 190 ms between note onsets */
    memset(s_tone, 0, sizeof(s_tone));
    add_note(0 * step, 440.00f, amp);   /* A4  */
    add_note(1 * step, 554.37f, amp);   /* C#5 */
    add_note(2 * step, 659.25f, amp);   /* E5  */
    add_note(3 * step, 880.00f, amp);   /* A5  */
}

/* Play the bell once (blocking, on the audio task only). */
static void play_bell_once(void)
{
    if (!s_dev) {
        return;
    }
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel         = 1,
        .channel_mask    = 0,
        .sample_rate     = AUDIO_SAMPLE_RATE,
        .mclk_multiple   = 0,
    };
    if (esp_codec_dev_open(s_dev, &fs) != 0) {
        ESP_LOGW(TAG, "codec open failed");
        return;
    }
    esp_codec_dev_set_out_vol(s_dev, AUDIO_VOLUME);
    amoled_io_expander_set_level(AUDIO_PA_PIN, 1);   /* enable speaker amp */
    vTaskDelay(pdMS_TO_TICKS(8));                    /* let the PA settle before audio */
    esp_codec_dev_write(s_dev, s_tone, (int)sizeof(s_tone));
    vTaskDelay(pdMS_TO_TICKS(60));                   /* let the I2S DMA drain the tail */
    amoled_io_expander_set_level(AUDIO_PA_PIN, 0);   /* disable amp (avoid idle hiss) */
    esp_codec_dev_close(s_dev);
}

static void audio_task(void *arg)
{
    (void)arg;
    /* Synthesise the chime here rather than in audio_init(): even optimised, the soft-float
     * synthesis has no business running on the main task during the boot CASE storm. This task runs
     * below the UI and the network stack, and generate_bell() yields as it goes. */
    generate_bell();
    for (;;) {
        xSemaphoreTake(s_play_sem, portMAX_DELAY);
        /* Coalesce rapid triggers into a single chime (non-repeating). */
        while (xSemaphoreTake(s_play_sem, 0) == pdTRUE) { }
        play_bell_once();
    }
}

esp_err_t audio_init(void)
{
    if (s_dev) {
        return ESP_OK;   /* already initialised */
    }

    /* I2S TX channel — ESP is the I2S master, the ES8311 is a slave. */
    i2s_chan_handle_t tx = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(AUDIO_I2S_PORT, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, NULL), TAG, "i2s_new_channel");

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = AUDIO_MCLK_GPIO,
            .bclk = AUDIO_BCLK_GPIO,
            .ws   = AUDIO_WS_GPIO,
            .dout = AUDIO_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { 0 },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &std_cfg), TAG, "i2s_std");

    /* esp_codec_dev: I2C control interface on the shared bus, I2S data interface, ES8311 driver. */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port       = AMOLED_I2C_NUM,
        .addr       = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = amoled_get_i2c_bus(),
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    audio_codec_i2s_cfg_t i2s_cfg = { .port = AUDIO_I2S_PORT, .rx_handle = NULL, .tx_handle = tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (!ctrl_if || !data_if) {
        ESP_LOGE(TAG, "codec I2C/I2S interface create failed");
        return ESP_FAIL;
    }

    es8311_codec_cfg_t es_cfg = {
        .ctrl_if     = ctrl_if,
        .gpio_if     = audio_codec_new_gpio(),
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,   /* playback only */
        .pa_pin      = -1,                            /* PA is on the TCA9554; we toggle it ourselves */
        .pa_reverted = false,
        .master_mode = false,                         /* ESP is the I2S master */
        .use_mclk    = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .no_dac_ref  = false,
        .mclk_div    = 256,
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es_cfg);
    if (!codec_if) {
        ESP_LOGE(TAG, "es8311_codec_new failed");
        return ESP_FAIL;
    }

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec_if,
        .data_if  = data_if,
    };
    s_dev = esp_codec_dev_new(&dev_cfg);
    if (!s_dev) {
        ESP_LOGE(TAG, "esp_codec_dev_new failed");
        return ESP_FAIL;
    }

    /* PA enable pin (TCA9554 P7) as output, disabled until we play. */
    amoled_io_expander_set_dir(AUDIO_PA_PIN, true);
    amoled_io_expander_set_level(AUDIO_PA_PIN, 0);

    /* Priority 1: below the LVGL task (2) and the network stack, just above idle. The chime is
     * never time-critical, and playback itself is mostly waiting on I2S DMA — but synthesis is a
     * long soft-float burst, so it must not be able to preempt Matter/Thread work. */
    s_play_sem = xSemaphoreCreateBinary();
    if (!s_play_sem || xTaskCreate(audio_task, "audio", 4096, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "audio task create failed");
        s_dev = NULL;   /* disable audio_play_alert() */
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Audio ready (ES8311, I2S%d, %d Hz)", AUDIO_I2S_PORT, AUDIO_SAMPLE_RATE);
    return ESP_OK;
}

void audio_play_alert(void)
{
    if (s_play_sem) {
        xSemaphoreGive(s_play_sem);
    }
}
