#include "MAX30100_BeatDetector.h"
#include <string.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

static void BeatDetector_DecreaseThreshold(BeatDetector *bd);

void BeatDetector_Init(BeatDetector *bd)
{
    bd->state = BEATDETECTOR_STATE_INIT;
    bd->threshold = BEATDETECTOR_MIN_THRESHOLD;
    bd->beatPeriod = 0;
    bd->lastMaxValue = 0;
    bd->tsLastBeat = 0;
}

float BeatDetector_GetRate(BeatDetector *bd)
{
    if (bd->beatPeriod != 0)
    {
        return 1.0f / bd->beatPeriod * 1000.0f * 60.0f;
    }
    else
    {
        return 0.0f;
    }
}

float BeatDetector_GetCurrentThreshold(BeatDetector *bd)
{
    return bd->threshold;
}

// Hàm chính: thêm mẫu mới, trả về true nếu phát hiện nhịp
bool BeatDetector_AddSample(BeatDetector *bd, float sample, uint32_t timestamp_ms)
{
    bool beatDetected = false;

    switch (bd->state)
    {
    case BEATDETECTOR_STATE_INIT:
        if (timestamp_ms > BEATDETECTOR_INIT_HOLDOFF)
        {
            bd->state = BEATDETECTOR_STATE_WAITING;
        }
        break;

    case BEATDETECTOR_STATE_WAITING:
        if (sample > bd->threshold)
        {
            bd->threshold = MIN(sample, BEATDETECTOR_MAX_THRESHOLD);
            bd->state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
        }
        // Tracking lost, resetting
        if (timestamp_ms - bd->tsLastBeat > BEATDETECTOR_INVALID_READOUT_DELAY)
        {
            bd->beatPeriod = 0;
            bd->lastMaxValue = 0;
        }
        BeatDetector_DecreaseThreshold(bd);
        break;

    case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
        if (sample < bd->threshold)
        {
            bd->state = BEATDETECTOR_STATE_MAYBE_DETECTED;
        }
        else
        {
            bd->threshold = MIN(sample, BEATDETECTOR_MAX_THRESHOLD);
        }
        break;

    case BEATDETECTOR_STATE_MAYBE_DETECTED:
        if (sample + BEATDETECTOR_STEP_RESILIENCY < bd->threshold)
        {
            // Found a beat
            beatDetected = true;
            bd->lastMaxValue = sample;
            bd->state = BEATDETECTOR_STATE_MASKING;
            float delta = (float)(timestamp_ms - bd->tsLastBeat);
            if (delta > 0)
            {
                bd->beatPeriod = BEATDETECTOR_BPFILTER_ALPHA * delta +
                                 (1.0f - BEATDETECTOR_BPFILTER_ALPHA) * bd->beatPeriod;
            }
            bd->tsLastBeat = timestamp_ms;
        }
        else
        {
            bd->state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
        }
        break;

    case BEATDETECTOR_STATE_MASKING:
        if (timestamp_ms - bd->tsLastBeat > BEATDETECTOR_MASKING_HOLDOFF)
        {
            bd->state = BEATDETECTOR_STATE_WAITING;
        }
        BeatDetector_DecreaseThreshold(bd);
        break;
    }

    return beatDetected;
}

// Hàm giảm ngưỡng threshold
static void BeatDetector_DecreaseThreshold(BeatDetector *bd)
{
    if (bd->lastMaxValue > 0 && bd->beatPeriod > 0)
    {
        bd->threshold -= bd->lastMaxValue * (1.0f - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET) /
                         (bd->beatPeriod / BEATDETECTOR_SAMPLES_PERIOD);
    }
    else
    {
        bd->threshold *= BEATDETECTOR_THRESHOLD_DECAY_FACTOR;
    }
    if (bd->threshold < BEATDETECTOR_MIN_THRESHOLD)
    {
        bd->threshold = BEATDETECTOR_MIN_THRESHOLD;
    }
}