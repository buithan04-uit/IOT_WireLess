#ifndef MAX30100_BEATDETECTOR_H
#define MAX30100_BEATDETECTOR_H

#include <stdint.h>
#include <stdbool.h>

#define BEATDETECTOR_INIT_HOLDOFF 2000   // ms
#define BEATDETECTOR_MASKING_HOLDOFF 200 // ms
#define BEATDETECTOR_BPFILTER_ALPHA 0.6f
#define BEATDETECTOR_MIN_THRESHOLD 20
#define BEATDETECTOR_MAX_THRESHOLD 800
#define BEATDETECTOR_STEP_RESILIENCY 30
#define BEATDETECTOR_THRESHOLD_FALLOFF_TARGET 0.3f
#define BEATDETECTOR_THRESHOLD_DECAY_FACTOR 0.99f
#define BEATDETECTOR_INVALID_READOUT_DELAY 2000 // ms
#define BEATDETECTOR_SAMPLES_PERIOD 10          // ms

typedef enum
{
    BEATDETECTOR_STATE_INIT,
    BEATDETECTOR_STATE_WAITING,
    BEATDETECTOR_STATE_FOLLOWING_SLOPE,
    BEATDETECTOR_STATE_MAYBE_DETECTED,
    BEATDETECTOR_STATE_MASKING
} BeatDetectorState;

typedef struct
{
    BeatDetectorState state;
    float threshold;
    float beatPeriod;
    float lastMaxValue;
    uint32_t tsLastBeat;
} BeatDetector;

// Khởi tạo detector
void BeatDetector_Init(BeatDetector *bd);

// Thêm mẫu mới, trả về true nếu phát hiện nhịp
bool BeatDetector_AddSample(BeatDetector *bd, float sample, uint32_t timestamp_ms);

// Lấy nhịp tim (bpm)
float BeatDetector_GetRate(BeatDetector *bd);

// Lấy ngưỡng hiện tại
float BeatDetector_GetCurrentThreshold(BeatDetector *bd);

#endif