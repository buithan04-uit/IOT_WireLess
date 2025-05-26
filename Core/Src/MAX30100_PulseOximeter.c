#include "MAX30100_PulseOximeter.h"
#include "stm32f1xx_hal.h" // hoặc dòng STM32 bạn dùng

static void checkSample(PulseOximeter *po);
static void checkCurrentBias(PulseOximeter *po);

void PulseOximeter_Init(PulseOximeter *po)
{
    po->state = PULSEOXIMETER_STATE_INIT;
    po->tsFirstBeatDetected = 0;
    po->tsLastBeatDetected = 0;
    po->tsLastBiasCheck = 0;
    po->tsLastCurrentAdjustment = 0;
    po->redLedCurrentIndex = (uint8_t)RED_LED_CURRENT_START;
    po->irLedCurrent = DEFAULT_IR_LED_CURRENT;
    po->onBeatDetected = NULL;
    CircularBuffer_Init(&po->hrm.readoutsBuffer);
}

bool PulseOximeter_Begin(PulseOximeter *po, PulseOximeterDebuggingMode debuggingMode)
{
    po->debuggingMode = debuggingMode;

    bool ready = MAX30100_Begin(&po->hrm);

    if (!ready)
    {
        // Có thể thêm debug qua UART nếu cần
        return false;
    }

    MAX30100_SetMode(&po->hrm, MAX30100_MODE_SPO2_HR);
	MAX30100_SetLedsCurrent(&po->hrm, po->irLedCurrent, (LEDCurrent)po->redLedCurrentIndex);
	// Thay thế 3 dòng dưới đây:
	// MAX30100_SetLedsPulseWidth(&po->hrm, MAX30100_SPC_PW_1600US_16BITS);
	// MAX30100_SetSamplingRate(&po->hrm, MAX30100_SAMPRATE_100HZ);
	// MAX30100_SetHighresModeEnabled(&po->hrm, true);
	// Bằng 1 dòng này:
	MAX30100_ConfigureSPO2(&po->hrm, MAX30100_SAMPRATE_100HZ, MAX30100_SPC_PW_1600US_16BITS, true);

    DCRemover_Init(&po->irDCRemover, DC_REMOVER_ALPHA);
    DCRemover_Init(&po->redDCRemover, DC_REMOVER_ALPHA);

    po->state = PULSEOXIMETER_STATE_IDLE;

    return true;
}

void PulseOximeter_Update(PulseOximeter *po)
{
    MAX30100_Update(&po->hrm);
    checkSample(po);
    checkCurrentBias(po);
}

float PulseOximeter_GetHeartRate(PulseOximeter *po)
{
    return BeatDetector_GetRate(&po->beatDetector);
}

uint8_t PulseOximeter_GetSpO2(PulseOximeter *po)
{
    return SpO2Calculator_GetSpO2(&po->spO2calculator);
}

uint8_t PulseOximeter_GetRedLedCurrentBias(PulseOximeter *po)
{
    return po->redLedCurrentIndex;
}

void PulseOximeter_SetOnBeatDetectedCallback(PulseOximeter *po, void (*cb)(void))
{
    po->onBeatDetected = cb;
}

void PulseOximeter_SetIRLedCurrent(PulseOximeter *po, LEDCurrent irLedNewCurrent)
{
    po->irLedCurrent = irLedNewCurrent;
    MAX30100_SetLedsCurrent(&po->hrm, po->irLedCurrent, (LEDCurrent)po->redLedCurrentIndex);
}

void PulseOximeter_Shutdown(PulseOximeter *po)
{
    MAX30100_Shutdown(&po->hrm);
}

void PulseOximeter_Resume(PulseOximeter *po)
{
    MAX30100_Resume(&po->hrm);
}

// --- Hàm nội bộ ---

static void checkSample(PulseOximeter *po)
{
    uint16_t rawIRValue, rawRedValue;

    // Lấy tất cả sample có sẵn
    while (MAX30100_GetRawValues(&po->hrm, &rawIRValue, &rawRedValue))
    {
        float irACValue = DCRemover_Step(&po->irDCRemover, (float)rawIRValue);
        float redACValue = DCRemover_Step(&po->redDCRemover, (float)rawRedValue);

        // Tín hiệu đưa vào beat detector là đảo dấu vì spike âm
        float filteredPulseValue = FilterBuLp1_Step(&po->lpf, -irACValue);
        bool beatDetected = BeatDetector_AddSample(&po->beatDetector, filteredPulseValue, HAL_GetTick());

        if (BeatDetector_GetRate(&po->beatDetector) > 0)
        {
            po->state = PULSEOXIMETER_STATE_DETECTING;
            SpO2Calculator_Update(&po->spO2calculator, irACValue, redACValue, beatDetected);
        }
        else if (po->state == PULSEOXIMETER_STATE_DETECTING)
        {
            po->state = PULSEOXIMETER_STATE_IDLE;
            SpO2Calculator_Reset(&po->spO2calculator);
        }

        // Nếu cần debug, có thể gửi giá trị qua UART ở đây

        if (beatDetected && po->onBeatDetected)
        {
            po->onBeatDetected();
        }
    }
}

static void checkCurrentBias(PulseOximeter *po)
{
    // Điều chỉnh dòng LED đỏ để cân bằng DC giữa IR và RED
    if (HAL_GetTick() - po->tsLastBiasCheck > CURRENT_ADJUSTMENT_PERIOD_MS)
    {
        bool changed = false;
        if (DCRemover_GetDCW(&po->irDCRemover) - DCRemover_GetDCW(&po->redDCRemover) > 70000 && po->redLedCurrentIndex < MAX30100_LED_CURR_50MA)
        {
            ++po->redLedCurrentIndex;
            changed = true;
        }
        else if (DCRemover_GetDCW(&po->redDCRemover) - DCRemover_GetDCW(&po->irDCRemover) > 70000 && po->redLedCurrentIndex > 0)
        {
            --po->redLedCurrentIndex;
            changed = true;
        }

        if (changed)
        {
            MAX30100_SetLedsCurrent(&po->hrm, po->irLedCurrent, (LEDCurrent)po->redLedCurrentIndex);
            po->tsLastCurrentAdjustment = HAL_GetTick();
            // Có thể gửi giá trị qua UART nếu cần debug
        }

        po->tsLastBiasCheck = HAL_GetTick();
    }
}
