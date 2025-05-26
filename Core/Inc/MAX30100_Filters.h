#ifndef MAX30100_FILTERS_H
#define MAX30100_FILTERS_H

#include <stdint.h>

// Bộ lọc Butterworth bậc 1 (Low pass)
typedef struct
{
	float v[2];
} FilterBuLp1;

// Khởi tạo bộ lọc Butterworth
static inline void FilterBuLp1_Init(FilterBuLp1 *f)
{
	f->v[0] = 0.0f;
	f->v[1] = 0.0f;
}

// Bước lọc (step)
static inline float FilterBuLp1_Step(FilterBuLp1 *f, float x)
{
	f->v[0] = f->v[1];
	f->v[1] = (0.2452372752527856f * x) + (0.5095254494944288f * f->v[0]);
	return (f->v[0] + f->v[1]);
}

// Bộ lọc loại DC (DC remover)
typedef struct
{
	float alpha;
	float dcw;
} DCRemover;

// Khởi tạo DCRemover với alpha
static inline void DCRemover_Init(DCRemover *dcr, float alpha)
{
	dcr->alpha = alpha;
	dcr->dcw = 0.0f;
}

// Bước lọc (step)
static inline float DCRemover_Step(DCRemover *dcr, float x)
{
	float olddcw = dcr->dcw;
	dcr->dcw = x + dcr->alpha * dcr->dcw;
	return dcr->dcw - olddcw;
}

// Lấy giá trị DC hiện tại
static inline float DCRemover_GetDCW(DCRemover *dcr)
{
	return dcr->dcw;
}

#endif