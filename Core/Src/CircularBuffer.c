#include "CircularBuffer.h"
#include <string.h>

// Khởi tạo buffer
void CircularBuffer_Init(CircularBuffer *cb)
{
	cb->head = 0;
	cb->tail = 0;
	cb->count = 0;
	memset(cb->buffer, 0, sizeof(cb->buffer));
}

// Thêm phần tử vào cuối buffer (push)
bool CircularBuffer_Push(CircularBuffer *cb, SensorReadout value)
{
	if (cb->count == CIRCULAR_BUFFER_SIZE)
	{
		// Buffer đầy, ghi đè phần tử đầu
		cb->head = (cb->head + 1) % CIRCULAR_BUFFER_SIZE;
		cb->count--;
	}
	cb->buffer[cb->tail] = value;
	cb->tail = (cb->tail + 1) % CIRCULAR_BUFFER_SIZE;
	cb->count++;
	return true;
}

// Xóa phần tử đầu buffer (pop/shift)
bool CircularBuffer_Pop(CircularBuffer *cb, SensorReadout *value)
{
	if (cb->count == 0)
		return false;
	*value = cb->buffer[cb->head];
	cb->head = (cb->head + 1) % CIRCULAR_BUFFER_SIZE;
	cb->count--;
	return true;
}

// Lấy phần tử đầu buffer mà không xóa
bool CircularBuffer_First(CircularBuffer *cb, SensorReadout *value)
{
	if (cb->count == 0)
		return false;
	*value = cb->buffer[cb->head];
	return true;
}

// Lấy phần tử cuối buffer mà không xóa
bool CircularBuffer_Last(CircularBuffer *cb, SensorReadout *value)
{
	if (cb->count == 0)
		return false;
	uint8_t last = (cb->tail == 0) ? (CIRCULAR_BUFFER_SIZE - 1) : (cb->tail - 1);
	*value = cb->buffer[last];
	return true;
}

// Truy cập phần tử theo chỉ số (0 là phần tử đầu)
bool CircularBuffer_Get(CircularBuffer *cb, uint8_t index, SensorReadout *value)
{
	if (index >= cb->count)
		return false;
	uint8_t pos = (cb->head + index) % CIRCULAR_BUFFER_SIZE;
	*value = cb->buffer[pos];
	return true;
}

// Số phần tử hiện có
uint8_t CircularBuffer_Size(CircularBuffer *cb)
{
	return cb->count;
}

// Số vị trí còn trống
uint8_t CircularBuffer_Available(CircularBuffer *cb)
{
	return CIRCULAR_BUFFER_SIZE - cb->count;
}

// Tổng dung lượng buffer
uint8_t CircularBuffer_Capacity(CircularBuffer *cb)
{
	return CIRCULAR_BUFFER_SIZE;
}

// Kiểm tra rỗng
bool CircularBuffer_IsEmpty(CircularBuffer *cb)
{
	return cb->count == 0;
}

// Kiểm tra đầy
bool CircularBuffer_IsFull(CircularBuffer *cb)
{
	return cb->count == CIRCULAR_BUFFER_SIZE;
}

// Xóa toàn bộ buffer
void CircularBuffer_Clear(CircularBuffer *cb)
{
	cb->head = 0;
	cb->tail = 0;
	cb->count = 0;
	memset(cb->buffer, 0, sizeof(cb->buffer));
}