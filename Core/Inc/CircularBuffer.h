#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

// Định nghĩa kích thước buffer, có thể sửa theo nhu cầu
#define CIRCULAR_BUFFER_SIZE 16

// Định nghĩa kiểu dữ liệu lưu trong buffer (ví dụ cho MAX30100)
typedef struct
{
	uint16_t ir;
	uint16_t red;
} SensorReadout;

typedef struct
{
	SensorReadout buffer[CIRCULAR_BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t count;
} CircularBuffer;

// Khởi tạo buffer
void CircularBuffer_Init(CircularBuffer *cb);

// Thêm phần tử vào cuối buffer
bool CircularBuffer_Push(CircularBuffer *cb, SensorReadout value);

// Xóa phần tử đầu buffer
bool CircularBuffer_Pop(CircularBuffer *cb, SensorReadout *value);

// Lấy phần tử đầu buffer mà không xóa
bool CircularBuffer_First(CircularBuffer *cb, SensorReadout *value);

// Lấy phần tử cuối buffer mà không xóa
bool CircularBuffer_Last(CircularBuffer *cb, SensorReadout *value);

// Truy cập phần tử theo chỉ số
bool CircularBuffer_Get(CircularBuffer *cb, uint8_t index, SensorReadout *value);

// Số phần tử hiện có
uint8_t CircularBuffer_Size(CircularBuffer *cb);

// Số vị trí còn trống
uint8_t CircularBuffer_Available(CircularBuffer *cb);

// Tổng dung lượng buffer
uint8_t CircularBuffer_Capacity(CircularBuffer *cb);

// Kiểm tra rỗng
bool CircularBuffer_IsEmpty(CircularBuffer *cb);

// Kiểm tra đầy
bool CircularBuffer_IsFull(CircularBuffer *cb);

// Xóa toàn bộ buffer
void CircularBuffer_Clear(CircularBuffer *cb);

#endif