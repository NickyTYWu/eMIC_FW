#include <string.h>
#include "ftm_ringbuff.h"
#include "ftm_common.h"


void cbInit(RingBuffer *cb, int size) {
	cb->size = size + 1;
	//cb->size = size;
	cb->start = 0;
	cb->end = 0;
	//cb->elems = (ElemType *)calloc(cb->size, sizeof(ElemType));
}

void cbPrint(RingBuffer *cb) {
	FTM_printf("size=0x%x, start=%d, end=%d\n", cb->size, cb->start, cb->end);
}

int cbIsFull(RingBuffer *cb) {
	return (cb->end + 1) % cb->size == cb->start;
	//return cb->end == (cb->start ^ cb->size); /* This inverts the most significant bit of start before comparison */
}

int cbIsEmpty(RingBuffer *cb) {
	return cb->end == cb->start;
}

int cbIncr(RingBuffer *cb, int p) {
	return (p + 1)&(2 * cb->size - 1); /* start and end pointers incrementation is done modulo 2*size */
}

void cbWrite(RingBuffer *cb, ElemType *elem) {
	//cb->elems[cb->end&(cb->size - 1)] = *elem;
	//if (cbIsFull(cb)) /* full, overwrite moves start pointer */
	//cb->start = cbIncr(cb, cb->start);
	//cb->end = cbIncr(cb, cb->end);
	cb->elems[cb->end] = *elem;
	cb->end = (cb->end + 1) % cb->size;
	if (cb->end == cb->start)
	cb->start = (cb->start + 1) % cb->size; /* full, overwrite */
}

void cbRead(RingBuffer *cb, ElemType *elem) {
	//*elem = cb->elems[cb->start&(cb->size - 1)];
	//cb->start = cbIncr(cb, cb->start);
	*elem = cb->elems[cb->start];
	cb->start = (cb->start + 1) % cb->size;
}
