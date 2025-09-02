#ifndef FTM_RINGBUFF_H_
#define FTM_RINGBUFF_H_

#include "ftm_common.h"

/* This approach adds one bit to end and start pointers */
typedef char ElemType;
/* Circular buffer object */

typedef struct {
	int         size;   /* maximum number of elements           */
	int         start;  /* index of oldest element              */
	int         end;    /* index at which to write new element  */
	ElemType	elems[FTMD_RX_DUMP_SIZE];  /* vector of elements                   */
} RingBuffer;


void cbInit(RingBuffer *, int );
void cbPrint(RingBuffer *);
int cbIsFull(RingBuffer *);
int cbIsEmpty(RingBuffer *);
int cbIncr(RingBuffer *, int );
void cbWrite(RingBuffer *, ElemType *);
void cbRead(RingBuffer *, ElemType *);
extern void FTM_printf(const char *fmt, ...);
#endif /* FTM_RINGBUFF_H_ */
