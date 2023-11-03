#ifndef MIDI_BUFFER_H
#define MIDI_BUFFER_H

#include "stdint.h"

#define MIDI_BUFFER_OK		0
#define MIDI_BUFFER_FULL	1
#define MIDI_BUFFER_EMPTY	2

typedef struct
{
	uint8_t *buffer;	// Ring buffer for data
	uint16_t head;		// Index for data head position
	uint16_t tail;		// Index for data tail position
	uint8_t full;		// Flag for whether buffer is full
	uint16_t size;		// Capacity of the buffer in bytes
} MidiRingBuf;

void midi_initRingBuffer(MidiRingBuf* buffer, uint16_t size);
void midi_resetRingBuffer(MidiRingBuf* buffer);
uint16_t midi_numDataRingBuffer(MidiRingBuf* buffer);
void midi_advanceRingBufferPointer(MidiRingBuf* buffer);
void midi_retreatRingBufferPointer(MidiRingBuf* buffer);
uint8_t midi_ringBufferEmpty(MidiRingBuf* buffer);

#endif // MIDI_BUFFER_H