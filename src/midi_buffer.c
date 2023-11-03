#include "midi_buffer.h"

void midi_initRingBuffer(MidiRingBuf* buffer, uint16_t size)
{
	// Assign the buffer capacity and init values
	buffer->size = size;
	midi_resetRingBuffer(buffer);

	for(int i=0; i<size; i++)
	{
		buffer->buffer[i] = 255;
	}
}

/**
  * @brief Resets the buffer
  * @param *buffer pointer to the buffer
  * @retval None
  */
void midi_resetRingBuffer(MidiRingBuf* buffer)
{
	buffer->head = 0;
	buffer->tail = 0;
	buffer->full = 0;
}

/**
  * @brief Returns the number of elements in the buffer
  * @param *buffer pointer to the buffer
  * @retval Number of elements
  */
uint16_t midi_numDataRingBuffer(MidiRingBuf* buffer)
{
	uint16_t size = buffer->size;
	if(!buffer->full)
	{
		if(buffer->head >= buffer->tail)
		{
			size = buffer->head - buffer->tail;
		}
		else
		{
			size = buffer->size + buffer->head - buffer->tail;
		}
	}
	return size;
}

/**
  * @brief Advances the buffer pointers (head and tail)
  * @param *buffer pointer to the buffer
  * @retval None
  */
void midi_advanceRingBufferPointer(MidiRingBuf* buffer)
{
	if(buffer->full)
	{
		buffer->tail = (buffer->tail + 1) % buffer->size;
	}

	buffer->head = (buffer->head + 1) % buffer->size;
	buffer->full = (buffer->head == buffer->tail);
}

/**
  * @brief Retreats the buffer pointers (head and tail)
  * @param *buffer pointer to the buffer
  * @retval None
  */
void midi_retreatRingBufferPointer(MidiRingBuf* buffer)
{
	buffer->full = 0;
	buffer->tail = (buffer->tail + 1) % buffer->size;
}

/**
  * @brief	Puts a byte of data in the buffer and returns an error if buffer is full
  * @param	*buffer pointer to the buffer
  * 			data the data to be copied into the buffer
  * @retval	Errorstate
  */
uint8_t midi_ringBufferPut(MidiRingBuf* buffer, uint8_t data)
{

	if(!buffer->full)
	{
		buffer->buffer[buffer->head] = data;
		midi_advanceRingBufferPointer(buffer);
		return MIDI_BUFFER_OK;
	}
	return MIDI_BUFFER_FULL;
}

/**
  * @brief	Gets a byte of data from the buffer and returns an error if buffer is empty
  * @param	*buffer pointer to the buffer
  * 			data pointer to store the data
  * @retval	Errorstate
  */
uint8_t midi_ringBufferGet(MidiRingBuf* buffer, uint8_t* data)
{
	if(!midi_ringBufferEmpty(buffer))
	{
		*data = buffer->buffer[buffer->tail];
		midi_retreatRingBufferPointer(buffer);
		return MIDI_BUFFER_OK;
	}
	return MIDI_BUFFER_EMPTY;
}

uint8_t midi_ringBufferEmpty(MidiRingBuf* buffer)
{
	return (!buffer->full && (buffer->head == buffer->tail));
}

