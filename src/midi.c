/*
 * midi.c
 *
 *  Created on: 15 Dec 2019
 *      Author: samspencer
 */

#include "midi.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifdef USE_USB_MIDI
#include "tusb.h"
#endif

#define TRUE 	1
#define FALSE 	0
// Sizes of the data ring buffers
// These can probably be lowered to conserve RAM depending on the application speed
#define DMA_THRESHOLD_NUM	9
#define STATUS_BYTE_NUM 1

uint32_t sysExId = 0;
uint8_t midiInChannel;	// MIDI in channel is set to omni by default
uint8_t numMidiInterfaces = 0;

/*	PRIVATE FUNCTION PROTOTYPES			*/
/* MIDI DATA PFP*/
MidiStatusByte midi_getStatus(MidiDataType inType, MidiChannel inChannel);

// -------------------- Clock -------------------- //
#ifdef USE_MIDI_CLOCK
/**
  * @brief
  * @param
  * @retval
  */
void midi_clockInit(MidiClockTx* midiClock)
{
	midiClock->subDivision = MidiClock4;
	midiClock->messageCounter = 0;
	midiClock->tempoLed = MIDI_CLOCK_NO_ASSIGN_INDEX;
	midiClock->statusLed = MIDI_CLOCK_NO_ASSIGN_INDEX;
	midiClock->switchIndex = MIDI_CLOCK_NO_ASSIGN_INDEX;
	midiClock->state = MidiClockStopped;
	midiClock->useLed = TRUE;

	for(uint8_t i=0; i<NUM_TAP_INTERVALS; i++)
	{
		midiClock->tapIntervals[i] = 0;
	}

	for(uint8_t i = 0; i<numMidiInterfaces; i++)
	{
		midiClock->config->midiHandles[i] = NULL;
	}
	midi_clockSetTempo(midiClock, DEFAULT_CLOCK_BPM*10);
}

/**
  * @brief
  * @param
  * @retval
  */

MidiErrorState midi_clockAssignHandle(MidiClockTx* midiClock, MidiInterface** midiHandles, uint8_t numHandles)
{
	// Ensure there is enough space to store the handle
	if(numHandles > numMidiInterfaces || midiHandles == NULL || midiClock == NULL)
	{
		return MidiParamError;
	}

	for(int i=0; i<numHandles; i++)
	{
		midiClock->config->midiHandles[i] = midiHandles[i];
	}
	return MidiOk;
}

MidiErrorState midi_clockSetTempo(MidiClockTx* midiClock, uint16_t newTempo)
{
	if(newTempo < MIN_CLOCK_BPM * 10)
	{
		midiClock->bpm = MIN_CLOCK_BPM * 10;
	}
	else if(newTempo > MAX_CLOCK_BPM * 10)
	{
		midiClock->bpm = MAX_CLOCK_BPM * 10;
	}
	else
	{
		midiClock->bpm = newTempo;
	}

	uint32_t newPeriod = round((25000000 / midiClock->bpm) * MIDI_CLOCK_TIMER_OFFSET);
	midiClock->clockTim->Instance->ARR = newPeriod;
	midiClock->clockTim->Init.Period = (uint16_t)newPeriod;
	return MidiOk;
}

void midi_clockStart(MidiClockTx* midiClock)
{
	midiClock->state = MidiClockRunning;
	__HAL_TIM_SET_COUNTER(midiClock->clockTim, 0);
	for(uint8_t i = 0; i<numMidiInterfaces; i++)
	{
		if(midiClock->config->midiHandles[i] == NULL)
		{
			break;
		}
		midi_Send(midiClock->config->midiHandles[i], Start, 0, 0, 0);
	}
	HAL_TIM_Base_Start_IT(midiClock->clockTim);

}

void midi_clockStop(MidiClockTx* midiClock)
{
	midiClock->state = MidiClockStopped;
	HAL_TIM_Base_Stop_IT(midiClock->clockTim);
	for(uint8_t i = 0; i<numMidiInterfaces; i++)
	{
		if(midiClock->config->midiHandles[i] == NULL)
		{
			break;
		}
		midi_Send(midiClock->config->midiHandles[i], Stop, 0, 0, 0);
	}
}

void midi_clockSendStop(MidiClockTx* midiClock)
{
	midiClock->state = MidiClockStopped;
	for(uint8_t i = 0; i<numMidiInterfaces; i++)
	{
		if(midiClock->config->midiHandles[i] == NULL)
		{
			break;
		}
		midi_send(midiClock->config->midiHandles[i], Stop, 0, 0, 0);
	}
}

void midi_clockSendStart(MidiClockTx* midiClock)
{
	midiClock->state = MidiClockRunning;
	__HAL_TIM_SET_COUNTER(midiClock->clockTim, 0);
	for(uint8_t i = 0; i<numMidiInterfaces; i++)
	{
		if(midiClock->config->midiHandles[i] == NULL)
		{
			break;
		}
		midi_send(midiClock->config->midiHandles[i], Start, 0, 0, 0);
	}
}

void midi_clockSend(MidiClockTx* midiClock)
{
	for(uint8_t i = 0; i<numMidiInterfaces; i++)
	{
		if(midiClock->config->midiHandles[i] != NULL && midiClock->config->midiHandles[i]->active)
		{
			midi_send(midiClock->config->midiHandles[i], Clock, 0, 0, 0);
		}
	}
	midiClock->messageCounter++;
}

void midi_clockTapTempoUpdate(MidiClockTx* midiClock)
{
	// Update the most recent tap interval
	midiClock->lastTapTime = HAL_GetTick() - midiClock->lastTapTime;

	// If the last tap was longer than the timeout duration, restart the clock to sync the downbeat
	if(midiClock->lastTapTime < MIDI_CLOCK_TAP_TIMEOUT)
	{
		// Increment the index and update the oldest tap interval

		midiClock->tapIntervals[midiClock->tapIntervalIndex] = midiClock->lastTapTime;
		midiClock->tapIntervalIndex++;
		if(midiClock->tapIntervalIndex == NUM_TAP_INTERVALS)
		{
			midiClock->tapIntervalIndex = 0;
		}
		// Average the tap tempo with a greater weighting towards the latest tap
		float tapAverage = 0;
		uint8_t indexCounter = 0;
		for(uint8_t i=0; i<NUM_TAP_INTERVALS; i++)
		{
			if(midiClock->tapIntervals[i] > 0)
			{
				tapAverage += midiClock->tapIntervals[i];
				indexCounter++;
			}
		}
		tapAverage = (tapAverage / indexCounter);
		uint16_t newBpm = round(10000/(tapAverage/60));
		midi_clockSetTempo(midiClock, newBpm);

	}
	else
	{
		for(uint8_t i=0; i<NUM_TAP_INTERVALS; i++)
		{
			midiClock->tapIntervals[i] = 0;
			midiClock->tapIntervalIndex = 0;
		}
		midiClock->clockTim->Instance->CNT = 0;
		midi_clockSend(midiClock);
		HAL_TIM_Base_Start_IT(midiClock->clockTim);
	}
	midiClock->lastTapTime = HAL_GetTick();
}

void midi_turnOffMidiClock(MidiClockTx* midiClock)
{
	for(uint8_t i=0; i<NUM_TAP_INTERVALS; i++)
		{
			midiClock->tapIntervals[i] = 0;
			midiClock->tapIntervalIndex = 0;
		}
	midiClock->clockTim->Instance->CNT = 0;
	midiClock->state = MidiClockStopped;
	HAL_TIM_Base_Stop_IT(midiClock->clockTim);
}
#endif

// --------------- Init and Config --------------- //

MidiErrorState midi_init(	MidiInterface* midiHandle, UART_HandleTypeDef* uartHandle, MidiDeviceType type,
		MidiChannel setChannel, MidiInterfaceDirection direction, uint8_t* ptrRxBuf,
		uint16_t size, IRQn_Type irq)
{
	// Assign data to the passed handle
	midiHandle->direction = direction;
	midiHandle->uartHandle = uartHandle;
	midiHandle->channel = setChannel;
	midiHandle->deviceType = type;
	midiHandle->newMessage = FALSE;
	midiHandle->txDataPending = FALSE;
	midiHandle->txState = MidiReady;
	midiHandle->numThruHandles = 0;

	if(type == UartMidi)
	{
		CLEAR_BIT(midiHandle->uartHandle->Instance->CR1, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE));
		CLEAR_BIT(midiHandle->uartHandle->Instance->CR3, (USART_CR3_EIE | USART_CR3_RXFTIE));

		/* In case of reception waiting for IDLE event, disable also the IDLE IE interrupt source */
		if (midiHandle->uartHandle->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
		{
			CLEAR_BIT(midiHandle->uartHandle->Instance->CR1, USART_CR1_IDLEIE);
		}

		READ_REG(midiHandle->uartHandle->Instance->ISR);
		READ_REG(midiHandle->uartHandle->Instance->RDR);
		/* At end of Rx process, restore huart->RxState to Ready */
		midiHandle->uartHandle->RxState = HAL_UART_STATE_READY;
		midiHandle->uartHandle->ReceptionType = HAL_UART_RECEPTION_STANDARD;

		/* Reset RxIsr function pointer */
		midiHandle->uartHandle->RxISR = NULL;
		// Once the error has been handled, restart
	}
	midiHandle->uartIRQn = irq;

#ifndef USE_UART_FIFO
	//Initialize the buffers
	midiHandle->rxBuf.buffer = ptrRxBuf;
	midi_initRingBuffer(&midiHandle->rxBuf, size);
	midiHandle->txBufIndex = 0;
	midiHandle->txQueueIndex = 0;
#endif

	// Assign NULL pointers to all callbacks to prevent hard faults (by checking for NULL)
	midiHandle->mNoteOffCallback = NULL;
	midiHandle->mNoteOnCallback = NULL;
	midiHandle->mAfterTouchPolyCallback = NULL;
	midiHandle->mControlChangeCallback = NULL;
	midiHandle->mProgramChangeCallback = NULL;
	midiHandle->mAfterTouchchannelCallback = NULL;
	midiHandle->mPitchBendCallback = NULL;
	midiHandle->mSystemExclusiveCallback = NULL;
	midiHandle->mTimeCodeQuarterFrameCallback = NULL;
	midiHandle->mSongPositionCallback = NULL;
	midiHandle->mSongSelectCallback = NULL;
	midiHandle->mTuneRequestCallback = NULL;
	midiHandle->mClockCallback = NULL;
	midiHandle->mStartCallback = NULL;
	midiHandle->mContinueCallback = NULL;
	midiHandle->mStopCallback = NULL;
	midiHandle->mActiveSensingCallback = NULL;
	midiHandle->mSystemResetCallback = NULL;
	numMidiInterfaces++;
	return MidiOk;
}

MidiErrorState midi_begin(MidiInterface* midiHandle)
{
	midiHandle->active = TRUE;
	midiHandle->newMessage = FALSE;
	midiHandle->txDataPending = FALSE;
	midiHandle->txState = MidiReady;
	if(midiHandle->direction != MidiOutOnly)
	{
		// Set the state to waiting for the status byte
		midiHandle->pendingRxType = MidiStatus;
		if(midiHandle->deviceType == UartMidi)
		{
			HAL_UART_AbortReceive_IT(midiHandle->uartHandle);
			// Wait until uart is ready
			__HAL_UART_CLEAR_OREFLAG(midiHandle->uartHandle);
			while(HAL_UART_GetState(midiHandle->uartHandle) != HAL_UART_STATE_READY);
			if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, STATUS_BYTE_NUM) != HAL_OK)
			{
				return MidiHalError;
			}
		}
		else if(midiHandle->deviceType == UsbMidi)
		{
			//USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
			//USBD_MIDI_ReceivePacket(&hUsbDeviceFS);
		}
	}
	return MidiOk;
}

MidiErrorState midi_assignSysExId(uint32_t id)
{
	// ensure that ID is no longer than 3 bytes
	if(id > 0xffffff)
	{
		return MidiParamError;
	}
	sysExId = id;
	return MidiOk;
}

void midi_assignChannel(uint8_t newChannel)
{
	if(newChannel <= 16)
	{
		midiInChannel = newChannel;
	}
}

// ---------------------- IO --------------------- //

/**
  * @brief System Clock Configuration
  * @param none
  * @retval None
  */
MidiErrorState midi_sendNoteOn(MidiInterface *midiHandle, MidiDataByte  inNoteNumber,
				MidiDataByte  inVelocity, MidiChannel inChannel)
{
	MidiErrorState status = midi_send(midiHandle, NoteOn, inNoteNumber, inVelocity, inChannel);
	return status;
}

/**
  * @brief System Clock Configuration
  * @param none
  * @retval None
  */
MidiErrorState midi_sendNoteOff(	MidiInterface *midiHandle, MidiDataByte  inNoteNumber,
					MidiDataByte  inVelocity, MidiChannel inChannel)
{
	MidiErrorState status = midi_send(midiHandle, NoteOff, inNoteNumber, inVelocity, inChannel);
	return status;
}

/**
  * @brief System Clock Configuration
  * @param none
  * @retval None
  */
MidiErrorState midi_sendProgramChange(	MidiInterface *midiHandle, MidiDataByte  inProgramNumber,
						MidiChannel inChannel)
{
	MidiErrorState status = midi_send(midiHandle, ProgramChange, inProgramNumber, NULL_BYTE, inChannel);
	return status;
}

/**
  * @brief System Clock Configuration
  * @param none
  * @retval None
  */
MidiErrorState midi_sendControlChange( MidiInterface *midiHandle, MidiDataByte  inControlNumber,
						MidiDataByte  inControlValue, MidiChannel inChannel)
{
	MidiErrorState status = midi_send(midiHandle, ControlChange, inControlNumber, inControlValue, inChannel);
	return status;
}



// ------------------ Input ----------------- //
MidiErrorState midi_read(MidiInterface *midiHandle)
{
	// Check to see if any new complete messages have been received
	if(midiHandle->newMessage)
	{
		while(midiHandle->newMessage)
		{
			uint16_t numBytes = 0;
			// Get the oldest status byte in the buffer
			MidiStatusByte status = 0;
			MidiDataByte d1 = 0;
			MidiDataByte d2 = 0;
			midi_ringBufferGet(&midiHandle->rxBuf, &status);
			uint8_t upperNibble = status & 0xf0;

			// Channel Voice messages
			if(upperNibble != 0xf0)
			{
				// Extract the channel from the lower nibble of the status byte
				// This must be offset by one to account for the zero indexing
				uint8_t channel = (status & 0x0f) + 1;

				// Program change is the only channel voice message which uses 1 data byte
				// All other channel voice messages require 2 data bytes to be received
				switch (upperNibble)
				{
				case NoteOff:
					midi_ringBufferGet(&midiHandle->rxBuf, &d1);
					midi_ringBufferGet(&midiHandle->rxBuf, &d2);
					numBytes += 3;
					if(midiInChannel == MIDI_CHANNEL_OMNI || midiInChannel == channel)
					{
						if(midiHandle->mNoteOffCallback != NULL)
						{
							midiHandle->mNoteOffCallback(midiHandle, channel, d1, d2);
						}
					}
					break;
				case NoteOn:
					midi_ringBufferGet(&midiHandle->rxBuf, &d1);
					midi_ringBufferGet(&midiHandle->rxBuf, &d2);
					numBytes += 3;
					if(midiInChannel == MIDI_CHANNEL_OMNI || midiInChannel == channel)
					{
						if(midiHandle->mNoteOnCallback != NULL)
						{
							midiHandle->mNoteOnCallback(midiHandle, channel, d1, d2);
						}
					}
					break;
				case AfterTouchPoly:

					break;
				case ControlChange:
					midi_ringBufferGet(&midiHandle->rxBuf, &d1);
					midi_ringBufferGet(&midiHandle->rxBuf, &d2);
					numBytes += 3;
					if(midiInChannel == MIDI_CHANNEL_OMNI || midiInChannel == channel)
					{
						if(midiHandle->mControlChangeCallback != NULL)
						{
							midiHandle->mControlChangeCallback(midiHandle, channel, d1, d2);
						}
					}
					break;
				case ProgramChange:
					midi_ringBufferGet(&midiHandle->rxBuf, &d1);
					numBytes += 2;
					if(midiInChannel == MIDI_CHANNEL_OMNI || midiInChannel == channel)
					{
						if(midiHandle->mProgramChangeCallback != NULL)
						{
							midiHandle->mProgramChangeCallback(midiHandle, channel, d1);
						}
					}
					break;
				case AfterTouchChannel:

					break;
				case PitchBend:

					break;
				}
				uint8_t thruPacket[] = {status, d1, d2};
				for(int i=0; i<numMidiInterfaces; i++)
				{
					if(midiHandle->thruHandles[i] != NULL && midiHandle->numThruHandles > 0 && midiHandle->thruHandles[i]->active)
					{
						// Check that there is enough room in the queue
						if(midiHandle->thruHandles[i]->txQueueIndex <  (MIDI_TX_QUEUE_SIZE-1-numBytes))
						{
							for(int j=0; j<numBytes; j++)
							{
								midiHandle->thruHandles[i]->txQueue[midiHandle->thruHandles[i]->txQueueIndex + j] = thruPacket[j];
							}
							midiHandle->thruHandles[i]->txQueueIndex += numBytes;
						}
					}
				}
			}

			// System Common and System Real-Time messages
			else
			{
				uint8_t thruPacket[3];
				uint8_t idByte1 = 0, idByte2 = 0, idByte3 = 0;
				uint16_t beats;
				switch (status)
				{
				case SystemExclusive:
					midi_ringBufferGet(&midiHandle->rxBuf, &idByte1);
					midi_ringBufferGet(&midiHandle->rxBuf, &idByte2);
					midi_ringBufferGet(&midiHandle->rxBuf, &idByte3);
					// Upon receiving valid SysEx message, check against the SysEx ID for reception
 					if(	idByte1 == ((sysExId>>16) & 0xff) &&
							idByte2 == ((sysExId>>8) & 0xff) &&
							idByte3 == ((sysExId) & 0xff))
					{
						// Get the number of bytes available in the message
						uint16_t dataLen = midi_numDataRingBuffer(&midiHandle->rxBuf) - 1;
						if(midiHandle->mSystemExclusiveCallback != NULL)
						{
							midiHandle->mSystemExclusiveCallback(midiHandle, dataLen);
							// Fetch the last byte to handle the SysEx end byte
						}
						//midi_numDataRingBuffer(&midiHandle->rxBuf);
						// TODO thru handling for sysex
						numBytes = 0;
					}
					break;
				case TimeCodeQuarterFrame:
					midi_ringBufferGet(&midiHandle->rxBuf, &d1);
					if(midiHandle->mTimeCodeQuarterFrameCallback != NULL)
					{
						midiHandle->mTimeCodeQuarterFrameCallback(midiHandle, d1);
					}
					numBytes = 2;
					thruPacket[0] = status;
					thruPacket[1] = d1;
					break;
				case SongSelect:
					midi_ringBufferGet(&midiHandle->rxBuf, &d1);
					if(midiHandle->mSongSelectCallback != NULL)
					{
						midiHandle->mSongSelectCallback(midiHandle, d1);
					}
					numBytes = 2;
					thruPacket[0] = status;
					thruPacket[1] = d1;
					break;
				case SongPosition:
					midi_ringBufferGet(&midiHandle->rxBuf, &d1);
					midi_ringBufferGet(&midiHandle->rxBuf, &d2);
					beats = (d2 << 8 & 0xff) | (d1 & 0xff);
					if(midiHandle->mSongPositionCallback != NULL)
					{
						midiHandle->mSongPositionCallback(midiHandle, beats);
					}
					numBytes = 3;
					thruPacket[0] = status;
					thruPacket[1] = d1;
					thruPacket[2] = d2;
					break;
				case TuneRequest:
					if(midiHandle->mTuneRequestCallback != NULL)
					{
						midiHandle->mTuneRequestCallback(midiHandle);
					}
					numBytes = 1;
					thruPacket[0] = status;
					break;
				case Clock:
					if(midiHandle->mClockCallback != NULL)
					{
						midiHandle->mClockCallback(midiHandle);
					}
					numBytes = 1;
					thruPacket[0] = status;
					break;
				case Start:
					if(midiHandle->mStartCallback != NULL)
					{
						midiHandle->mStartCallback(midiHandle);
					}
					numBytes = 1;
					thruPacket[0] = status;
					break;
				case Continue:
					if(midiHandle->mContinueCallback != NULL)
					{
						midiHandle->mContinueCallback(midiHandle);
					}
					numBytes = 1;
					thruPacket[0] = status;
					break;
				case Stop:
					if(midiHandle->mStopCallback != NULL)
					{
						midiHandle->mStopCallback(midiHandle);
					}
					numBytes = 1;
					thruPacket[0] = status;
					break;
				case ActiveSensing:
					if(midiHandle->mActiveSensingCallback != NULL)
					{
						midiHandle->mActiveSensingCallback(midiHandle);
					}
					numBytes = 1;
					thruPacket[0] = status;
					break;
				case SystemReset:
					if(midiHandle->mSystemResetCallback != NULL)
					{
						midiHandle->mSystemResetCallback(midiHandle);
					}
					numBytes = 1;
					thruPacket[0] = status;
					break;
				}
				for(int i=0; i<numMidiInterfaces; i++)
				{
					if(midiHandle->thruHandles[i] != NULL && midiHandle->numThruHandles > 0 && midiHandle->thruHandles[i]->active)
					{
						// Check that there is enough room in the queue
						if(midiHandle->thruHandles[i]->txQueueIndex <  (MIDI_TX_QUEUE_SIZE-1-numBytes))
						{
							for(int j=0; j<numBytes; j++)
							{
								midiHandle->thruHandles[i]->txQueue[midiHandle->thruHandles[i]->txQueueIndex + j] = thruPacket[j];
							}
							midiHandle->thruHandles[i]->txQueueIndex += numBytes;
						}
					}
				}
			}

			// If there are no new messages, reset the newMessage flag
			if(midi_ringBufferEmpty(&midiHandle->rxBuf))
			{
				midiHandle->newMessage = FALSE;
			}
		}
		for(int i=0; i<numMidiInterfaces; i++)
		{
			// Check that the thru handle is valid
			if( midiHandle->thruHandles[i] != NULL && midiHandle->thruHandles[i]->active
					&& (midiHandle->thruHandles[i]->direction == MidiOutOnly|| midiHandle->thruHandles[i]->direction == MidiFull))
			{
				midi_sendPacket(midiHandle->thruHandles[i]);
				midiHandle->thruHandles[i]->txQueueIndex = 0;
			}
		}
		return MidiOk;
	}
	return MidiNoData;
}

/*
 * Special use case of the MIDI library. This read copies all received valid MIDI data to the provided buffer.
 * This is useful for chips that are acting as a MIDI bridge/expander, and don't require callbacks.
 * It is the responsibility of the calling function to ensure the buffer is large enough/empty,
 * otherwise data may be dropped and/or overwritten.
 * Note that no channel checking is performed, only checks for valid MIDI commands
 */
MidiErrorState midi_readCustom(MidiInterface *midiHandle, uint8_t* buf, uint16_t* numCopiedBytes)
{
	// Check to see if any new complete messages have been received
	if(midiHandle->newMessage)
	{
		uint16_t numBytes = 0;
		while(midiHandle->newMessage)
		{
			// Get the oldest status byte in the buffer
			midi_ringBufferGet(&midiHandle->rxBuf, &buf[numBytes]);
			int numMsgBytes = midi_numDataBytesForMessage(buf[numBytes]);
			numBytes++;

			if(numMsgBytes == 1)
			{
				midi_ringBufferGet(&midiHandle->rxBuf, &buf[numBytes]);
				numBytes++;
			}
			else if(numMsgBytes == 2)
			{
				midi_ringBufferGet(&midiHandle->rxBuf, &buf[numBytes]);
				numBytes++;
				midi_ringBufferGet(&midiHandle->rxBuf, &buf[numBytes]);
				numBytes++;
			}
			else if(numMsgBytes == -1)
			{
				// First, check the number of bytes that are in the sysxex message.
				uint16_t numSysExBytes = midi_numDataRingBuffer(&midiHandle->rxBuf);
				for(int i=0; i<numSysExBytes; i++)
				{
					midi_ringBufferGet(&midiHandle->rxBuf, &buf[numBytes]);
					numBytes++;
				}

			}

			// If there are no new messages, reset the newMessage flag
			if(midi_ringBufferEmpty(&midiHandle->rxBuf))
			{
				midiHandle->newMessage = FALSE;
			}
		}
		*numCopiedBytes = numBytes;
		return MidiOk;
	}
	return MidiNoData;
}


// ------------------ Events ----------------- //
/**
  * @brief
  * @param
  * @retval
  */
void midi_txUartHandler(MidiInterface* midiHandle)
{
	// When a transfer has completed, check if more data is available to transfer, if so, transfer it.

	if(midiHandle->txQueueIndex == 0)
	{
		midiHandle->txState = MidiReady;
		return;
	}
	// If not, then set the state flag to ready to indicate that a new packet transfer may commence
	midiHandle->txState = MidiReady;
	// If there is still more data to transfer, begin a new transfer
	midi_sendPacket(midiHandle);
	return;
}

/**
  * @brief	Handles received uart data. Data is received to the rxBuf (or FIFO buffer in FIFO mode).
  * 			This data is only 'peeked' to determine the packet type. Once the first byte of a MIDI message has been read,
  * 			The required number of consecutive bytes are also received in interrupt mode to the rxBuffer.
  * 			The main application is required to call midi_read() to see if any new MIDI data has been received.
  * 			Note that checking which UART MIDI interface was received is done in the main application.
  * 			Note that channel matching checks are performed in the midi_read() function.
  * @param none
  * @retval None
  */
void midi_rxUartHandler(MidiInterface* midiHandle)
{
	// If the handle is waiting for a status byte
	if(midiHandle->pendingRxType == MidiStatus)
	{
		/* First, check if a status byte was received as expected
		 * If not, keep receiving single bytes until a valid status byte is received.
		 * This can occur if the device is connected midway through a midi message, or a fault in another device.
		 */
		uint8_t upperNibble = midiHandle->rxRawBuf[0] & 0xf0;
		if(!((upperNibble >> 7) & 1))
		{

			if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 1) != HAL_OK)
			{
				return;
			}
		}
		// Copy the received byte into the rxBuffer
		if(midi_ringBufferPut(&midiHandle->rxBuf, midiHandle->rxRawBuf[0]) != MidiOk)
		{
			return;
		}

		// Channel Voice messages
		else if(upperNibble != 0xf0)
		{
			// Program change is the only channel voice message which uses 1 data byte
			// All other channel voice messages require 2 data bytes to be received
			if(upperNibble != ProgramChange)
			{
				if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 2) != HAL_OK)
				{
					return;
				}
				midiHandle->pendingNumData = 2;
			}
			else
			{
				if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 1) != HAL_OK)
				{
					return;
				}
				midiHandle->pendingNumData = 1;
			}
			midiHandle->pendingRxType = MidiData;
		}

		// System Common and System Real-Time messages
		else
		{
			// Get the required number of bytes for the message
			int numBytes = midi_numDataBytesForMessage(midiHandle->rxBuf.buffer[0]);

			if(numBytes == 0)
			{
				midiHandle->pendingNumData = 1;
				midiHandle->newMessage = TRUE;
				midiHandle->pendingRxType = MidiStatus;

				// Once data has been received, begin listening for a new status message
				if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 1) != HAL_OK)
				{
					return;
				}
			}
			// Time code quarter frame and song select messages only require 1 data byte
			if(numBytes == 1)
			{
				if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 1) != HAL_OK)
				{
					return;
				}
				midiHandle->pendingNumData = 1;
				midiHandle->pendingRxType = MidiData;
			}
			// Song position messages required 2 data bytes
			else if(numBytes == 2)
			{
				if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 2) != HAL_OK)
				{
					return;
				}
				midiHandle->pendingNumData = 2;
				midiHandle->pendingRxType = MidiData;
			}

			// Sysex
			else if(numBytes == -1)
			{
				// Receive data in a buffer until end of sysex byte is sent
				midiHandle->pendingRxType = MidiSysEx;
 				midiHandle->pendingNumData = 1;
				if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 1) != HAL_OK)
				{
					return;
				}
			}
		}
	}

	// If device has received a status byte and is waiting on the message data
	else if(midiHandle->pendingRxType == MidiData)
	{
		// Check if a valid data byte was actually received
		if(((midiHandle->rxRawBuf[0] >> 7) & 1))
		{
			if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 1) != HAL_OK)
			{
				return;
			}
			midiHandle->pendingRxType = MidiStatus;
		}
		else
		{
			midi_ringBufferPut(&midiHandle->rxBuf, midiHandle->rxRawBuf[0]);
			// Check if a second data byte is required to be copied into the rxBuf
			if(midiHandle->pendingNumData == 2)
			{
				if(((midiHandle->rxRawBuf[1] >> 7) & 1))
				{
					if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 1) != HAL_OK)
					{
						return;
					}
					midiHandle->pendingRxType = MidiStatus;
				}
				else
				{
					midi_ringBufferPut(&midiHandle->rxBuf, midiHandle->rxRawBuf[1]);
				}
			}
			midiHandle->pendingNumData = 1;
			midiHandle->newMessage = TRUE;
			midiHandle->pendingRxType = MidiStatus;

			// Once data has been received, begin listening for a new status message
			if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 1) != HAL_OK)
			{
				return;
			}
		}
	}

	// Handle incoming SysEx data
	else if(midiHandle->pendingRxType == MidiSysEx)
	{
		// Copy the latest received byte into the rxBuf, and check for a terminating message
		midi_ringBufferPut(&midiHandle->rxBuf, midiHandle->rxRawBuf[0]);
		if(midiHandle->rxRawBuf[0] == SystemExclusiveEnd)
		{
			midiHandle->pendingNumData = 1;
			midiHandle->newMessage = TRUE;
			midiHandle->pendingRxType = MidiStatus;
		}
		// Once data has been received, begin listening for a new status message
		if(HAL_UART_Receive_IT(midiHandle->uartHandle, midiHandle->rxRawBuf, 1) != HAL_OK)
		{
			return;
		}
	}
	return ;
}

void midi_errorUartHandler(MidiInterface* midiHandle)
{
	// Once the error has been handled, restart the MIDI interface
	midi_begin(midiHandle);
	return;
}


// ------------- Callback Assignment ------------ //
void midi_setHandleNoteOn(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  note, uint8_t  velocity))
{
	midiHandle->mNoteOnCallback = fptr;
}

void midi_setHandleNoteOff(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  note, uint8_t  velocity))
{
	midiHandle->mNoteOffCallback = fptr;
}

void midi_setHandleAfterTouchPoly(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  note, uint8_t  pressure))
{

}

void midi_setHandleControlChange(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  number, uint8_t  value))
{
	midiHandle->mControlChangeCallback = fptr;
}

void midi_setHandleSystemExclusive(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint16_t size))
{
	midiHandle->mSystemExclusiveCallback = fptr;
}

void midi_setHandleProgramChange(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  number))
{
	midiHandle->mProgramChangeCallback = fptr;
}

void midi_setHandleAfterTouchchannel(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t  Channel, uint8_t  pressure))
{
	midiHandle->mAfterTouchchannelCallback = fptr;
}

void midi_setHandlePitchBend(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t  Channel, int bend))
{
	midiHandle->mPitchBendCallback = fptr;
}

void midi_setHandleTimeCodeQuarterFrame(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t  data))
{
	midiHandle->mTimeCodeQuarterFrameCallback = fptr;
}

void midi_setHandleSongPosition(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint16_t beats))
{
	midiHandle->mSongPositionCallback = fptr;
}

void midi_setHandleSongSelect(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t  songnumber))
{
	midiHandle->mSongSelectCallback = fptr;
}

void midi_setHandleTuneRequest(MidiInterface* midiHandle, void (*fptr)(void* midiHandle))
{
	midiHandle->mTuneRequestCallback = fptr;
}

void midi_setHandleClock(MidiInterface* midiHandle, void (*fptr)(void* midiHandle))
{
	midiHandle->mClockCallback = fptr;
}

void midi_setHandleStart(MidiInterface* midiHandle, void (*fptr)(void* midiHandle))
{
	midiHandle->mStartCallback = fptr;
}

void midi_setHandleContinue(MidiInterface* midiHandle, void (*fptr)(void* midiHandle))
{
	midiHandle->mContinueCallback = fptr;
}

void midi_setHandleStop(MidiInterface* midiHandle, void (*fptr)(void* midiHandle))
{
	midiHandle->mStopCallback = fptr;
}

void midi_setHandleActiveSensing(MidiInterface* midiHandle, void (*fptr)(void* midiHandle))
{
	midiHandle->mActiveSensingCallback = fptr;
}

void midi_setHandleSystemReset(MidiInterface* midiHandle, void (*fptr)(void* midiHandle))
{
	midiHandle->mSystemResetCallback = fptr;
}


// ------------ Private Functions ------------ //
void midi_Send(	MidiInterface *midiHandle, MidiDataType type,
									MidiDataByte data1, MidiDataByte data2, MidiChannel channel)
{
	// Check for valid channel and data type
	if (channel >= MIDI_CHANNEL_OFF  || type < 0x80)
	{
		return;
	}

	// Protection: remove MSBs on data
	data1 &= 0x7f;
	data2 &= 0x7f;

	// Merge the message type and channel into a single status byte
	MidiStatusByte status;
	if(channel == 0)
	{
		status = type;
	}
	else
	{
		status = midi_getStatus(type, channel);
	}

	// Find out how many date bytes need to be transfered for the given message type
	uint8_t numDataBytes =  midi_numDataBytesForMessage(status);
	// If a transmission is already in process, queue the data up in the txBuf
	if(midiHandle->txState == MidiBusy)
	{
		if(midiHandle->txQueueIndex == MIDI_TX_QUEUE_SIZE - 1)
		{
			return;
		}
		// The status byte is always required for every message
		midiHandle->txQueue[midiHandle->txQueueIndex] = status;
		midiHandle->txQueueIndex++;
		if(numDataBytes == 1)
		{
			midiHandle->txQueue[midiHandle->txQueueIndex] = data1;
			midiHandle->txQueueIndex++;
		}
		if (numDataBytes == 2)
		{
			midiHandle->txQueue[midiHandle->txQueueIndex] = data1;
			midiHandle->txQueue[midiHandle->txQueueIndex] = data2;
			midiHandle->txQueueIndex += 2;
		}
		midiHandle->txDataPending = TRUE;
	}

	// If no transmissions are currently taking place, load the data directly into the transmit buffer
	else
	{
		// Load data into the buffer
		// The status byte is always required for every message
		midiHandle->txBuf[0] = status;
		if(numDataBytes == 1)
		{
			midiHandle->txBuf[1] = data1;
		}
		else if (numDataBytes == 2)
		{
			midiHandle->txBuf[1] = data1;
			midiHandle->txBuf[2] = data2;
		}
		if(midiHandle->deviceType == UartMidi)
		{
			// Lock the buffer transfer state
			midiHandle->txState = MidiBusy;
			HAL_UART_Transmit_IT(midiHandle->uartHandle, midiHandle->txBuf, numDataBytes + 1);

		}
#ifdef USE_USB_MIDI
		else if(midiHandle->deviceType == UsbMidi)
		{
				tud_midi_stream_write(0, midiHandle->txBuf, numDataBytes + 1);
		}
#endif
	}

	//TODO: check handling for sysex
	// other messages
	return;
}

void midi_sendSysEx(	MidiInterface *midiHandle, uint8_t* data, uint16_t len, uint32_t sysExId)
{
	// If a transmission is already in process, queue the data up in the txBuf
	if(midiHandle->txState == MidiBusy)
	{
		if((MIDI_TX_QUEUE_SIZE - 1) - midiHandle->txQueueIndex > len)
		{
			return;
		}
		// Populate the SysEx structure for the message
		midiHandle->txQueue[midiHandle->txQueueIndex] = SystemExclusive;
		midiHandle->txQueueIndex++;
		midiHandle->txQueue[midiHandle->txQueueIndex] = (sysExId >> 16) & 0xff;
		midiHandle->txQueueIndex++;
		midiHandle->txQueue[midiHandle->txQueueIndex] = (sysExId >> 8) & 0xff;
		midiHandle->txQueueIndex++;
		midiHandle->txQueue[midiHandle->txQueueIndex] = sysExId & 0xff;
		midiHandle->txQueueIndex++;

		// Populate the message data
		for(int i=0; i<len; i++)
		{
			midiHandle->txQueue[midiHandle->txQueueIndex] = data[i];
			midiHandle->txQueueIndex++;
		}
		midiHandle->txQueue[midiHandle->txQueueIndex] = SystemExclusiveEnd;
		midiHandle->txQueueIndex++;
		midiHandle->txDataPending = TRUE;
	}


	// If no transmissions are currently taking place, load the data directly into the transmit buffer
	else
	{
		// Populate the SysEx structure for the message
		midiHandle->txBuf[0] = SystemExclusive;
		midiHandle->txBuf[1] = (sysExId >> 16) & 0xff;
		midiHandle->txBuf[2] = (sysExId >> 8) & 0xff;
		midiHandle->txBuf[3] = sysExId & 0xff;

		// Populate the message data
		for(int i=0; i<len; i++)
		{
			midiHandle->txBuf[i+4] = data[i];
		}
		midiHandle->txBuf[len+4] = SystemExclusiveEnd;
		if(midiHandle->deviceType == UartMidi)
		{
			// Lock the buffer transfer state
			midiHandle->txState = MidiBusy;
			if(HAL_UART_Transmit_IT(midiHandle->uartHandle, midiHandle->txBuf, len+5) != HAL_OK)
			{
				return;
			}
			return;
		}
#ifdef USE_USB_MIDI
		if(midiHandle->deviceType == UsbMidi)
		{
			// todo if(CDC_CheckTxReady())
			{
				//tud_midi_stream_write(0, midiHandle->txBuf, 3);
				return;
			}
		}
#endif
	}

	//TODO: check handling for sysex
	// other messages
	return;
}

/**
  * @brief 	Checks how many data bytes comprise of a message with a given status
  * @param 	Status code of the message
  * @retval Number of data bytes. -1 for sysex status, and -2 for invalid status
  */
int midi_numDataBytesForMessage(MidiStatusByte status)
{
	// Channel voice messages
	if((status & 0xf0) != 0xf0)
	{
		// Mask the upper nibble which stores the channel voice type
		MidiStatusByte chanStatus = status & 0xf0;
		switch(chanStatus)
		{
		case ProgramChange:
			return 1;
		case AfterTouchChannel:
			return 1;
		case 0:
			// Indicates an invalid MIDI message
			return -2;
		default:
			return 2;
		}
	}

	// System common & system real-time messages
	else
	{
		switch(status)
		{
		case TimeCodeQuarterFrame:
			return 1;
		case SongPosition:
			return 2;
		case SongSelect:
			return 1;
		case SystemExclusive:
			return -1;
		default:
			return 0;
		}
	}
}

void midi_sendPacket(MidiInterface* midiHandle)
{
	// Because this may be called even though there is no new queued data, check if new data is available
	if(midiHandle->txQueueIndex == 0 || midiHandle == NULL)
	{
		return;
	}

	// Check if a transmission is already in progress.
	// If it is, no action needs to be taken as the tx complete call back will start another transfer.
	if(midiHandle->txState == MidiBusy)
	{
		return;
	}

	// Set the state to busy to avoid interrupts
	midiHandle->txState = MidiBusy;
	// Find out how many bytes exist in the txBuf
	uint8_t numBytes = midiHandle->txQueueIndex;
	// If no transfer is taking place, copy a packet into the txPacketBuf, and begin the transfer
	memcpy(midiHandle->txBuf, midiHandle->txQueue, numBytes);
	// Reset the queue index
	midiHandle->txQueueIndex = 0;


	if(midiHandle->deviceType == UartMidi)
	{
		if(HAL_UART_Transmit_IT(midiHandle->uartHandle, midiHandle->txBuf, numBytes) != HAL_OK)
		{
			return;
		}
	}
#ifdef USE_USB_MIDI
	else if(midiHandle->deviceType == UsbMidi)
	{
		{
			tud_midi_stream_write(0, midiHandle->txBuf, numBytes);
		}
	}
#endif

	// Reset the buffer index to indicate that they are empty
	midiHandle->txQueueIndex = 0;
	midiHandle->txBufIndex = 0;
	return;
}

MidiDataType midi_getStatusType(uint8_t status)
{
	// Check if the status is a channel type message
	if((status & 0xf0) != 0xf0)
	{
		return status & 0xf0;
	}
	else
	{
		return status;
	}
}

/**
  * @brief System Clock Configuration
  * @param none
  * @retval None
  */
MidiStatusByte midi_getStatus(MidiDataType inType, MidiChannel inChannel)
{
	return (uint8_t)inType | ((inChannel) & 0x0f);
}

/*! \brief Encode System Exclusive messages.
 SysEx messages are encoded to guarantee transmission of data bytes higher than
 127 without breaking the MIDI protocol. Use this static method to convert the
 data you want to send.
 \param inData The data to encode.
 \param outSysEx The output buffer where to store the encoded message.
 \param inLength The length of the input buffer.
 \param inFlipHeaderBits True for Korg and other who store MSB in reverse order
 \return The length of the encoded output buffer.
 @see decodeSysEx
 Code inspired from Ruin & Wesen's SysEx encoder/decoder - http://ruinwesen.com
 */
unsigned encodeSysEx(const uint8_t* inData, uint8_t* outSysEx,
                     unsigned inLength, uint8_t inFlipHeaderBits)
{
	unsigned outLength  = 0;     // Num bytes in output array.
	uint8_t count          = 0;     // Num 7bytes in a block.
	outSysEx[0]         = 0;

	for (unsigned i = 0; i < inLength; ++i)
	{
		const uint8_t data = inData[i];
		const uint8_t msb  = data >> 7;
		const uint8_t body = data & 0x7f;

		outSysEx[0] |= (msb << (inFlipHeaderBits ? count : (6 - count)));
		outSysEx[1 + count] = body;

		if (count++ == 6)
		{
			outSysEx   += 8;
			outLength  += 8;
			outSysEx[0] = 0;
			count       = 0;
		}
	}
	return outLength + count + (count != 0 ? 1 : 0);
}

/*! \brief Decode System Exclusive messages.
 SysEx messages are encoded to guarantee transmission of data bytes higher than
 127 without breaking the MIDI protocol. Use this static method to reassemble
 your received message.
 \param inSysEx The SysEx data received from MIDI in.
 \param outData The output buffer where to store the decrypted message.
 \param inLength The length of the input buffer.
 \param inFlipHeaderBits True for Korg and other who store MSB in reverse order
 \return The length of the output buffer.
 @see encodeSysEx @see getSysExArrayLength
 Code inspired from Ruin & Wesen's SysEx encoder/decoder - http://ruinwesen.com
 */
unsigned decodeSysEx(const uint8_t* inSysEx, uint8_t* outData,
                     unsigned inLength, uint8_t inFlipHeaderBits)
{
	unsigned count  = 0;
	uint8_t msbStorage = 0;
	uint8_t byteIndex  = 0;

	for (unsigned i = 0; i < inLength; ++i)
	{
		if ((i % 8) == 0)
		{
			msbStorage = inSysEx[i];
			byteIndex  = 6;
		}
		else
		{
			const uint8_t body     = inSysEx[i];
			const uint8_t shift    = inFlipHeaderBits ? 6 - byteIndex : byteIndex;
			const uint8_t msb      = byte(((msbStorage >> shift) & 1) << 7);
			byteIndex--;
			outData[count++] = msb | body;
		}
	}
	return count;
}
