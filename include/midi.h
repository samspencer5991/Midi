/*
 *
 *  Created on: 15 Dec 2019
 *      Author: samspencer
 */

#ifndef MIDI_H_
#define MIDI_H_

#if FRAMEWORK_STM32CUBE
#if defined(STM32G4xx)
#include "stm32g4xx_hal.h"
#endif
#endif
#include "stdint.h"
#include "midi_buffer.h"
#include "midi_defs.h"

#define USE_USB_MIDI
#define USE_MIDI_CLOCK

#define MIDI_CHANNEL_OMNI     0
#define MIDI_CHANNEL_OFF     	17 // and over
#define NULL_BYTE 0

#ifndef MIDI_TX_QUEUE_SIZE
#define MIDI_TX_QUEUE_SIZE 1024
#endif

#ifndef MIDI_TX_BUF_SIZE
#define MIDI_TX_BUF_SIZE 1024
#endif

#ifndef MAX_MIDI_DELAY_MS
#define MAX_MIDI_DELAY_MS	255
#endif

#define PITCH_BEND_UPPER_BYTE_MAX	0x7f
#define PITCH_BEND_MAX					0x3fff

#define MIN_CLOCK_BPM					45
#define DEFAULT_CLOCK_BPM				120
#define MAX_CLOCK_BPM 					240
#define MIDI_CLOCK_TIMER_OFFSET 		1.003		// Scaling factor to account for processing delays
#define NUM_TAP_INTERVALS				3			// Number of recorded times between tap tempo inputs
#define LED_ACTIVE_CLOCK_MESSAGES	4
#define MIDI_CLOCK_TAP_TIMEOUT		1500
#define MIDI_CLOCK_NO_ASSIGN_INDEX	-1
#define MIDI_INTERFACE_INDEX_SIZE	8
#define MIDI_CLOCK_NO_SWITCH_INDEX 	-1

#define MIDI_CLOCK_LED_NONE			0
#define MIDI_CLOCK_LED_OFF				1
#define MIDI_CLOCK_LED_ON				2

#define MIDI_TYPE_A	0
#define MIDI_TYPE_B	1
#define MIDI_TIP		2
#define MIDI_RING		3

// Type Definitions
typedef uint8_t MidiStatusByte;
typedef uint8_t MidiDataByte;
typedef uint8_t MidiChannel;
typedef uint8_t MidiFilterMode;

typedef enum
{
	MidiHalError,
	MidiParamError,
	MidiMemError,
	MidiBufferFull,
	MidiBufferEmpty,
	MidiNoData,
	MidiTxBusy,
	MidiMessagesFull,
	MidiNoFreeLfo,
	MidiLfoNotAssigned,
	MidiOk
} MidiErrorState;

// MIDI device tyoe
typedef enum
{
	UsbMidi,
	UartMidi
} MidiDeviceType;


typedef enum
{
	// Switch behaviour
	CustomTurnOnSwitch,
	CustomTurnOffSwitch,
	CustomToggleSwitch,

	CustomResetSequentialStep,
	CustomIncrementSequentialStep,
	CustomDecrementSequentialStep,
	CustomGoToSequentialStep,
	CustomQueueNextSequentialStep,
	CustomQueueSequentialStep,

	CustomResetScrollingStep,
	CustomIncrementScrollingStep,
	CustomDecrementScrollingStep,
	CustomGoToScrollingStep,
	CustomQueueNextScrollingStep,
	CustomQueueScrollingStep,

	// Bank navigation
	CustomBankUp,
	CustomBankDown,
	CustomGoToBank,

	// Expression pedal steps
	CustomIncrementExpStep,
	CustomDecrementExpStep,
	CustomGoToExpStep,

	// Set list mode
	CustomToggleSetListMode,
	CustomToggleSetListBrowser,


	// TRS switching
	CustomTrsSwitchOut,
	CustomVoltageOut,

	// UI
	CustomSetUIMode
} MidiSmartMessage;

typedef enum
{
	CinReserved						= 0x00,
	CinCableEvent					= 0x01,
	CinSystemCommonTwoByte		= 0x02,
	CinSystemCommonThreeByte	= 0x03,
	CinSysExStartContinue		= 0x04,
	CinSysExSingleByte			= 0x05,
	CinSysExTwoByte				= 0x06,
	CinSysExThreeByte				= 0x07,
	CinNoteOff						= 0x08,
	CinNoteOn						= 0x09,
	CinAfterTouchPoly        	= 0x0A,
	CinControlChange         	= 0x0B,
	CinProgramChange         	= 0x0C,
	CinAfterTouchChannel     	= 0x0D,
	CinPitchBend             	= 0x0E,
	CinSingleByte					= 0x0F
} CodeIndexNumber;

typedef enum
{
	MidiStatus,
	MidiSysEx,
	MidiData
} MidiPendingRxType;

typedef enum
{
	MidiClock4 			= 1,
	MidiClock4T			= 2,
	MidiClock4Dot		= 3,
	MidiClock8 			= 4,
	MidiClock8T 		= 5,
	MidiClock8Dot 	= 6,
	MidiClock16			= 7,
	MidiClock16T		= 8,
	MidiClock16Dot	= 9
} MidiClockSubDivision;

// Simple type for tracking the state of a direction of the midi transport
typedef enum
{
	MidiBusy,
	MidiTxInProgress,
	MidiReady
} MidiState;

typedef enum
{
	MidiInOnly,
	MidiOutOnly,
	MidiFull
} MidiInterfaceDirection;

typedef enum
{
	MidiClockRunning,
	MidiClockStopped
} MidiClockState;

typedef struct
{
	MidiChannel channel;
	MidiDataType type;
	MidiDataByte data1;
	MidiDataByte data2;
	uint8_t valid;
	uint8_t length;
} MidiMessageInfo;

// MIDI Interface structure
//typedef struct MidiInterface;

typedef struct MidiInterface
{
	/*	HARDWARE	*/
	// Check to see if the hardware is configured for the physical type
	#ifdef HAL_UART_MODULE_ENABLED
	UART_HandleTypeDef* uartHandle;
	IRQn_Type uartIRQn;
	#endif

	MidiDeviceType deviceType;	// Stores the hardware protocol for midi communication (USB, UART etc...)
	MidiChannel channel;				// Active receiving channel for the interface
	uint8_t active;							// State of the interface. If inactive, it will not accept new messages in it's buffers
	MidiInterfaceDirection direction;
	struct MidiInterface** thruHandles;
	uint8_t numThruHandles;
	uint8_t thruBuffer[2];
	/* Data Buffers */

	uint8_t rxRawBuf[2];						// Buffer to hold raw data from the UART port
	MidiRingBuf rxBuf;						// Incoming data buffer
	uint8_t txQueue[MIDI_TX_QUEUE_SIZE];	// Array buffer for midi data waiting to be transfered
	uint16_t txQueueIndex;					// Stores the current buffer element, and also the number of elements
	uint8_t txBuf[MIDI_TX_BUF_SIZE];		// Packet buffer for the data to transmit
	uint16_t txBufIndex;						// Store the current buffer element, and also the number of elements
	uint8_t txDataPending;					// Indicates whether there is new data waiting in the txBuf
	MidiState txState;						// Current state of the tx transport interface (ie. transmitting or ready)
	uint8_t pendingNumData;					// Number of bytes receiver is waiting for
	uint8_t pendingMessageIndex;			// Index of the current byte in the pending message
	uint8_t pendingMessage[3];				// Buffer for the pending message
	MidiMessageInfo message;
	uint8_t newMessage;						// Flag to determine if a complete MIDI message has been received
	uint32_t lastMessageTime;				// Time of last message received
	

	/* Callback handles */
	void (*mNoteOffCallback)(void* midiHandle, uint8_t channel, uint8_t note, uint8_t velocity);
	void (*mNoteOnCallback)(void* midiHandle, uint8_t channel, uint8_t note, uint8_t velocity);
	void (*mAfterTouchPolyCallback)(void* midiHandle, uint8_t channel, uint8_t note, uint8_t  elocity);
	void (*mControlChangeCallback)(void* midiHandle, uint8_t channel, uint8_t number, uint8_t value);
	void (*mProgramChangeCallback)(void* midiHandle, uint8_t channel, uint8_t number);
	void (*mAfterTouchChannelCallback)(void* midiHandle, uint8_t channel, uint8_t value);
	void (*mPitchBendCallback)(void* midiHandle, uint8_t channel, int value);
	void (*mSystemExclusiveCallback)(void* midiHandle, uint16_t size);
	void (*mTimeCodeQuarterFrameCallback)(void* midiHandle, uint8_t data);
	void (*mSongPositionCallback)(void* midiHandle, uint16_t beats);
	void (*mSongSelectCallback)(void* midiHandle, uint8_t songnumber);
	void (*mTuneRequestCallback)(void* midiHandle);
	void (*mClockCallback)(void* midiHandle);
	void (*mStartCallback)(void* midiHandle);
	void (*mContinueCallback)(void* midiHandle);
	void (*mStopCallback)(void* midiHandle);
	void (*mActiveSensingCallback)(void* midiHandle);
	void (*mSystemResetCallback)(void* midiHandle);
} MidiInterface;

typedef enum
{
	TapNoQuantise,
	TempoPoint5Quantise,
	Tempo1PointQuantise
} TapTempoQuantisation;

// Non-volatile data for MIDI clock configuration
typedef struct
{
	MidiInterface** midiHandles;
	TapTempoQuantisation quantise;
} MidiClockConfig;

typedef struct
{
	MidiClockState state;
	MidiClockConfig* config;	// Pointer to clock config
	MidiClockSubDivision subDivision;
	uint16_t bpm;
#ifdef FRAMEWORK_STM32CUBE
	TIM_HandleTypeDef* clockTim;
#endif
	uint16_t tapIntervals[NUM_TAP_INTERVALS];
	uint32_t lastTapTime;
	uint8_t tapIntervalIndex;
	uint8_t messageCounter;
	int tempoLed;
	int statusLed;
	int switchIndex;
	uint8_t useLed;
	volatile uint8_t ledIndicatorState;
} MidiClockTx;



extern uint8_t midiInChannel;	// MIDI in channel is set to omni by default
extern uint32_t sysExId;


/* Init and Config */
MidiErrorState midi_init(	MidiInterface* midiHandle, UART_HandleTypeDef* uartHandle, MidiDeviceType type,
															MidiChannel setChannel, MidiInterfaceDirection direction,
															uint8_t* ptrRxBuf, uint16_t size, IRQn_Type irq);
MidiErrorState midi_begin(MidiInterface* midiHandle);
MidiErrorState midi_assignSysExId(uint32_t id);
void midi_assignChannel(uint8_t newChannel);

/* IO */
void midi_SendNoteOn(MidiInterface* midiHandle, MidiDataByte  inNoteNumber, MidiDataByte  inVelocity, MidiChannel inChannel);
void midi_SendNoteOff(MidiInterface* midiHandle, MidiDataByte  inNoteNumber, MidiDataByte  inVelocity, MidiChannel inChannel);
void midi_SendProgramChange(MidiInterface* midiHandle, MidiDataByte  inProgramNumber, MidiChannel inChannel);
void midi_SendControlChange(MidiInterface* midiHandle, MidiDataByte  inControlNumber, MidiDataByte  inControlValue, MidiChannel inChannel);
/* TODO
void sendPitchBend(int inPitchValue,    Channel inchannel);
void sendPitchBend(double inPitchValue, Channel inchannel);

void sendPolyPressure(DataByte  inNoteNumber,
							 DataByte  inPressure,
							 Channel inchannel);

void sendAfterTouch(DataByte  inPressure,
						   Channel inchannel);

void sendSysEx(	unsigned inLength,
					const uint8_t * inArray,
					uint8_t inArrayContainsBoundaries);
void sendTimeCodeQuarterFrame(DataByte  inTypeNibble,
									 DataByte  inValuesNibble);

void sendTimeCodeQuarterFrame(DataByte  inData);
*/

void midi_Send(	MidiInterface* midiHandle, MidiDataType type, MidiDataByte data1,
													MidiDataByte data2, MidiChannel channel);
void midi_SendSysEx(	MidiInterface *midiHandle, uint8_t* data, uint16_t len, uint32_t sysExId);
void midi_sendPacket(MidiInterface* midiHandle);


void midi_Read(MidiInterface *midiHandle);
MidiErrorState midi_readCustom(MidiInterface *midiHandle, uint8_t* buf, uint16_t* numCopiedBytes);
void midi_rxUartHandler(MidiInterface* midiHandle);
void midi_errorUartHandler(MidiInterface* midiHandle);
void midi_txUartHandler(MidiInterface* midiHandle);

/* Utility */
MidiStatusByte midi_getStatus(MidiDataType inType, MidiChannel inChannel);
int midi_numDataBytesForMessage(MidiStatusByte status);
MidiDataType midi_getStatusType(uint8_t status);
void midi_convertNoteNumberToText(uint8_t number, char* str);

/* Clock */
#ifdef USE_MIDI_CLOCK
MidiErrorState midi_clockAssignHandle(MidiClockTx* midiClock, MidiInterface** midiHandles, uint8_t numHandles);
MidiErrorState midi_clockSetTempo(MidiClockTx* midiClock, uint16_t newTempo);
void midi_clockInit(MidiClockTx* midiClock);
void midi_clockStart(MidiClockTx* midiClock);
void midi_clockStop(MidiClockTx* midiClock);
void midi_clockSend(MidiClockTx* midiClock);
void midi_clockSendStop(MidiClockTx* midiClock);
void midi_clockSendStart(MidiClockTx* midiClock);
void midi_clockTapTempoUpdate(MidiClockTx* midiClock);
void midi_turnOffMidiClock(MidiClockTx* midiClock);
#endif


/* INPUT CALLBACK MANAGEMENT */
void midi_setHandleNoteOff(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  note, uint8_t  velocity));
void midi_setHandleNoteOn(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  note, uint8_t  velocity));
void midi_setHandleAfterTouchPoly(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  note, uint8_t  pressure));
void midi_setHandleControlChange(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  number, uint8_t  value));
void midi_setHandleProgramChange(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint8_t  channel, uint8_t  number));
void midi_setHandleSystemExclusive(MidiInterface *midiHandle, void (*fptr)(void* midiHandle, uint16_t size));

void midi_setHandleAfterTouchchannel(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t Channel, uint8_t pressure));
void midi_setHandlePitchBend(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t Channel, int bend));
void midi_setHandleTimeCodeQuarterFrame(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t  data));
void midi_setHandleSongPosition(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint16_t beats));
void midi_setHandleSongSelect(MidiInterface* midiHandle, void (*fptr)(void* midiHandle, uint8_t songnumber));
void midi_setHandleTuneRequest(MidiInterface* midiHandle, void (*fptr)(void* midiHandle));
void midi_setHandleClock(MidiInterface* midiHandle, void (*fptr)(void* midiHandle));
void midi_setHandleStart(MidiInterface* midiHandle, void (*fptr)(void* midiHandle));
void midi_setHandleContinue(MidiInterface* midiHandle, void (*fptr)(void* midiHandle));
void midi_setHandleStop(MidiInterface* midiHandle, void (*fptr)(void* midiHandle));
void midi_setHandleActiveSensing(MidiInterface* midiHandle, void (*fptr)(void* midiHandle));
void midi_setHandleSystemReset(MidiInterface* midiHandle, void (*fptr)(void* midiHandle));

void midi_disconnectCallbackFromType(MidiInterface* midiHandle, MidiDataType inType);

MidiErrorState midi_sendSongPosition(unsigned inBeats);
MidiErrorState midi_sendSongSelect(MidiDataByte  inSongNumber);
MidiErrorState midi_sendTuneRequest();
MidiErrorState midi_sendRealTime(MidiDataType inType);

unsigned encodeSysEx(const uint8_t* inData, uint8_t* outSysEx, unsigned inLength, uint8_t inFlipHeaderBits);
unsigned decodeSysEx(const uint8_t* inSysEx, uint8_t* outData, unsigned inLength, uint8_t inFlipHeaderBits);

#endif /* MIDI_H_ */
