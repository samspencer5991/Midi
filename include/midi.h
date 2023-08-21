/*
 *
 *  Created on: 15 Dec 2019
 *      Author: samspencer
 */

#ifndef MIDI_H_
#define MIDI_H_

#include "usart.h"
#include "stdint.h"

#ifdef USE_MIDI_TRIGGERS
//#include "midi_trigger.h"
#endif

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

#define PITCH_BEND_UPPER_BYTE_MAX		0x7f
#define PITCH_BEND_MAX							0x3fff

#define MIN_CLOCK_BPM								45
#define DEFAULT_CLOCK_BPM						120
#define MAX_CLOCK_BPM 							240
#define MIDI_CLOCK_TIMER_OFFSET 		1.003		// Scaling factor to account for processing delays
#define NUM_TAP_INTERVALS						3				// Number of recorded times between tap tempo inputs
#define LED_ACTIVE_CLOCK_MESSAGES		4
#define MIDI_CLOCK_TAP_TIMEOUT			1500
#define MIDI_CLOCK_NO_ASSIGN_INDEX	-1
#define MIDI_INTERFACE_INDEX_SIZE		8
#define MIDI_CLOCK_NO_SWITCH_INDEX 	-1

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

// MIDI data types
typedef enum
{
	InvalidType   					= 0x00,	// For notifying errors

	// Custom
	MidiCustomMessage				= 0x70,	// Generic custom message
	// Standard MIDI
	NoteOff               	= 0x80, // Note Off
	NoteOn                	= 0x90, // Note On
	AfterTouchPoly        	= 0xA0, // Polyphonic AfterTouch
	ControlChange         	= 0xB0, // Control Change
	ProgramChange         	= 0xC0, // Program Change
	AfterTouchChannel     	= 0xD0, // channel (monophonic) AfterTouch
	PitchBend             	= 0xE0, // Pitch Bend
	SystemExclusive       	= 0xF0, // System Exclusive
	TimeCodeQuarterFrame  	= 0xF1, // System Common - MIDI Time Code Quarter Frame
	SongPosition          	= 0xF2, // System Common - Song Position Pointer
	SongSelect            	= 0xF3, // System Common - Song Select
	Reserved1								= 0xF4,	// Undefined (reserved)
	Reserved2								= 0xF5,	// Undefined (reserved)
	TuneRequest           	= 0xF6, // System Common - Tune Request
	SystemExclusiveEnd			= 0xF7,	// System Exclusive terminating byte
	Clock                 	= 0xF8, // System Real Time - Timing Clock
	Start                 	= 0xFA, // System Real Time - Start
	Continue              	= 0xFB, // System Real Time - Continue
	Stop                  	= 0xFC, // System Real Time - Stop
	Undefined								= 0xFD,	// Undefined (reserved)
	ActiveSensing         	= 0xFE, // System Real Time - Active Sensing
	SystemReset           	= 0xFF // System Real Time - System Reset
} MidiDataType;

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


	// TRS switching
	CustomTrsSwitchOut,
	CustomVoltageOut,

	// UI
	CustomSetUIMode
} MidiSmartMessage;

typedef enum
{
	CinReserved								= 0x00,
	CinCableEvent							= 0x01,
	CinSystemCommonTwoByte		= 0x02,
	CinSystemCommonThreeByte	= 0x03,
	CinSysExStartContinue			= 0x04,
	CinSysExSingleByte				= 0x05,
	CinSysExTwoByte						= 0x06,
	CinSysExThreeByte					= 0x07,
	CinNoteOff								= 0x08,
	CinNoteOn									= 0x09,
	CinAfterTouchPoly        	= 0x0A,
	CinControlChange         	= 0x0B,
	CinProgramChange         	= 0x0C,
	CinAfterTouchChannel     	= 0x0D,
	CinPitchBend             	= 0x0E,
	CinSingleByte							= 0x0F
} CodeIndexNumber;

/* Brief Enumeration of Control Change command numbers.
 See the detailed controllers numbers & description here:
 http://www.somascape.org/midi/tech/spec.html#ctrlnums
 */
typedef enum
{
    // High resolution Continuous Controllers MSB (+32 for LSB) ----------------
    BankSelect            				= 0,
    ModulationWheel             	= 1,
    BreathController            	= 2,
    // CC3 undefined
    FootController             		= 4,
    PortamentoTime              	= 5,
    DataEntry                   	= 6,
    cCannelVolume               	= 7,
    Balance                     	= 8,
    // CC9 undefined
    Pan                         	= 10,
    ExpressionController        	= 11,
    EffectControl1              	= 12,
    EffectControl2              	= 13,
    // CC14 undefined
    // CC15 undefined
    GeneralPurposeController1   	= 16,
    GeneralPurposeController2   	= 17,
    GeneralPurposeController3   	= 18,
    GeneralPurposeController4   	= 19,

    // Switches ----------------------------------------------------------------
    Sustain                     	= 64,
    Portamento                  	= 65,
    Sostenuto                   	= 66,
    SoftPedal                   	= 67,
    Legato                      	= 68,
    Hold                        	= 69,

    // Low resolution continuous controllers -----------------------------------
    SoundController1            	= 70,   // Synth: Sound Variation   FX: Exciter On/Off
    SoundController2            	= 71,   // Synth: Harmonic Content  FX: Compressor On/Off
    SoundController3            	= 72,   // Synth: Release Time      FX: Distortion On/Off
    SoundController4            	= 73,   // Synth: Attack Time       FX: EQ On/Off
    SoundController5            	= 74,   // Synth: Brightness        FX: Expander On/Off
    SoundController6            	= 75,   // Synth: Decay Time        FX: Reverb On/Off
    SoundController7            	= 76,   // Synth: Vibrato Rate      FX: Delay On/Off
    SoundController8            	= 77,   // Synth: Vibrato Depth     FX: Pitch Transpose On/Off
    SoundController9            	= 78,   // Synth: Vibrato Delay     FX: Flange/Chorus On/Off
    SoundController10           	= 79,   // Synth: Undefined         FX: Special Effects On/Off
    GeneralPurposeController5   	= 80,
    GeneralPurposeController6   	= 81,
    GeneralPurposeController7   	= 82,
    GeneralPurposeController8   	= 83,
    PortamentoControl           	= 84,
    // CC85 to CC90 undefined
    Effects1                    	= 91,   // Reverb send level
    Effects2                    	= 92,   // Tremolo depth
    Effects3                    	= 93,   // Chorus send level
    Effects4                    	= 94,   // Celeste depth
    Effects5                    	= 95,   // Phaser depth

    // channel Mode messages ---------------------------------------------------
    AllSoundOff                 	= 120,
    ResetAllControllers         	= 121,
    LocalControl                	= 122,
    AllNotesOff                 	= 123,
    OmniModeOff                 	= 124,
    OmniModeOn                  	= 125,
    MonoModeOn                  	= 126,
    PolyModeOn                  	= 127
} MidiControlChangeNumber;

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

/* Ring Buffer */
#ifndef USE_UART_FIFO
typedef struct
{
	uint8_t *buffer;	// Ring buffer for TX uart data
	uint16_t head;		// Index for data head position
	uint16_t tail;		// Index for data tail position
	uint8_t full;		// Flag for whether buffer is full
	uint16_t size;		// Capacity of the buffer in bytes
} MidiRingBuf;
#endif

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
#ifndef USE_UART_FIFO
	/* Because some microcontrollers may not have a FIFO buffer for the UART port (STM32F072 for example),
	 * ring buffers can be implemented by commenting out USE_UART_FIFO at the top of this file.
	 *
	 * There are two buffers for each direction of the transport.
	 * The rxRawBuf is quite small, and only stores enough bytes to receive 1 complete message.
	 * That data is then copied into the rxBuf once a complete message has been received.
	 *
	 * For transmission, the txBuf is used to store all messages the application wishes to send.
	 * These are then copied into the txPacket array buffer to be transmitted.
	 * This mechanism allows for new messages to be queued ready for transmission without interrupting the existing transmission.
	 *
	 * The maximum transmission packet size is determined by TX_MAX_QUEUE.
	 * Depending on the number of bytes to transmit, DMA (larger) or Interrupt (smaller) will be used.
	 * Once the transmission is complete, if there are more messages in the txBuf,
	 * these are loaded into the txPacket buffer, and then transmitted as well, ensuring a constant flow of data.
	 */
	uint8_t rxRawBuf[2];						// Buffer to hold raw data from the UART port
	MidiRingBuf rxBuf;						// Incoming data buffer
	uint8_t txQueue[MIDI_TX_QUEUE_SIZE];	// Array buffer for midi data waiting to be transfered
	uint16_t txQueueIndex;					// Stores the current buffer element, and also the number of elements
	uint8_t txBuf[MIDI_TX_BUF_SIZE];		// Packet buffer for the data to transmit
	uint16_t txBufIndex;						// Store the current buffer element, and also the number of elements
	uint8_t txDataPending;					// Indicates whether there is new data waiting in the txBuf
	MidiState txState;						// Current state of the tx transport interface (ie. transmitting or ready)
	MidiPendingRxType pendingRxType;		// What type of midi data is pending reception
	uint8_t pendingNumData;					// Number of bytes receiver is waiting for
	uint8_t newMessage;						// Flag to determine if a complete MIDI message has been received
#endif
	/* Callback handles */
	void (*mNoteOffCallback)(void* midiHandle, uint8_t channel, uint8_t note, uint8_t velocity);
	void (*mNoteOnCallback)(void* midiHandle, uint8_t channel, uint8_t note, uint8_t velocity);
	void (*mAfterTouchPolyCallback)(void* midiHandle, uint8_t channel, uint8_t note, uint8_t  elocity);
	void (*mControlChangeCallback)(void* midiHandle, uint8_t channel, uint8_t number, uint8_t value);
	void (*mProgramChangeCallback)(void* midiHandle, uint8_t channel, uint8_t number);
	void (*mAfterTouchchannelCallback)(void* midiHandle, uint8_t channel, uint8_t value);
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

typedef struct
{
	MidiClockState state;
	MidiInterface* midiHandles[NUM_MIDI_INTERFACES];
	MidiClockSubDivision subDivision;
	uint16_t bpm;
#ifdef USE_MIDI_CLOCK
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
MidiErrorState midi_sendNoteOn(MidiInterface* midiHandle, MidiDataByte  inNoteNumber, MidiDataByte  inVelocity, MidiChannel inChannel);
MidiErrorState midi_sendNoteOff(MidiInterface* midiHandle, MidiDataByte  inNoteNumber, MidiDataByte  inVelocity, MidiChannel inChannel);
MidiErrorState midi_sendProgramChange(MidiInterface* midiHandle, MidiDataByte  inProgramNumber, MidiChannel inChannel);
MidiErrorState midi_sendControlChange(MidiInterface* midiHandle, MidiDataByte  inControlNumber, MidiDataByte  inControlValue, MidiChannel inChannel);
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

MidiErrorState midi_send(	MidiInterface* midiHandle, MidiDataType type, MidiDataByte data1,
													MidiDataByte data2, MidiChannel channel);
MidiErrorState midi_sendSysEx(	MidiInterface *midiHandle, uint8_t* data, uint16_t len, uint32_t sysExId);
MidiErrorState midi_sendPacket(MidiInterface* midiHandle);


MidiErrorState midi_read(MidiInterface *midiHandle);
MidiErrorState midi_readCustom(MidiInterface *midiHandle, uint8_t* buf, uint16_t* numCopiedBytes);
MidiErrorState midi_rxUartHandler(MidiInterface* midiHandle);
MidiErrorState midi_errorUartHandler(MidiInterface* midiHandle);
MidiErrorState midi_txUartHandler(MidiInterface* midiHandle);

/* Utility */
MidiStatusByte midi_getStatus(MidiDataType inType, MidiChannel inChannel);
MidiErrorState midi_ringBufferPut(MidiRingBuf* buffer, uint8_t data);
MidiErrorState midi_ringBufferGet(MidiRingBuf* buffer, uint8_t* data);
int midi_numDataBytesForMessage(MidiStatusByte status);
MidiDataType midi_getStatusType(uint8_t status);
void midi_convertNoteNumberToText(uint8_t number, char* str);

/* Clock */
#ifdef USE_MIDI_CLOCK
MidiErrorState midi_clockAssignHandle(MidiClockTx* midiClock, MidiInterface** midiHandles, uint8_t numHandles);
MidiErrorState midi_clockSetTempo(MidiClockTx* midiClock, uint16_t newTempo);
void midi_clockInit(MidiClockTx* midiClock, TIM_HandleTypeDef* htim);
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

void midi_setHandleAfterTouchchannel(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle, uint8_t  Channel, uint8_t  pressure));
void midi_setHandlePitchBend(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle, uint8_t  Channel, int bend));
void midi_setHandleTimeCodeQuarterFrame(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle, uint8_t  data));
void midi_setHandleSongPosition(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle, unsigned beats));
void midi_setHandleSongSelect(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle, uint8_t  songnumber));
void midi_setHandleTuneRequest(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle));
void midi_setHandleClock(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle));
void midi_setHandleStart(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle));
void midi_setHandleContinue(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle));
void midi_setHandleStop(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle));
void midi_setHandleActiveSensing(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle));
void midi_setHandleSystemReset(MidiInterface* midiHandle, void (*fptr)(midiInterface_t* midiHandle));

void midi_disconnectCallbackFromType(MidiInterface* midiHandle, MidiDataType inType);

MidiErrorState midi_sendSongPosition(unsigned inBeats);
MidiErrorState midi_sendSongSelect(MidiDataByte  inSongNumber);
MidiErrorState midi_sendTuneRequest();
MidiErrorState midi_sendRealTime(MidiDataType inType);

	//void launchCallback();

#endif /* MIDI_H_ */
