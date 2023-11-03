#ifndef MIDI_DEFS_H
#define MIDI_DEFS_H

#include <stdint.h>

// -----------------------------------------------------------------------------

#define MIDI_CHANNEL_OMNI       0
#define MIDI_CHANNEL_OFF        17 // and over

#define MIDI_PITCHBEND_MIN      -8192
#define MIDI_PITCHBEND_MAX      8191

/*! Receiving Active Sensing 
*/
static const uint16_t ActiveSensingTimeout = 300;

// -----------------------------------------------------------------------------
// Type definitions

typedef uint8_t StatusByte;
typedef uint8_t DataByte;
typedef uint8_t Channel;
typedef uint8_t FilterMode;

// -----------------------------------------------------------------------------
// Errors
static const uint8_t ErrorParse = 0;
static const uint8_t ErrorActiveSensingTimeout = 1;
static const uint8_t WarningSplitSysEx = 2;

// -----------------------------------------------------------------------------

// MIDI data types
typedef enum
{
	InvalidType   				= 0x00,	// For notifying errors

	// Custom
	MidiCustomMessage			= 0x70,	// Generic custom message
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
	Reserved1					= 0xF4,	// Undefined (reserved)
	Reserved2					= 0xF5,	// Undefined (reserved)
	TuneRequest           	= 0xF6, // System Common - Tune Request
	SystemExclusiveEnd		= 0xF7,	// System Exclusive terminating byte
	Clock                 	= 0xF8, // System Real Time - Timing Clock
	Start                 	= 0xFA, // System Real Time - Start
	Continue              	= 0xFB, // System Real Time - Continue
	Stop                  	= 0xFC, // System Real Time - Stop
	Undefined					= 0xFD,	// Undefined (reserved)
	ActiveSensing         	= 0xFE, // System Real Time - Active Sensing
	SystemReset           	= 0xFF // System Real Time - System Reset
} MidiDataType;

// -----------------------------------------------------------------------------

/*! Enumeration of Thru filter modes */

typedef enum 
{
	Off                   = 0,  ///< Thru disabled (nothing passes through).
	Full                  = 1,  ///< Fully enabled Thru (every incoming message is sent back).
	SameChannel           = 2,  ///< Only the messages on the Input Channel will be sent back.
	DifferentChannel      = 3,  ///< All the messages but the ones on the Input Channel will be sent back.
} ThruMode;

// -----------------------------------------------------------------------------

/*! \brief Enumeration of Control Change command numbers.
 See the detailed controllers numbers & description here:
 http://www.somascape.org/midi/tech/spec.html#ctrlnums
 */
typedef enum
{
    // High resolution Continuous Controllers MSB (+32 for LSB) ----------------
    BankSelect                  = 0,
    ModulationWheel             = 1,
    BreathController            = 2,
    // CC3 undefined
    FootController              = 4,
    PortamentoTime              = 5,
    DataEntryMSB                = 6,
    ChannelVolume               = 7,
    Balance                     = 8,
    // CC9 undefined
    Pan                         = 10,
    ExpressionController        = 11,
    EffectControl1              = 12,
    EffectControl2              = 13,
    // CC14 undefined
    // CC15 undefined
    GeneralPurposeController1   = 16,
    GeneralPurposeController2   = 17,
    GeneralPurposeController3   = 18,
    GeneralPurposeController4   = 19,

    DataEntryLSB                = 38,

    // Switches ----------------------------------------------------------------
    Sustain                     = 64,
    Portamento                  = 65,
    Sostenuto                   = 66,
    SoftPedal                   = 67,
    Legato                      = 68,
    Hold                        = 69,

    // Low resolution continuous controllers -----------------------------------
    SoundController1            = 70,   ///< Synth: Sound Variation   FX: Exciter On/Off
    SoundController2            = 71,   ///< Synth: Harmonic Content  FX: Compressor On/Off
    SoundController3            = 72,   ///< Synth: Release Time      FX: Distortion On/Off
    SoundController4            = 73,   ///< Synth: Attack Time       FX: EQ On/Off
    SoundController5            = 74,   ///< Synth: Brightness        FX: Expander On/Off
    SoundController6            = 75,   ///< Synth: Decay Time        FX: Reverb On/Off
    SoundController7            = 76,   ///< Synth: Vibrato Rate      FX: Delay On/Off
    SoundController8            = 77,   ///< Synth: Vibrato Depth     FX: Pitch Transpose On/Off
    SoundController9            = 78,   ///< Synth: Vibrato Delay     FX: Flange/Chorus On/Off
    SoundController10           = 79,   ///< Synth: Undefined         FX: Special Effects On/Off
    GeneralPurposeController5   = 80,
    GeneralPurposeController6   = 81,
    GeneralPurposeController7   = 82,
    GeneralPurposeController8   = 83,
    PortamentoControl           = 84,
    // CC85 to CC90 undefined
    Effects1                    = 91,   ///< Reverb send level
    Effects2                    = 92,   ///< Tremolo depth
    Effects3                    = 93,   ///< Chorus send level
    Effects4                    = 94,   ///< Celeste depth
    Effects5                    = 95,   ///< Phaser depth
    DataIncrement               = 96,
    DataDecrement               = 97,
    NRPNLSB                     = 98,   ///< Non-Registered Parameter Number (LSB)
    NRPNMSB                     = 99,   ///< Non-Registered Parameter Number (MSB)
    RPNLSB                      = 100,  ///< Registered Parameter Number (LSB)
    RPNMSB                      = 101,  ///< Registered Parameter Number (MSB)

    // Channel Mode messages ---------------------------------------------------
    AllSoundOff                 = 120,
    ResetAllControllers         = 121,
    LocalControl                = 122,
    AllNotesOff                 = 123,
    OmniModeOff                 = 124,
    OmniModeOn                  = 125,
    MonoModeOn                  = 126,
    PolyModeOn                  = 127
} MidiControlChangeNumber;

typedef struct 
{
	typedef enum 
	{
		PitchBendSensitivity    = 0x0000,
		ChannelFineTuning       = 0x0001,
		ChannelCoarseTuning     = 0x0002,
		SelectTuningProgram     = 0x0003,
		SelectTuningBank        = 0x0004,
		ModulationDepthRange    = 0x0005,
		NullFunction            = (0x7f << 7) + 0x7f,
	} RegisteredParameterNumbers;
} RPN;

#endif // MIDI_DEFS_H