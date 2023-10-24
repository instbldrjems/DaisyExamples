 #include "daisy_patch.h"
#include "daisysp.h"
#include <string>

using namespace daisy;
using namespace daisysp;

#define NUM_VOICES 32
#define MAX_DELAY ((size_t)(10.0f * 48000.0f))

MidiUsbHandler midi;


// Hardware
DaisyPatch hw;

// Synthesis
PolyPluck<NUM_VOICES> synth;
// 10 second delay line on the external SDRAM
DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delay;
ReverbSc                                  verb;

// Persistent filtered Value for smooth delay time changes.
float smooth_time;

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    float sig, delsig;           // Mono Audio Vars
    float trig, nn, decay;       // Pluck Vars
    float deltime, delfb, kval;  // Delay Vars
    float dry, send, wetl, wetr; // Effects Vars
    float samplerate;

    // Assign Output Buffers
    float *out_left, *out_right;
    out_left  = out[0];
    out_right = out[1];

    samplerate = hw.AudioSampleRate();
    hw.ProcessDigitalControls();
    hw.ProcessAnalogControls();

    /** Listen to MIDI for new changes */
    midi.Listen();

    trig = 0.0f;

    /** When there are messages waiting in the queue... */
        while(midi.HasEvents())
        {
            /** Pull the oldest one from the list... */
            auto msg = midi.PopEvent();
            switch(msg.type)
            {
                case NoteOn:
                {
                    /** and change the frequency of the oscillator */
                    auto note_msg = msg.AsNoteOn();
                    if(note_msg.velocity != 0)
                    {
                         trig = 1.0f;
                    }
                }
                break;
                    // Since we only care about note-on messages in this example
                    // we'll ignore all other message types
                default: break;
            }
        }

    // Handle Triggering the Plucks
    //trig = 0.0f;
    //if(hw.encoder.RisingEdge() || hw.gate_input[DaisyPatch::GATE_IN_1].Trig())
        //trig = 1.0f;

    // Set MIDI Note for new Pluck notes.
    nn = 24.0f + hw.GetKnobValue(DaisyPatch::CTRL_1) * 60.0f;
    nn = static_cast<int32_t>(nn); // Quantize to semitones

    // Read knobs for decay;
    decay = 0.5f + (hw.GetKnobValue(DaisyPatch::CTRL_2) * 0.5f);
    synth.SetDecay(decay);

    // Get Delay Parameters from knobs.
    kval    = hw.GetKnobValue(DaisyPatch::CTRL_3);
    deltime = (0.001f + (kval * kval) * 5.0f) * samplerate;
    delfb   = hw.GetKnobValue(DaisyPatch::CTRL_4);

    // Synthesis.
    for(size_t i = 0; i < size; i++)
    {
        // Smooth delaytime, and set.
        fonepole(smooth_time, deltime, 0.0005f);
        delay.SetDelay(smooth_time);

        // Synthesize Plucks
        sig = synth.Process(trig, nn);

        //		// Handle Delay
        delsig = delay.Read();
        delay.Write(sig + (delsig * delfb));

        // Create Reverb Send
        dry  = sig + delsig;
        send = dry * 0.6f;
        verb.Process(send, send, &wetl, &wetr);

        // Output
        out_left[i]  = dry + wetl;
        out_right[i] = dry + wetr;
    }
}

int main(void)
{
    // Init everything.
    float samplerate;
    hw.Init();
    samplerate = hw.AudioSampleRate();

    /** Initialize USB Midi 
		 *  by default this is set to use the built in (USB FS) peripheral.
		 * 
		 *  by setting midi_cfg.transport_config.periph = MidiUsbTransport::Config::EXTERNAL
		 *  the USB HS pins can be used (as FS) for MIDI 
		 */
    MidiUsbHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = MidiUsbTransport::Config::INTERNAL;
    midi.Init(midi_cfg);

    //briefly display the module name
    std::string str  = "Pluck Echo";
    char *      cstr = &str[0];
    hw.display.WriteString(cstr, Font_7x10, true);
    hw.display.Update();
    hw.DelayMs(1000);

    synth.Init(samplerate);

    delay.Init();
    delay.SetDelay(samplerate * 0.8f); // half second delay

    verb.Init(samplerate);
    verb.SetFeedback(0.85f);
    verb.SetLpFreq(2000.0f);

    // Start the ADC and Audio Peripherals on the Hardware
    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    for(;;)
    {
        hw.DisplayControls(false);
 
    }
}
