/*
Audio Generation for Raspberry Pi Synthesizer
By: William Abajian and Stephen Scneider
Copyright: See below
Function 'play_api_test' is the one of interest,
all else is the same as before, besides some
imported libraries and arrays and edits to main
*/
/*
Copyright (c) 2012, Broadcom Europe Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Audio output demo using OpenMAX IL though the ilcient helper library

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <semaphore.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <zmq.h>

#include "bcm_host.h"
#include "ilclient.h"

#include <stdio.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <string.h>
#include <linux/limits.h>


#define DELAY_LENGTH    196500
#define N_WAVE          1024    /* dimension of Sinewave[] */
#define PI (1<<16>>1)

/*All our waves we defined in sinewave.c*/
#define SIN1(x) Sinewave_1[((x)>>6) & (N_WAVE-1)]
#define SIN2(x) Sinewave_2[((x)>>6) & (N_WAVE-1)]
#define SIN3(x) Sinewave_3[((x)>>6) & (N_WAVE-1)]
#define SIN4(x) Sinewave_4[((x)>>6) & (N_WAVE-1)]
#define SIN5(x) Sinewave_5[((x)>>6) & (N_WAVE-1)]
#define COS(x) SIN((x)+(PI>>1))

#define TRI1(x) Trianglewave_1[((x)>>6) & (N_WAVE-1)]
#define TRI2(x) Trianglewave_2[((x)>>6) & (N_WAVE-1)]
#define TRI3(x) Trianglewave_3[((x)>>6) & (N_WAVE-1)]
#define TRI4(x) Trianglewave_4[((x)>>6) & (N_WAVE-1)]
#define TRI5(x) Trianglewave_5[((x)>>6) & (N_WAVE-1)]

#define SQR1(x) Squarewave_1[((x)>>6) & (N_WAVE-1)]
#define SQR2(x) Squarewave_2[((x)>>6) & (N_WAVE-1)]
#define SQR3(x) Squarewave_3[((x)>>6) & (N_WAVE-1)]
#define SQR4(x) Squarewave_4[((x)>>6) & (N_WAVE-1)]
#define SQR5(x) Squarewave_5[((x)>>6) & (N_WAVE-1)]

#define SAW1(x) Sawtoothwave_1[((x)>>6) & (N_WAVE-1)]
#define SAW2(x) Sawtoothwave_2[((x)>>6) & (N_WAVE-1)]
#define SAW3(x) Sawtoothwave_3[((x)>>6) & (N_WAVE-1)]
#define SAW4(x) Sawtoothwave_4[((x)>>6) & (N_WAVE-1)]
#define SAW5(x) Sawtoothwave_5[((x)>>6) & (N_WAVE-1)]


#define OUT_CHANNELS(num_channels) ((num_channels) > 4 ? 8: (num_channels) > 2 ? 4: (num_channels))
extern short Sinewave_1[];
extern short Sinewave_2[];
extern short Sinewave_3[];
extern short Sinewave_4[];
extern short Sinewave_5[];

extern short Trianglewave_1[];
extern short Trianglewave_2[];
extern short Trianglewave_3[];
extern short Trianglewave_4[];
extern short Trianglewave_5[];

extern short Squarewave_1[];
extern short Squarewave_2[];
extern short Squarewave_3[];
extern short Squarewave_4[];
extern short Squarewave_5[];

extern short Sawtoothwave_1[];
extern short Sawtoothwave_2[];
extern short Sawtoothwave_3[];
extern short Sawtoothwave_4[];
extern short Sawtoothwave_5[];

extern float g1_scale[];
int g_scale[64];

#ifndef countof
    #define countof(arr) (sizeof(arr) / sizeof(arr[0]))
#endif

#define BUFFER_SIZE_SAMPLES 1024

typedef int int32_t;

typedef struct {
    sem_t sema;
    ILCLIENT_T *client;
    COMPONENT_T *audio_render;
    COMPONENT_T *list[2];
    OMX_BUFFERHEADERTYPE *user_buffer_list; // buffers owned by the client
    uint32_t num_buffers;
    uint32_t bytes_per_sample;
} AUDIOPLAY_STATE_T;

static void input_buffer_callback(void *data, COMPONENT_T *comp)
{
    // do nothing - could add a callback to the user
    // to indicate more buffers may be available.
}

int32_t audioplay_create(AUDIOPLAY_STATE_T **handle,
                        uint32_t sample_rate,
                        uint32_t num_channels,
                        uint32_t bit_depth,
                        uint32_t num_buffers,
                        uint32_t buffer_size)
{
    uint32_t bytes_per_sample = (bit_depth * OUT_CHANNELS(num_channels)) >> 3;
    int32_t ret = -1;
 
    *handle = NULL;
 
    // basic sanity check on arguments
    if(sample_rate >= 8000 && sample_rate <= 192000 &&
       (num_channels >= 1 && num_channels <= 8) &&
       (bit_depth == 16 || bit_depth == 32) &&
       num_buffers > 0 &&
       buffer_size >= bytes_per_sample)
    {
        // buffer lengths must be 16 byte aligned for VCHI
        int size = (buffer_size + 15) & ~15;
        AUDIOPLAY_STATE_T *st;
  
        // buffer offsets must also be 16 byte aligned for VCHI
        st = calloc(1, sizeof(AUDIOPLAY_STATE_T));
  
        if(st)
        {
            OMX_ERRORTYPE error;
            OMX_PARAM_PORTDEFINITIONTYPE param;
            OMX_AUDIO_PARAM_PCMMODETYPE pcm;
            int32_t s;
   
            ret = 0;
            *handle = st;
   
            // create and start up everything
            s = sem_init(&st->sema, 0, 1);
            assert(s == 0);
   
            st->bytes_per_sample = bytes_per_sample;
            st->num_buffers = num_buffers;
   
            st->client = ilclient_init();
            assert(st->client != NULL);
   
            ilclient_set_empty_buffer_done_callback(st->client, input_buffer_callback, st);
   
            error = OMX_Init();
            assert(error == OMX_ErrorNone);
   
            ilclient_create_component(st->client, &st->audio_render, "audio_render", ILCLIENT_ENABLE_INPUT_BUFFERS | ILCLIENT_DISABLE_ALL_PORTS);
            assert(st->audio_render != NULL);
   
            st->list[0] = st->audio_render;
   
            // set up the number/size of buffers
            memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
            param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
            param.nVersion.nVersion = OMX_VERSION;
            param.nPortIndex = 100;
   
            error = OMX_GetParameter(ILC_GET_HANDLE(st->audio_render), OMX_IndexParamPortDefinition, &param);
            assert(error == OMX_ErrorNone);
   
            param.nBufferSize = size;
            param.nBufferCountActual = num_buffers;
   
            error = OMX_SetParameter(ILC_GET_HANDLE(st->audio_render), OMX_IndexParamPortDefinition, &param);
            assert(error == OMX_ErrorNone);
   
            // set the pcm parameters
            memset(&pcm, 0, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
            pcm.nSize = sizeof(OMX_AUDIO_PARAM_PCMMODETYPE);
            pcm.nVersion.nVersion = OMX_VERSION;
            pcm.nPortIndex = 100;
            pcm.nChannels = OUT_CHANNELS(num_channels);
            pcm.eNumData = OMX_NumericalDataSigned;
            pcm.eEndian = OMX_EndianLittle;
            pcm.nSamplingRate = sample_rate;
            pcm.bInterleaved = OMX_TRUE;
            pcm.nBitPerSample = bit_depth;
            pcm.ePCMMode = OMX_AUDIO_PCMModeLinear;
   
            switch(num_channels) {
            case 1:
                pcm.eChannelMapping[0] = OMX_AUDIO_ChannelCF;
                break;
            case 3:
                pcm.eChannelMapping[2] = OMX_AUDIO_ChannelCF;
                pcm.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
                pcm.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
                break;
            case 8:
                pcm.eChannelMapping[7] = OMX_AUDIO_ChannelRS;
            case 7:
                pcm.eChannelMapping[6] = OMX_AUDIO_ChannelLS;
            case 6:
                pcm.eChannelMapping[5] = OMX_AUDIO_ChannelRR;
            case 5:
                pcm.eChannelMapping[4] = OMX_AUDIO_ChannelLR;
            case 4:
                pcm.eChannelMapping[3] = OMX_AUDIO_ChannelLFE;
                pcm.eChannelMapping[2] = OMX_AUDIO_ChannelCF;
            case 2:
                pcm.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
                pcm.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
                break;
            }
   
            error = OMX_SetParameter(ILC_GET_HANDLE(st->audio_render), OMX_IndexParamAudioPcm, &pcm);
            assert(error == OMX_ErrorNone);
   
            ilclient_change_component_state(st->audio_render, OMX_StateIdle);
            if(ilclient_enable_port_buffers(st->audio_render, 100, NULL, NULL, NULL) < 0)
            {
                // error
                ilclient_change_component_state(st->audio_render, OMX_StateLoaded);
                ilclient_cleanup_components(st->list);
    
                error = OMX_Deinit();
                assert(error == OMX_ErrorNone);
    
                ilclient_destroy(st->client);
    
                sem_destroy(&st->sema);
                free(st);
                *handle = NULL;
                return -1;
            }
   
            ilclient_change_component_state(st->audio_render, OMX_StateExecuting);
        }
    }

    return ret;
}

int32_t audioplay_delete(AUDIOPLAY_STATE_T *st)
{
    OMX_ERRORTYPE error;
 
    ilclient_change_component_state(st->audio_render, OMX_StateIdle);
 
    error = OMX_SendCommand(ILC_GET_HANDLE(st->audio_render), OMX_CommandStateSet, OMX_StateLoaded, NULL);
    assert(error == OMX_ErrorNone);
 
    ilclient_disable_port_buffers(st->audio_render, 100, st->user_buffer_list, NULL, NULL);
    ilclient_change_component_state(st->audio_render, OMX_StateLoaded);
    ilclient_cleanup_components(st->list);
 
    error = OMX_Deinit();
    assert(error == OMX_ErrorNone);
 
    ilclient_destroy(st->client);
 
    sem_destroy(&st->sema);
    free(st);
 
    return 0;
}

uint8_t *audioplay_get_buffer(AUDIOPLAY_STATE_T *st)
{
    OMX_BUFFERHEADERTYPE *hdr = NULL;
 
    hdr = ilclient_get_input_buffer(st->audio_render, 100, 0);
 
    if(hdr)
    {
        // put on the user list
        sem_wait(&st->sema);
  
        hdr->pAppPrivate = st->user_buffer_list;
        st->user_buffer_list = hdr;
  
        sem_post(&st->sema);
    }
 
    return hdr ? hdr->pBuffer : NULL;
}

int32_t audioplay_play_buffer(AUDIOPLAY_STATE_T *st,
                              uint8_t *buffer,
                              uint32_t length)
{
    OMX_BUFFERHEADERTYPE *hdr = NULL, *prev = NULL;
    int32_t ret = -1;
 
    if(length % st->bytes_per_sample)
        return ret;
 
    sem_wait(&st->sema);
 
    // search through user list for the right buffer header
    hdr = st->user_buffer_list;
    while(hdr != NULL && hdr->pBuffer != buffer && hdr->nAllocLen < length)
    {
        prev = hdr;
        hdr = hdr->pAppPrivate;
    }
 
    if(hdr) // we found it, remove from list
    {
        ret = 0;
        if(prev)
            prev->pAppPrivate = hdr->pAppPrivate;
        else
            st->user_buffer_list = hdr->pAppPrivate;
    }
 
    sem_post(&st->sema);
 
    if(hdr)
    {
        OMX_ERRORTYPE error;
  
        hdr->pAppPrivate = NULL;
        hdr->nOffset = 0;
        hdr->nFilledLen = length;
  
        error = OMX_EmptyThisBuffer(ILC_GET_HANDLE(st->audio_render), hdr);
        assert(error == OMX_ErrorNone);
    }
 
    return ret;
}

int32_t audioplay_set_dest(AUDIOPLAY_STATE_T *st, const char *name)
{
    int32_t success = -1;
    OMX_CONFIG_BRCMAUDIODESTINATIONTYPE ar_dest;
 
    if (name && strlen(name) < sizeof(ar_dest.sName))
    {
        OMX_ERRORTYPE error;
        memset(&ar_dest, 0, sizeof(ar_dest));
        ar_dest.nSize = sizeof(OMX_CONFIG_BRCMAUDIODESTINATIONTYPE);
        ar_dest.nVersion.nVersion = OMX_VERSION;
        strcpy((char *)ar_dest.sName, name);
  
        error = OMX_SetConfig(ILC_GET_HANDLE(st->audio_render), OMX_IndexConfigBrcmAudioDestination, &ar_dest);
        assert(error == OMX_ErrorNone);
        success = 0;
    }
 
    return success;
}


uint32_t audioplay_get_latency(AUDIOPLAY_STATE_T *st)
{
    OMX_PARAM_U32TYPE param;
    OMX_ERRORTYPE error;
 
    memset(&param, 0, sizeof(OMX_PARAM_U32TYPE));
    param.nSize = sizeof(OMX_PARAM_U32TYPE);
    param.nVersion.nVersion = OMX_VERSION;
    param.nPortIndex = 100;
 
    error = OMX_GetConfig(ILC_GET_HANDLE(st->audio_render), OMX_IndexConfigAudioRenderingLatency, &param);
    assert(error == OMX_ErrorNone);
 
    return param.nU32;
}

#define CTTW_SLEEP_TIME 10
#define MIN_LATENCY_TIME 20

static const char *audio_dest[] = {"local", "hdmi"};
void play_api_test(int samplerate, int bitdepth, int nchannels, int dest)
{
    AUDIOPLAY_STATE_T *st;
    int32_t ret;
    unsigned int i, j, n, k; //pointers to count
    int phase = 0; //index into wave array
    int inc = 500<<16; //frequency to be outputted
    int inc2; //secondary frequency for modulation
    int dinc = 0; //ammount to vary frequency for wah=wah
    int buffer_size = (BUFFER_SIZE_SAMPLES * bitdepth * OUT_CHANNELS(nchannels))>>3;
    unsigned char buffer[100]; //buffer for SPI to ADC
 
    int volume = 1023; //volume setting: 0-1023
    int octave = 0; //octave setting:0-3
    int shape; //shape: sine, triangle, square, saw
    int harmonic; //number proportional to number of harmonics
    int play = 1; //Output flag
    int wowow = 0; //Modulate flag
    int wowow_freq = 0; //Modulate Setting
    int overdrive = 0; //Modulate flag
    int16_t val; //analog value to be outputted
    uint8_t *lastbuf;
    int easy = 0;// Easy mode flag


    int delay_number = 0; //Number of delay samples: 0-3
    unsigned long delay_freq = 16000; //Frequnecy of delay samples
    int16_t delay_val[4][DELAY_LENGTH]; //Stores delay samples
    unsigned long delay_pointer[4]; //Points to next location in delay_val

    //initialize pointers
    for(k = 0; k<4; k++){
        delay_pointer[k] = 0;
    } 
 
    /*ZMQ Setup*/
    void *context = zmq_ctx_new ();
    void *requester = zmq_socket (context, ZMQ_PULL);
    zmq_connect (requester, "tcp://localhost:5555");
    char z_buffer[10];

    /*Audio Setup*/
    assert(dest == 0 || dest == 1);
 
    ret = audioplay_create(&st, samplerate, nchannels, bitdepth, 10, buffer_size);
    assert(ret == 0);
 
    ret = audioplay_set_dest(st, audio_dest[dest]);
    assert(ret == 0);
 
    // iterate for 5 seconds worth of packets
    for (n=0; n<((samplerate * 1000)/ BUFFER_SIZE_SAMPLES); n++)
    {
        uint8_t *buf;
        int16_t *p;
        uint32_t latency;   
        while((buf = audioplay_get_buffer(st)) == NULL)
            usleep(10*1000); 
        p = (int16_t *) buf;    

        /* Poll the Inputs*/
        
        //Modulation toggle on right button
        if(digitalRead(18) == 1){ //if right button not pressed
            wowow = 0; //turn it off
        }
        else{ //if pressed
            wowow = 1; //turn it on
        }
    
        //Attack settings (flip if statement to press to pause)
        if(digitalRead(23) == 1){//if moddle button not pressed
            play = 0; //don't play sounds
        }
        else{
            play = 1; //play sounds
        }

        //Shutdown Button
        if(digitalRead(24) == 0){//if left button is pressed
            system("sudo shutdown -h now"); //shutdown Pi
        }

        //Input Toggle-switches between CV and knob for frequnecy
        if(digitalRead(4) == 0){ //if top switch is flipped up
            zmq_recv (requester, z_buffer, 10, ZMQ_NOBLOCK); //receive packet from CV
            inc = atoi(z_buffer)<<9; //change frequency to received value
        }
        else{ //if flipped down
            buffer[1] = 0xC0; //prepare output message
            buffer[0] = 0x1; //for ADC on SPI
            wiringPiSPIDataRW(0,buffer,3); //send request to ADC
            inc = ((buffer[1]<<8) + buffer[2])<<16; //set frequency to response
        }

        //Easy Mode Toggle
        if(digitalRead(17) == 0){ //if middle switch is flipped up
            easy = 0; //don't bucket frequencies
        }
        else{ //if flipped down
            easy = 1; //round frequencies to g-scale
        }

        //'Overdrive' Toggle
        if(digitalRead(22) == 0){ //if bttom switch is flipped up
            overdrive = 0; //Modulate uses wah-wah
        }
        else{ //if flipped down
            overdrive = 1; //Modulate uses 'overdrive'
        }

        //Wowow Frequency Knob
        buffer[1] = 0x80; //prepare output message
        buffer[0] = 0x1; //for ADC on SPI
        wiringPiSPIDataRW(0,buffer,3); //send request to ADC
        wowow_freq = ((buffer[1]<<8) + buffer[2]); //set frequency to response
    
        //Delay Frequency Knob
        buffer[1] = 0x90; //prepare output message
        buffer[0] = 0x1; //for ADC on SPI
        wiringPiSPIDataRW(0,buffer,3); //send request to ADC
        delay_freq = (((buffer[1]<<8) + buffer[2])<<6) + 10; //set frequency to response
    
    
        //Delay Number Knob
        buffer[1] = 0xA0; //prepare output message
        buffer[0] = 0x1; //for ADC on SPI
        wiringPiSPIDataRW(0,buffer,3); //send request to ADC
        delay_number = (buffer[1]); //Set number of samples to response>>8

        //Shape Knob
        buffer[1] = 0xB0; //prepare output message
        buffer[0] = 0x1; //for ADC on SPI
        wiringPiSPIDataRW(0,buffer,3); //send request to ADC
        shape = buffer[1]; //Set shape to response >> 8

        //Harmonic Knob
        buffer[1] = 0xD0; //prepare output message
        buffer[0] = 0x1; //for ADC on SPI
        wiringPiSPIDataRW(0,buffer,3); //send request to ADC
        harmonic = buffer[1]; //Set harmonics to response >> 8

        //Octave Slider
        buffer[1] = 0xE0; //prepare output message
        buffer[0] = 0x1; //for ADC on SPI
        wiringPiSPIDataRW(0,buffer,3); //send request to ADC
        octave = 3-buffer[1]; //set octave to response >> 8

        //Volume Slider
        buffer[1] = 0xF0; //prepare output message
        buffer[0] = 0x1; //for ADC on SPI
        wiringPiSPIDataRW(0,buffer,3); //send request to ADC
        volume = 1023-((buffer[1]<<8) + buffer[2]); //Set volume to response
        
        if(easy){ //if easy mode is on
            for(k=0;k<63;k++){ //cycle through G-Scale
                if(inc <= g_scale[k]){ //if set frequency is below indexed frequnecy
                    inc = g_scale[k]; //set frequency to indexed frequency
                    break; //stop cycling
                }
            }
        }

        if(overdrive){ //if overdrive is on
            inc2 = inc; //make it sound funky
        }

        //Prepare one period's worth of values
        for (i=0; i<BUFFER_SIZE_SAMPLES; i++)
        {   
            //Switch based on which shape is chosen
            switch(shape){
                //If sinewave
                case 0:
                    //Switch based on which harmonic is chosen
                    switch(harmonic){
                        case 0:
                            val = play*(volume * SIN1(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 1:
                            val = play*(volume * SIN2(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 2:
                            val = play*(volume * SIN3(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 3:
                            val = play*(volume * SIN4(phase<<(octave)))>>10; //10 is for volume
                            break;
                        default:
                            val = play*(volume * SIN1(phase<<(octave)))>>10; //10 is for volume
                    }
                    break;
                //If Trianglewave
                case 1:
                    //Switch based on which harmonic is chosen
                    switch(harmonic){
                        case 0:
                            /*val is composed of play to determine if it should be 0 or not,
                            volume scales it to the volume, the the appropriate table is indexed
                            into, changing the octave by shifting to double, and it is scaled 
                            to the correct magnitude for output by dividing by 1024*/
                            val = play*(volume * TRI1(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 1:
                            val = play*(volume * TRI2(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 2:
                            val = play*(volume * TRI3(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 3:
                            val = play*(volume * TRI4(phase<<(octave)))>>10; //10 is for volume
                            break;
                        default:
                            val = play*(volume * TRI1(phase<<(octave)))>>10; //10 is for volume
                    }
            
                    break;
                //If Squarewave
                case 2:
                    //Switch based on which harmonic is chosen
                    switch(harmonic){
                        case 0:
                            val = play*(volume * SQR1(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 1:
                            val = play*(volume * SQR2(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 2:
                            val = play*(volume * SQR3(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 3:
                            val = play*(volume * SQR4(phase<<(octave)))>>10; //10 is for volume
                            break;
                        default:
                            val = play*(volume * SQR1(phase<<(octave)))>>10; //10 is for volume
                    }
                    break;
                //If Saw wave
                case 3:
                    //Switch based on which harmonic is chosen
                    switch(harmonic){
                        case 0:
                            val = play*(volume * SAW1(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 1:
                            val = play*(volume * SAW2(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 2:
                            val = play*(volume * SAW3(phase<<(octave)))>>10; //10 is for volume
                            break;
                        case 3:
                            val = play*(volume * SAW4(phase<<(octave)))>>10; //10 is for volume
                            break;
                        default:
                            val = play*(volume * SAW1(phase<<(octave)))>>10; //10 is for volume
                    }
                    break;
                default:
                    val = play*(volume * SIN1(phase<<(octave)))>>10; //10 is for volume

            }
            //If modulation is on
            if(wowow == 1){
                inc2 += dinc; //vary frequency by dinc
                if (inc2>>16 < wowow_freq) //if inc is below the upper frequnecy
                    dinc++; //increment the incrememnter
                else //else 
                    dinc--; //decrement the incrememter
            }
            //if modulation is off
            else{
                inc2 = inc; //set frequency to actual frequency
                dinc = 0; //reset frequency incrementer
            }

            //Define how fast to index lookup table based on frequency
            phase += inc2>>16;
            
            //Save current val into delay tables for later playback
            for(k = 0;k<delay_number;k++){
                //save val at pointer
                delay_val[k][delay_pointer[k]] = val;
                //increment pointer for next value, wrapping when appropriate
                delay_pointer[k] = (delay_pointer[k]+1)%(delay_freq*(k+1));
            }

            //Push val onto buffer to be outputted
            for(j=0; j<OUT_CHANNELS(nchannels); j++){
                if (bitdepth == 32)
                    *p++ = 0;
                *p = .5*val;

                //if there is delay
                if(delay_number>0){
                    //add the first delay buffered value, scaled
                    *p += .125*delay_val[0][delay_pointer[0]];
                }
                //if there is 2 or more delay samples
                if(delay_number>1){
                    //add the next delay buffered value, scaled
                    *p += .0625*delay_val[1][delay_pointer[1]];
                }
                //if there is 3 or more delay samples
                if(delay_number>2){
                    //add the next delay buffered value, scaled
                    *p += .03125*delay_val[2][delay_pointer[2]];
                }
                *p++;
            }
        }

        // try and wait for a minimum latency time (in ms) before
        // sending the next packet
        while((latency = audioplay_get_latency(st)) > (samplerate * (MIN_LATENCY_TIME + CTTW_SLEEP_TIME) / 1000)){
            usleep(CTTW_SLEEP_TIME*1000);
        }
        ret = audioplay_play_buffer(st, buf, buffer_size);
        assert(ret == 0);
    }
   audioplay_delete(st);
}

int main (int argc, char **argv)
{
    // 0=headphones, 1=hdmi
    int audio_dest = 0;
    // audio sample rate in Hz
    int samplerate = 48000;
    // numnber of audio channels
    int channels = 2;
    // number of bits per sample
    int bitdepth = 16;
    int i = 0;
    bcm_host_init();
    
    if (argc > 1)
        audio_dest = atoi(argv[1]);
    if (argc > 2)
        channels = atoi(argv[2]);
    if (argc > 3)
        samplerate = atoi(argv[3]);

    wiringPiSetupGpio();//Setup Pins using BCM numbering
    pinMode(18, INPUT); //Set Pin 18 to Input (Right Button)
    pinMode(23, INPUT); //Set Pin 23 to Input (Middle Button)
    pinMode(24, INPUT); //Set Pin 24 to Input (Left Button)
    pinMode(4, INPUT); //Set Pin 4 to Input (Top Switch)
    pinMode(17, INPUT); //Set Pin 17 to Input (Middle Switch)
    pinMode(22, INPUT); //Set Pin 22 to Input (Bottom Switch)
    wiringPiSPISetup(0, 1000000); //Setup SPI with 1MHz Clockspeed
    
    //Print starting message to terminal
    printf("Outputting audio to %s\n", audio_dest==0 ? "analogue":"hdmi");
    
    //convert g_scale to inc settings (ie int>>16)
    for(i=0;i<63;i++){
        g_scale[i] = (int)(g1_scale[i]*65536);
    }    
    
    //call play function
    play_api_test(samplerate, bitdepth, channels, audio_dest);
    return 0;
}
