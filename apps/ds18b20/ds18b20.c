/*
 * Copyright (c) 2012, Jens Nielsen
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL JENS NIELSEN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "contiki.h"
#include "sensors.h"
#include "ownet.h"
#include "owlink.h"
#include <string.h> /*for memset*/

#define NUM_DEVICES 4

typedef struct
{
    unsigned char serial[ 8 ];
    signed short lastTemp;
#ifndef DS18B20_CONTINUOUS_POLL
    struct pt convert_pt;
#endif
}device_t;

static device_t devices[ NUM_DEVICES ];
static unsigned char num_devices;

static int active;
static int idle;


static void ds18b20_scan();
static char ds18b20_isParasite();
static void ds18b20_startConvert(void);
static signed short ds18b20_fetchTemp( unsigned char device );

#ifdef DS18B20_CONTINUOUS_POLL

PROCESS(ds18b20_process, "DS18B20");

PROCESS_THREAD(ds18b20_process, ev, data)
{
    unsigned char i;

    PROCESS_BEGIN();

    while( 1 )
    {
        ds18b20_scan();

        ds18b20_startConvert();

        if ( ds18b20_isParasite() )
        {
            /*TODO!*/
        }
        else
        {
            PROCESS_WAIT_UNTIL( owReadBit() != 0 );
        }

        for ( i = 0; i < num_devices; i++ )
        {
            signed short temp = ds18b20_fetchTemp( i );
            devices[ i ].lastTemp = temp;
        }
    }

    PROCESS_END();
}

#else

static
PT_THREAD( ds18b20_updateTemp( int device ) )
{
    signed short temp;
    PT_BEGIN( &devices[ device ].convert_pt );

    if ( idle != 1 )
        PT_WAIT_UNTIL( &devices[ device ].convert_pt, idle == 1 );

    idle = 0;

    ds18b20_startConvert();

    if ( ds18b20_isParasite() )
    {
        /*TODO!*/
    }
    else
    {
        PT_WAIT_UNTIL( &devices[ device ].convert_pt, owReadBit() != 0 );
    }

    temp = ds18b20_fetchTemp( device );
    devices[ device ].lastTemp = temp;

    idle = 1;

    PT_END( &devices[ device ].convert_pt );
}

#endif

void ds18b20_init()
{
    memset( devices, 0, sizeof(devices) );

#ifdef DS18B20_CONTINUOUS_POLL
    process_start( &ds18b20_process, NULL );
#else
    {
        char i;
        for ( i=0; i<NUM_DEVICES; i++ )
        {
            PT_INIT( &devices[ i ].convert_pt );
        }
    }
#endif
}



static void ds18b20_scan()
{
    unsigned char found;
    unsigned char i;

    num_devices = 0;

    found = owFirst( 0, 1, 0 );
    if ( found )
    {
        owSerialNum( 0, devices[ 0 ].serial, 1 );
        num_devices++;
    }

    for ( i = 1; found && i < NUM_DEVICES; i++ )
    {
        found = owNext( 0, 1, 0 );
        if ( found )
        {
            owSerialNum( 0, devices[ i ].serial, 1 );
            num_devices++;
        }
    }
}

static char ds18b20_isParasite()
{
    owTouchReset();
    owWriteByte(0xCC);
    owWriteByte(0xB4);
    return ( owReadBit() == 0 );
}

static void ds18b20_startConvert()
{
    /*issue convert temp command to all devices*/
    owTouchReset();
    owWriteByte(0xCC);
    owWriteByte(0x44);
}

static signed short ds18b20_fetchTemp( unsigned char device )
{
    if ( device < NUM_DEVICES && devices[ device ].serial[ 0 ] != 0 )
    {
        unsigned char i;
        unsigned char b1, b2;

        /*address specified device*/
        owTouchReset();
        owWriteByte(0x55);
        for (i = 0; i < 8; i++)
        {
            owWriteByte(devices[ device ].serial[ i ]);
        }

        /*read the first two bytes of scratchpad*/
        owWriteByte(0xBE);
        b1 = owReadByte();
        b2 = owReadByte();

        return ( (signed short) b2 << 8 ) | ( b1 & 0xFF );
    }

    return ~0;
}




/*---------------------------------------------------------------------------*/
static int
value(int type, int device)
{
  int temp;
  signed short t;

#ifndef DS18B20_CONTINUOUS_POLL
  ds18b20_updateTemp( device );
#endif
  t = devices[ device ].lastTemp;

  temp = ( (t + 8) / 16 );
  return temp;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    active = 0;
    ds18b20_init();
    return 1;
  case SENSORS_ACTIVE:
    if(value) {
      if(!active) {
        active = 1;
        idle = 1;
        ds18b20_scan();
      }
    } else {
      active = 0;
    }
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type)
  {
    case SENSORS_ACTIVE:
      return active;
    case SENSORS_READY:
      return idle;
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
#define value_fn( device ) static int value##device (int type) { return value( type, device ); }
#define sensor_def( device ) \
        SENSORS_SENSOR(temperature_sensor##device, "ds18b20_"#device, \
                       value##device, configure, status);
#define sensor( device ) \
  value_fn( device ) \
  sensor_def( device )

#define temperature_sensor0 temperature_sensor
sensor( 0 );
sensor( 1 );
sensor( 2 );
sensor( 3 );
