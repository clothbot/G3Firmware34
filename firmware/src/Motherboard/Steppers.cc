/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

/*
  This software uses the following components from Jetty Firmware:
        JKN Advance
        YAJ (Yet Another Jerk)
        Advance Pressure Relax
  authored by Dan Newman and Jetty.
*/

#define __STDC_LIMIT_MACROS
#include "Steppers.hh"
#include "StepperAxis.hh"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <math.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "EepromMap.hh"
#include "Eeprom.hh"
#include "EepromDefaults.hh"
#ifdef   SMALL_4K_RAM
#include "Errors.hh"
#endif

#ifdef HAS_STEPPER_ACCELERATION
#include "StepperAccel.hh"
#endif

// Which master_steps to use when calculating the feed rate
#define MASTER_STEPS planner_master_steps_cfr

#ifdef   SMALL_4K_RAM
#define MINIMUM_FREE_SRAM 100
#endif

#if defined(STORE_RAM_USAGE_TO_EEPROM) || defined(SMALL_4K_RAM)

//Stack checking
//http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=52249
extern uint8_t _end;
extern uint8_t __stack;

#define STACK_CANARY 0xc5

void StackPaint(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));

void StackPaint(void)
{
#if 0
    uint8_t *p = &_end;

    while(p <= &__stack)
    {
        *p = STACK_CANARY;
        p++;
    }
#else
    __asm volatile ("    ldi r30,lo8(_end)\n"
                    "    ldi r31,hi8(_end)\n"
                    "    ldi r24,lo8(0xc5)\n" /* STACK_CANARY = 0xc5 */
                    "    ldi r25,hi8(__stack)\n"
                    "    rjmp .cmp\n"
                    ".loop:\n"
                    "    st Z+,r24\n"
                    ".cmp:\n"
                    "    cpi r30,lo8(__stack)\n"
                    "    cpc r31,r25\n"
                    "    brlo .loop\n"
                    "    breq .loop"::);
#endif
}


uint16_t StackCount(void)
{
    const uint8_t *p = &_end;
    uint16_t       c = 0;

    while(*p == STACK_CANARY && p <= &__stack)
    {
        p++;
        c++;
    }

    return c;
}

#endif

#ifdef STORE_RAM_USAGE_TO_EEPROM

void storeStackFreeToEeprom(void)
{
    uint16_t sc = StackCount();
    eeprom::putEepromUInt32(eeprom::RAM_USAGE_DEBUG, (uint32_t)sc);
}

#endif

namespace steppers
{


volatile bool is_running;
int32_t intervals;
volatile int32_t intervals_remaining;
StepperAxis axes[STEPPER_COUNT];
volatile bool is_homing;
#ifdef HAS_STEPPER_ACCELERATION
FPTYPE axis_steps_per_unit_inverse[NUM_AXIS];
#endif

#ifdef HAS_STEPPER_ACCELERATION
bool acceleration = false;
bool planner = false;
bool strangled = false;
uint8_t plannerMaxBufferSize;
volatile bool force_acceleration_off = false;

static Point lastTarget, droppedSegmentsRelative;
static int32_t	droppedSegmentsUs;
bool accelerationSwitchBackLockout;

// Segments are accelerated when segmentAccelState is true; unaccelerated otherwise
static bool segmentAccelState = true;

#endif

bool holdZ = false;

//Also requires DEBUG_ZADVANCE to be defined in StepperAccel.h
//#define TIME_STEPPER_INTERRUPT

#if defined(DEBUG_ZADVANCE) && defined(TIME_STEPPER_INTERRUPT)
uint16_t debugTimer = 0;
#endif


bool isRunning()
{
    return is_running || is_homing;
}

bool isHoming()
{
    return is_homing;
}

//public:
void init(Motherboard& motherboard)
{
    is_running = false;
    for (int i = 0; i < STEPPER_COUNT; i++)
    {
        axes[i] = StepperAxis(motherboard.getStepperInterface(i));
    }
}

void abort()
{
#ifdef HAS_STEPPER_ACCELERATION
    if ( acceleration )
    {
        quickStop();
        lastTarget = getCurrentPosition();
    }
#endif
    if (( is_homing ) && ( acceleration ) && ( force_acceleration_off )) switchToAcceleratedDriver();

    is_running = false;
    is_homing = false;
}

float convertAxisMMToFloat(int64_t st )
{
    float aspmf = (float)st;
    for (uint8_t i=0 ; i < STEPS_PER_MM_PRECISION; i ++ )
        aspmf /= 10.0;
    return aspmf;
}

//Returns true if the eeprom contains sane values
//We check Jog Mode is 0, 1 or 2 and STEPS_PER_MM_B is STEPS_PER_MM_B_DEFAULT
//Note that STEPS_PER_MM_B is currently unused.  If we ever use STEPS_PER_MM_B, we will want to find
//another method or just rely on JogMode.
//This is mainly a solution that gets around issues with RobG's firmware conflicting with this one.

bool eepromIsSane()
{
    uint8_t jogModeSettings = eeprom::getEeprom8(eeprom::JOG_MODE_SETTINGS);
    jogModeSettings = jogModeSettings >> 1;

    //Valid values are 0,1 or 2, anything else means we have corruption
    if ( jogModeSettings > 2 )	return false;

    if ( eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_B) != EEPROM_DEFAULT_STEPS_PER_MM_B )
        return false;

    return true;
}

void reset()
{
#ifdef HAS_STEPPER_ACCELERATION
    if ( ! eepromIsSane() ) eeprom::setJettyFirmwareDefaults();

    uint8_t accel = eeprom::getEeprom8(eeprom::STEPPER_DRIVER, EEPROM_DEFAULT_STEPPER_DRIVER);

    acceleration = accel & 0x01;
    planner = (accel & 0x02)?true:false;
    strangled = (accel & 0x04)?true:false;

    if ( strangled )
    {
        acceleration = true;
        setSegmentAccelState(false);
    }
    else
    {
        setSegmentAccelState(true);
    }

    if ( acceleration )
    {
        axis_steps_per_unit[X_AXIS] = convertAxisMMToFloat(eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_X, EEPROM_DEFAULT_STEPS_PER_MM_X));
        axis_steps_per_unit[Y_AXIS] = convertAxisMMToFloat(eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_Y, EEPROM_DEFAULT_STEPS_PER_MM_Y));
        axis_steps_per_unit[Z_AXIS] = convertAxisMMToFloat(eeprom::getEepromStepsPerMM(eeprom::STEPS_PER_MM_Z, EEPROM_DEFAULT_STEPS_PER_MM_Z));
        axis_steps_per_unit[E_AXIS] = (float)eeprom::getEepromUInt32(eeprom::ACCEL_E_STEPS_PER_MM, EEPROM_DEFAULT_ACCEL_E_STEPS_PER_MM) / 10.0;

        for ( uint8_t i = 0; i < NUM_AXIS; i ++ )
            axis_steps_per_unit_inverse[i] = FTOFP(1.0 / axis_steps_per_unit[i]);

        max_acceleration_units_per_sq_second[X_AXIS] = eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_X, EEPROM_DEFAULT_ACCEL_MAX_ACCELERATION_X);
        max_acceleration_units_per_sq_second[Y_AXIS] = eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Y, EEPROM_DEFAULT_ACCEL_MAX_ACCELERATION_Y);
        max_acceleration_units_per_sq_second[Z_AXIS] = eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_Z, EEPROM_DEFAULT_ACCEL_MAX_ACCELERATION_Z);
        max_acceleration_units_per_sq_second[E_AXIS] = eeprom::getEepromUInt32(eeprom::ACCEL_MAX_ACCELERATION_A, EEPROM_DEFAULT_ACCEL_MAX_ACCELERATION_A);

        for (uint8_t i = 0; i < NUM_AXIS; i ++)
            axis_steps_per_sqr_second[i] = (uint32_t)((float)max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i]);

        max_feedrate[X_AXIS] = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_X, EEPROM_DEFAULT_ACCEL_MAX_FEEDRATE_X));
        max_feedrate[Y_AXIS] = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Y, EEPROM_DEFAULT_ACCEL_MAX_FEEDRATE_Y));
        max_feedrate[Z_AXIS] = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_Z, EEPROM_DEFAULT_ACCEL_MAX_FEEDRATE_Z));
        max_feedrate[E_AXIS] = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_FEEDRATE_A, EEPROM_DEFAULT_ACCEL_MAX_FEEDRATE_A));

        p_acceleration		= FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_NORM, EEPROM_DEFAULT_ACCEL_MAX_EXTRUDER_NORM));
        p_retract_acceleration	= FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_EXTRUDER_RETRACT, EEPROM_DEFAULT_ACCEL_MAX_EXTRUDER_RETRACT));
        minimumfeedrate	  = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MIN_FEED_RATE, EEPROM_DEFAULT_ACCEL_MIN_FEED_RATE) / 10.0);
        mintravelfeedrate = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MIN_TRAVEL_FEED_RATE, EEPROM_DEFAULT_ACCEL_MIN_TRAVEL_FEED_RATE) / 10.0);
        extruder_deprime_steps	  = (int32_t)(((float)eeprom::getEepromUInt32(eeprom::ACCEL_EXTRUDER_DEPRIME, EEPROM_DEFAULT_ACCEL_EXTRUDER_DEPRIME) / 10.0) * axis_steps_per_unit[E_AXIS]);
        max_speed_change[X_AXIS]  = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_SPEED_CHANGE_X, EEPROM_DEFAULT_ACCEL_MAX_SPEED_CHANGE_X)	 / 10.0);
        max_speed_change[Y_AXIS]  = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_SPEED_CHANGE_Y, EEPROM_DEFAULT_ACCEL_MAX_SPEED_CHANGE_Y)	 / 10.0);
        max_speed_change[Z_AXIS]  = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_SPEED_CHANGE_Z, EEPROM_DEFAULT_ACCEL_MAX_SPEED_CHANGE_Z)	 / 10.0);
        max_speed_change[E_AXIS]  = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MAX_SPEED_CHANGE_A, EEPROM_DEFAULT_ACCEL_MAX_SPEED_CHANGE_A)	 / 10.0);

#ifdef FIXED
        smallest_max_speed_change = max_speed_change[Z_AXIS];
        for (uint8_t i = 0; i < NUM_AXIS; i++)
            if ( max_speed_change[i] < smallest_max_speed_change )
                smallest_max_speed_change = max_speed_change[i];
#endif
        FPTYPE advanceK	 	= FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_ADVANCE_K, EEPROM_DEFAULT_ACCEL_ADVANCE_K)		/ 100000.0);
        FPTYPE advanceK2	= FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_ADVANCE_K2, EEPROM_DEFAULT_ACCEL_ADVANCE_K2)		/ 100000.0);
        FPTYPE noodleDiameter	= FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_NOODLE_DIAMETER, EEPROM_DEFAULT_ACCEL_NOODLE_DIAMETER)	/ 100.0);

        minimumSegmentTime = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MIN_SEGMENT_TIME, EEPROM_DEFAULT_ACCEL_MIN_SEGMENT_TIME)		/ 10000.0);
        minimumPlannerSpeed = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_MIN_PLANNER_SPEED, EEPROM_DEFAULT_ACCEL_MIN_PLANNER_SPEED));
        extruder_only_max_feedrate = FTOFP((float)eeprom::getEepromUInt32(eeprom::ACCEL_REV_MAX_FEED_RATE, EEPROM_DEFAULT_ACCEL_REV_MAX_FEED_RATE));
        slowdown_limit = (int)eeprom::getEepromUInt32(eeprom::ACCEL_SLOWDOWN_LIMIT, EEPROM_DEFAULT_ACCEL_SLOWDOWN_LIMIT);
        if ( slowdown_limit > (BLOCK_BUFFER_SIZE / 2))	slowdown_limit = 0;

        if (eeprom::getEepromUInt32(eeprom::ACCEL_CLOCKWISE_EXTRUDER, EEPROM_DEFAULT_ACCEL_CLOCKWISE_EXTRUDER) == 0)	clockwise_extruder = false;
        else									clockwise_extruder = true;

        force_acceleration_off = false;

        if ( planner ) 	plannerMaxBufferSize = BLOCK_BUFFER_SIZE - 1;
        else		plannerMaxBufferSize = 1;

        plan_init(advanceK, advanceK2, noodleDiameter, FTOFP(axis_steps_per_unit[E_AXIS]), holdZ);	//Initialize planner
        st_init();											//Initialize stepper
        droppedSegmentsRelative[0] = 0;
        droppedSegmentsRelative[1] = 0;
        droppedSegmentsRelative[2] = 0;
        droppedSegmentsRelative[3] = 0;
        droppedSegmentsRelative[4] = 0;
        droppedSegmentsUs = 0;
        lastTarget = Point(st_get_position(X_AXIS), st_get_position(Y_AXIS), st_get_position(Z_AXIS), st_get_position(E_AXIS), 0);
    }
    else lastTarget = getPosition();
#endif
}

/// Define current position as given point
void definePosition(const Point& position)
{
#ifdef HAS_STEPPER_ACCELERATION
    if (( acceleration ) && ( ! force_acceleration_off ))
    {
        plan_set_position(position[0], position[1], position[2], position[3]);
    }
    else
    {
#endif
        for (int i = 0; i < STEPPER_COUNT; i++)
        {
            axes[i].definePosition(position[i]);
        }
#ifdef HAS_STEPPER_ACCELERATION
    }
    lastTarget = position;
#endif
}

/// Get position
/// When accelerated, this is the target position of the command at the end of the pipeline
const Point getPosition()
{
#if STEPPER_COUNT > 3
#ifdef HAS_STEPPER_ACCELERATION
    if (( acceleration ) && ( ! force_acceleration_off ))
        return lastTarget;
    else
#endif
#if STEPPER_COUNT > 4
        return Point(axes[0].position,axes[1].position,axes[2].position,axes[3].position,axes[4].position);
#else
        return Point(axes[0].position,axes[1].position,axes[2].position,axes[3].position, 0);
#endif
#else
#ifdef HAS_STEPPER_ACCELERATION
    if (( acceleration ) && ( ! force_acceleration_off ))
        return Point(lastTarget[0], lastTarget[1], lastTarget[2]);
    else
#endif
        return Point(axes[0].position,axes[1].position,axes[2].position);
#endif
}

/// Get current position
/// When accelerated, this is the position right now
/// advisable to call drainAccelerationBuffer before calling this
/// Only called by Control Panel

const Point getCurrentPosition()
{
#ifdef HAS_STEPPER_ACCELERATION
    if (( acceleration ) && ( ! force_acceleration_off ))
    {
        Point p;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            p = Point(st_get_position(X_AXIS), st_get_position(Y_AXIS), st_get_position(Z_AXIS), st_get_position(E_AXIS), 0);
        }
        return p;
    }
#endif
    return getPosition();
}

#ifdef HAS_STEPPER_ACCELERATION

//Drains the acceleration buffer down to empty before returning

void drainAccelerationBuffer(void)
{
    if (( acceleration ) && ( ! force_acceleration_off ))
    {
        while ( ! st_empty() ) _delay_us(10);
    }
}

#endif

void setHoldZ(bool holdZ_in)
{
    holdZ = holdZ_in;
}

#ifdef HAS_STEPPER_ACCELERATION

//Calculates the feedrate based on moving from "from" to "to"
FPTYPE calcFeedRate(const Point& from, const Point& to, int32_t interval, bool includeEAxis )
{
    //Calculate the distance in mm's by:
    //Calculate the delta distances and convert to mm's
    //Then sqr of sum of squares of the deltas for the distance
    bool override_master_steps = false;
    uint32_t planner_master_steps_cfr = 0;
    FPTYPE   planner_distance_cfr     = 0;

    for ( uint8_t i = 0; i <= Z_AXIS; i++ )
    {
        planner_steps[i] = to[i] - from[i];
        if ( planner_steps[i] != 0 )
        {
            delta_mm[i] = FPMULT2(ITOFP(planner_steps[i]), axis_steps_per_unit_inverse[i]);
            planner_steps[i] = labs(planner_steps[i]);
            planner_distance_cfr += FPSQUARE(delta_mm[i]);
            if ( (uint32_t)planner_steps[i] > planner_master_steps_cfr )
                planner_master_steps_cfr = (uint32_t)planner_steps[i];
        }
        else
            delta_mm[i] = 0;
    }

    if ( planner_master_steps_cfr == 0 )
    {
        includeEAxis          = true;
        override_master_steps = true;
    }

    planner_master_steps = planner_master_steps_cfr;
    planner_distance     = planner_distance_cfr;

    for ( uint8_t i = Z_AXIS+1; i < NUM_AXIS; i++ )
    {
        planner_steps[i] = to[i] - from[i];
        if ( planner_steps[i] != 0 )
        {
            delta_mm[i] = FPMULT2(ITOFP(planner_steps[i]), axis_steps_per_unit_inverse[i]);
            planner_steps[i] = labs(planner_steps[i]);
            planner_distance += FPSQUARE(delta_mm[i]);
            if ( (uint32_t)planner_steps[i] > planner_master_steps )
                planner_master_steps = (uint32_t)planner_steps[i];
        }
        else
            delta_mm[i] = 0;
    }

    if ( override_master_steps ) planner_master_steps_cfr = planner_master_steps;

    if ( includeEAxis )
    {
        planner_distance = FPSQRT(planner_distance);
        planner_distance_cfr = planner_distance;
    }
    else
    {
        planner_distance_cfr = FPSQRT(planner_distance_cfr);
        planner_distance     = FPSQRT(planner_distance);
    }

    // Now compute the feed rate
    planner_distance = FPSQRT(planner_distance_cfr);
    if ( (interval <= 0) || (MASTER_STEPS == 0) ) return 0;
    return ((planner_distance_cfr * 1000000.0) / ((FPTYPE)interval * (FPTYPE)MASTER_STEPS));
}

#endif

void setTarget(const Point& target, int32_t dda_interval)
{
#ifdef HAS_STEPPER_ACCELERATION
    if (( acceleration ) && ( force_acceleration_off ) && ( ! accelerationSwitchBackLockout ) && ( target[3] != lastTarget[3] )) switchToAcceleratedDriver();

    if (( acceleration ) && ( ! force_acceleration_off ))
    {
        FPTYPE feedRate = calcFeedRate(lastTarget, target, dda_interval, true);

        //Figure out the distance between x1,y1 and x2,y2 using pythagoras
        if ( plan_buffer_line(target[0], target[1], target[2], target[3], feedRate, 0, segmentAccelState) )
            lastTarget = target;
        if ( movesplanned() >=  plannerMaxBufferSize)	is_running = true;
        else						is_running = false;
    }
    else
    {
#endif
        int32_t max_delta = 0;
        for (int i = 0; i < AXIS_COUNT; i++)
        {
            axes[i].setTarget(target[i], false);
            const int32_t delta = axes[i].delta;
            // Only shut z axis on inactivity
            if (i == Z_AXIS && !holdZ) axes[i].enableStepper(delta != 0);
            else if (delta != 0) axes[i].enableStepper(true);
            if (delta > max_delta)
            {
                max_delta = delta;
            }
        }
        // compute number of intervals for this move
        intervals = ((max_delta * dda_interval) / INTERVAL_IN_MICROSECONDS);
        intervals_remaining = intervals;
        const int32_t negative_half_interval = -intervals / 2;
        for (int i = 0; i < AXIS_COUNT; i++)
        {
            axes[i].counter = negative_half_interval;
        }
        is_running = true;

#ifdef HAS_STEPPER_ACCELERATION
        lastTarget = target;

    }
#endif
}

void setTargetNew(const Point& target, int32_t us, uint8_t relative)
{
#ifdef HAS_STEPPER_ACCELERATION
    if (( acceleration ) && ( force_acceleration_off ) && ( ! accelerationSwitchBackLockout ) &&
            ((((relative & (1 << 3)) != 0) && ( target[3] != 0 )) ||
             (((relative & (1 << 3)) == 0) && ( target[3] != lastTarget[3]))))
        switchToAcceleratedDriver();

    if (( acceleration ) && ( ! force_acceleration_off ))
    {
        Point newPosition = target;
        for (uint8_t i = 0; i < AXIS_COUNT; i ++)
        {
            if ((relative & (1 << i)) != 0)
            {
                newPosition[i] = lastTarget[i] + target[i];
                newPosition[i] += droppedSegmentsRelative[i];
            }
        }
        us += droppedSegmentsUs;

        int32_t max_delta = 0;
        for (int i = 0; i < AXIS_COUNT; i++)
        {
            if (i == E_AXIS && max_delta != 0) continue;
            int32_t delta = newPosition[i] - lastTarget[i];
            if ( delta < 0 ) delta *= -1;
            if (delta > max_delta)
            {
                max_delta = delta;
            }
        }
        int32_t dda_interval = us / max_delta;
        FPTYPE feedRate = calcFeedRate(lastTarget, newPosition, dda_interval, false);

        if ( plan_buffer_line(newPosition[0], newPosition[1], newPosition[2], newPosition[3], feedRate, 0, segmentAccelState) )
        {
            lastTarget = newPosition;
            droppedSegmentsRelative = Point(0,0,0,0,0);
            droppedSegmentsUs = 0;
        }
        else
        {
            //All information is stored in "relative" co-ords
            for (uint8_t i = 0; i < AXIS_COUNT; i ++)
            {
                if ((relative & (1 << i)) != 0)
                    droppedSegmentsRelative[i] += target[i];
                else	droppedSegmentsRelative[i]  = target[i] - lastTarget[i];
            }
            droppedSegmentsUs += us;
        }
        if ( movesplanned() >=  plannerMaxBufferSize)	is_running = true;
        else						is_running = false;
    }
    else
    {
#endif
        for (int i = 0; i < AXIS_COUNT; i++)
        {
            axes[i].setTarget(target[i], (relative & (1 << i)) != 0);
            // Only shut z axis on inactivity
            const int32_t delta = axes[i].delta;
            if (i == Z_AXIS && !holdZ)
            {
                axes[i].enableStepper(delta != 0);
            }
            else if (delta != 0)
            {
                axes[i].enableStepper(true);
            }

#ifdef HAS_STEPPER_ACCELERATION
            lastTarget[i] = axes[i].absoluteTarget;
#endif
        }
        // compute number of intervals for this move
        intervals = us / INTERVAL_IN_MICROSECONDS;
        intervals_remaining = intervals;
        const int32_t negative_half_interval = -intervals / 2;
        for (int i = 0; i < AXIS_COUNT; i++)
        {
            axes[i].counter = negative_half_interval;
        }
        is_running = true;

#ifdef HAS_STEPPER_ACCELERATION
    }
#endif
}

#ifdef HAS_STEPPER_ACCELERATION
void switchToRegularDriver(bool lockout)
{

#ifdef STORE_RAM_USAGE_TO_EEPROM
    storeStackFreeToEeprom();
#endif

    accelerationSwitchBackLockout = lockout;
    if (( ! acceleration ) || ( acceleration && force_acceleration_off ))	return;

    //Wait for the buffer to empty
    drainAccelerationBuffer();

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        is_running = false;
        Point currentPosition = getPosition();
        for (int i = 0; i < STEPPER_COUNT; i++)
        {
            axes[i].definePosition(currentPosition[i]);
        }

        force_acceleration_off = true;

        //Change the interrupt frequency to the regular driver
        Motherboard::getBoard().setupFixedStepperTimer();
    }
}

void switchToAcceleratedDriver()
{
    accelerationSwitchBackLockout = false;

    if (( ! acceleration ) || ( acceleration && ( ! force_acceleration_off )))	return;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        force_acceleration_off = false;
        if ( ! strangled ) setSegmentAccelState(true);

        Point currentPosition = Point(axes[0].position, axes[1].position, axes[2].position, axes[3].position, axes[4].position);
        definePosition(currentPosition);

        //Change the interrupt frequency to the accelerated driver
        Motherboard::getBoard().setupAccelStepperTimer();
    }
}

bool isAccelerated(void)
{
    if ( acceleration && ( ! force_acceleration_off ))	return true;
    return false;
}

#endif

/// Start homing
void startHoming(const bool maximums, const uint8_t axes_enabled, const uint32_t us_per_step)
{
#ifdef   SMALL_4K_RAM
    uint16_t sc = StackCount();
#if defined (__AVR_ATmega2560__)
    sc -= 4096;	//Take of 4K to simulate 644p
#endif
    if ( sc < MINIMUM_FREE_SRAM)
        Motherboard::getBoard().indicateError(ERR_NO_FREE_SRAM);
#endif
#ifdef HAS_STEPPER_ACCELERATION
    if (( acceleration ) && (! force_acceleration_off )) switchToRegularDriver(false);
#endif

    intervals_remaining = INT32_MAX;
    intervals = us_per_step / INTERVAL_IN_MICROSECONDS;
    const int32_t negative_half_interval = -intervals / 2;
    for (int i = 0; i < AXIS_COUNT; i++)
    {
        axes[i].counter = negative_half_interval;
        if ((axes_enabled & (1<<i)) != 0)
        {
            axes[i].setHoming(maximums);
        }
        else
        {
            axes[i].delta = 0;
        }
    }
    is_homing = true;
}

/// Enable/disable the given axis.
void enableAxis(uint8_t index, bool enable)
{
    if (index < STEPPER_COUNT)
    {
        axes[index].enableStepper(enable);
    }
}

/// Report if the given axis is enabled or disabled
bool isEnabledAxis(uint8_t index)
{
    if (index < STEPPER_COUNT)
    {
        axes[index].isEnabledStepper();
    }
}

bool isAtMaximum(uint8_t index)
{
    if (index < STEPPER_COUNT)
    {
        return axes[index].isAtMaximum();
    }
    return false;
}


bool isAtMinimum(uint8_t index)
{
    if (index < STEPPER_COUNT)
    {
        return axes[index].isAtMinimum();
    }
    return false;
}

#ifdef DEBUG_ZADVANCE

void doLcd()
{
    Motherboard::getBoard().lcd.setCursor(0,0);
    Motherboard::getBoard().lcd.writeFloat((float)zadvance,3);
    Motherboard::getBoard().lcd.write(' ');
    Motherboard::getBoard().lcd.setCursor(0,3);
    Motherboard::getBoard().lcd.writeFloat((float)zadvance2,3);
    Motherboard::getBoard().lcd.write(' ');
}

#endif


void runSteppersSlice()
{
    //Set Foo and Bar pins for acceleration information
#ifdef HAS_STEPPER_ACCELERATION
    if (( acceleration ) && ( ! force_acceleration_off ))
    {
#ifdef INTERFACE_BAR_PIN
        INTERFACE_BAR_PIN.setValue(true);
#endif

        uint8_t bufferUsed = movesplanned();

#ifdef INTERFACE_FOO_PIN
        if ( bufferUsed < 3 ) INTERFACE_FOO_PIN.setValue(false);
        else		      INTERFACE_FOO_PIN.setValue(true);
#endif
    }
    else
    {
#endif
#ifdef INTERFACE_BAR_PIN
        INTERFACE_BAR_PIN.setValue(false);
#endif
#ifdef HAS_STEPPER_ACCELERATION
    }
#endif

    //Set the stepper interrupt timer
#if defined(DEBUG_ZADVANCE) && defined(TIME_STEPPER_INTERRUPT)
    zadvance2 = debugTimer;
#endif
}


void doInterrupt()
{
#if defined(DEBUG_ZADVANCE) && defined(TIME_STEPPER_INTERRUPT)
    DEBUG_TIMER_START;
#endif

#ifdef HAS_STEPPER_ACCELERATION
    if (( acceleration ) && ( ! force_acceleration_off ))
    {
        if ( st_interrupt() ) is_running = false;
    }
    else
    {
#endif
        if (is_running)
        {
            if (intervals_remaining-- == 0)
            {
                is_running = false;
            }
            else
            {
                for (int i = 0; i < STEPPER_COUNT; i++)
                {
                    axes[i].doInterrupt(intervals);
                }
            }
        }
        else if (is_homing)
        {
            is_homing = false;
            for (int i = 0; i < STEPPER_COUNT; i++)
            {
                bool still_homing = axes[i].doHoming(intervals);
                is_homing = still_homing || is_homing;
            }

#ifdef HAS_STEPPER_ACCELERATION
            if (( ! is_homing ) && ( acceleration ) && ( force_acceleration_off )) switchToAcceleratedDriver();
#endif
        }
#ifdef HAS_STEPPER_ACCELERATION
    }
#endif

#if defined(DEBUG_ZADVANCE) && defined(TIME_STEPPER_INTERRUPT)
    DEBUG_TIMER_FINISH;
    debugTimer = DEBUG_TIMER_TCTIMER_USI;
#endif
}

#ifdef HAS_STEPPER_ACCELERATION

bool doAdvanceInterrupt()
{
    if ( acceleration ) st_advance_interrupt();
}

void setSegmentAccelState(bool state)
{
    segmentAccelState = state;
}

#endif

}
