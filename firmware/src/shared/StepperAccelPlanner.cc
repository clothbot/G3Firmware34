/*
  StepperAccelPlanner.cc - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  This software uses the following components from Jetty Firmware:
  JKN Advance
  YAJ (Yet Another Jerk)
  Advance Pressure Relax
  authored by Dan Newman and Jetty.
*/

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

/*
    Reasoning behind the mathematics in this module (in the key of 'Mathematica'):

    s == speed, a == acceleration, t == time, d == distance

    Basic definitions:

    Speed[s_, a_, t_] := s + (a*t)
    Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]

    Distance to reach a specific speed with a constant acceleration:

    Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
    d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()

    Speed after a given distance of travel with constant acceleration:

    Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
    m -> Sqrt[2 a d + s^2]

    DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]

    When to start braking (di) to reach a specified destionation speed (s2) after accelerating
    from initial speed s1 without ever stopping at a plateau:

    Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
    di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()

    IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
*/

#ifdef SIMULATOR
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "Simulator.hh"
#endif

#include "Configuration.hh"

#ifdef HAS_STEPPER_ACCELERATION


#include "StepperAccelPlanner.hh"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#ifndef SIMULATOR
#include  <avr/interrupt.h>
#endif
#include "StepperAccel.hh"
#ifndef SIMULATOR
#include "StepperInterface.hh"
#include "Motherboard.hh"
#endif

#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))

#define VEPSILON 1.0e-5

// v1 != v2
#ifdef FIXED
#define VNEQ(v1,v2) ((v1) != (v2))
#define VLT(v1,v2)  ((v1) < (v2))
#else
#define VNEQ(v1,v2) (abs((v1)-(v2)) > VEPSILON)
#define VLT(v1,v2)  (((v1) + VEPSILON) < (v2))
#endif

//===========================================================================
//=============================public variables ============================
//===========================================================================

uint32_t minsegmenttime;
FPTYPE max_feedrate[NUM_AXIS]; // set the max speeds
uint32_t max_acceleration_units_per_sq_second[NUM_AXIS]; // Use M201 to override by software
FPTYPE minimumfeedrate;
FPTYPE p_acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
FPTYPE p_retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
FPTYPE smallest_max_speed_change;
FPTYPE max_speed_change[NUM_AXIS]; //The speed between junctions in the planner, reduces blobbing
FPTYPE mintravelfeedrate;
FPTYPE minimumPlannerSpeed;
FPTYPE extruder_only_max_feedrate;
int slowdown_limit;
float axis_steps_per_unit[NUM_AXIS];

bool disable_slowdown = true;
uint32_t axis_steps_per_sqr_second[NUM_AXIS];
FPTYPE extrution_area, steps_per_cubic_mm_e;
FPTYPE extruder_advance_k = 0, extruder_advance_k2 = 0;
FPTYPE delta_mm[NUM_AXIS];
FPTYPE planner_distance;
uint32_t planner_master_steps;
int32_t planner_steps[NUM_AXIS];
FPTYPE vmax_junction;
FPTYPE cosine;

// minimum time in seconds that a movement needs to take if the buffer is emptied.
// Increase this number if you see blobs while printing high speed & high detail.
// It will slowdown on the detailed stuff.
// Comment out to disable
FPTYPE minimumSegmentTime;

// The current position of the tool in absolute steps
int32_t position[NUM_AXIS];   //rescaled from extern when axis_steps_per_unit are changed by gcode

static FPTYPE prev_speed[NUM_AXIS];
static FPTYPE previous_inverse_millimeters;

#ifndef SIMULATOR
static StepperInterface *stepperInterface;
#else
static block_t *sblock = NULL;
static char prior_dropped[1024] = "\0";
#endif
bool acceleration_zhold = false;

#ifdef DEBUG_BLOCK_BY_MOVE_INDEX
uint32_t current_move_index = 0;
#endif

//===========================================================================
//=================semi-private variables, used in inline  functions    =====
//===========================================================================
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now


// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
#ifndef SAVE_SPACE
FORCE_INLINE
#endif
static uint8_t next_block_index(uint8_t block_index)
{
    block_index++;
    if (block_index == BLOCK_BUFFER_SIZE)
    {
        block_index = 0;
    }
    return(block_index);
}


// Returns the index of the previous block in the ring buffer
#ifndef SAVE_SPACE
FORCE_INLINE
#endif
static uint8_t prev_block_index(uint8_t block_index)
{
    if (block_index == 0)
    {
        block_index = BLOCK_BUFFER_SIZE;
    }
    block_index--;
    return(block_index);
}

//===========================================================================
//=============================functions         ============================
//===========================================================================

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
// given acceleration:
FORCE_INLINE int32_t estimate_acceleration_distance(int32_t initial_rate_sq, int32_t target_rate_sq, int32_t acceleration_doubled)
{
    if (acceleration_doubled!=0)
    {
        return((target_rate_sq-initial_rate_sq)/acceleration_doubled);
    }
    else
    {
        return 0;  // acceleration was 0, set acceleration distance to 0
    }
}

FORCE_INLINE int32_t intersection_distance(int32_t initial_rate_sq, int32_t final_rate_sq, int32_t acceleration_doubled, int32_t distance)
{
    if (acceleration_doubled!=0)
    {
        return((acceleration_doubled*distance-initial_rate_sq+final_rate_sq)/(acceleration_doubled << 1) );
    }
    else
    {
        return 0;  // acceleration was 0, set intersection distance to 0
    }
}


//Same as final_speed, except this one works with step_rates.
//Regular final_speed will overflow if we use step_rates instead of mm/s

FORCE_INLINE FPTYPE final_speed_step_rate(uint32_t acceleration, uint32_t initial_velocity, int32_t distance)
{
    uint32_t v2 = initial_velocity * initial_velocity;
    //Although it's highly unlikely, if target_rate < initial_rate, then distance could be negative.
    if ( distance < 0 )
    {
        uint32_t term2 = (acceleration * (uint32_t)abs(distance)) << 1;

        if ( term2 >= v2 )	return 0;
        v2 -= term2;
    }
    else	v2 += (acceleration * (uint32_t)distance) << 1;
    return FPSQRT(v2);
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t *block, FPTYPE entry_factor, FPTYPE exit_factor)
{

    if ( (!block->use_accel) || (entry_factor > KCONSTANT_1) ) entry_factor = KCONSTANT_1;
    if ( (!block->use_accel) || (exit_factor > KCONSTANT_1) ) exit_factor = KCONSTANT_1;

#ifdef FIXED
#ifdef NO_CEIL_FLOOR
    uint32_t initial_rate = (uint32_t)FPTOI(KCONSTANT_0_5 + FPMULT2(ITOFP(block->nominal_rate), entry_factor)); // (step/min)
    uint32_t final_rate = (uint32_t)FPTOI(KCONSTANT_0_5 + FPMULT2(ITOFP(block->nominal_rate), exit_factor)); // (step/min)
#else
    uint32_t initial_rate = (uint32_t)FPTOI(FPCEIL(FPMULT2(ITOFP(block->nominal_rate), entry_factor))); // (step/min)
    uint32_t final_rate = (uint32_t)FPTOI(FPCEIL(FPMULT2(ITOFP(block->nominal_rate), exit_factor))); // (step/min)
#endif
#else
#ifdef NO_CEIL_FLOOR
    uint32_t initial_rate = (uint32_t)(0.5 + block->nominal_rate*entry_factor); // (step/min)
    uint32_t final_rate = (uint32_t)(0.5 + block->nominal_rate*exit_factor); // (step/min)
#else
    uint32_t initial_rate = FPCEIL(block->nominal_rate*entry_factor); // (step/min)
    uint32_t final_rate = FPCEIL(block->nominal_rate*exit_factor); // (step/min)
#endif
#endif
    // Limit minimal step rate (Otherwise the timer will overflow.)
    if(initial_rate <120)
    {
        initial_rate=120;
    }
    if(final_rate < 120)
    {
        final_rate=120;
    }

    int32_t initial_rate_sq = (int32_t)(initial_rate * initial_rate);
    int32_t nominal_rate_sq = (int32_t)(block->nominal_rate * block->nominal_rate);
    int32_t final_rate_sq   = (int32_t)(final_rate   * final_rate);

    int32_t acceleration = block->acceleration_st;
    int32_t acceleration_doubled = acceleration << 1;
    int32_t accelerate_steps = 0;
    int32_t decelerate_steps = 0;
    if ( block->use_accel )
    {
        accelerate_steps = estimate_acceleration_distance(initial_rate_sq, nominal_rate_sq, acceleration_doubled);
        decelerate_steps = estimate_acceleration_distance(nominal_rate_sq, final_rate_sq, -acceleration_doubled);
    }

    // accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
    // accelerate_steps = min(accelerate_steps,(int32_t)block->step_event_count);

    // Calculate the size of Plateau of Nominal Rate.
    int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;

    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_steps < 0)
    {
        accelerate_steps = intersection_distance(initial_rate_sq, final_rate_sq, acceleration_doubled, (int32_t)block->step_event_count);
        accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
        accelerate_steps = min(accelerate_steps,(int32_t)block->step_event_count);
        plateau_steps = 0;
    }
    int32_t decelerate_after = accelerate_steps + plateau_steps;

#ifdef SIMULATOR
    sblock = block;
#endif
    int32_t advance_lead_entry = 0, advance_lead_exit = 0, advance_lead_prime = 0, advance_lead_deprime = 0;
    int32_t advance_pressure_relax = 0;

    if ( block->use_advance_lead )
    {
        uint32_t maximum_rate;
        if ( plateau_steps == 0 ) maximum_rate = FPTOI(final_speed_step_rate(block->acceleration_st, initial_rate,
                    accelerate_steps + 1));
        else maximum_rate = block->nominal_rate;
        if (accelerate_steps > 0)
        {
#ifdef JKN_ADVANCE_LEAD_ACCEL
            advance_lead_entry = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)block->acceleration_st>>4)));
#else
            advance_lead_entry = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)(maximum_rate - initial_rate))));
#endif

#ifdef JKN_ADVANCE_LEAD_DE_PRIME
            advance_lead_prime = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)initial_rate)) - KCONSTANT_0_5);
#endif

#ifndef SIMULATOR
            if (advance_lead_entry < 0) advance_lead_entry = 0;
            if (advance_lead_prime < 0) advance_lead_prime = 0;
#endif
        }
        if ((decelerate_after+1) < (int32_t)block->step_event_count)
        {
#ifdef JKN_ADVANCE_LEAD_ACCEL
            advance_lead_exit = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)block->acceleration_st>>4)));
#else
            advance_lead_exit = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)(maximum_rate - final_rate))));
#endif

#ifdef JKN_ADVANCE_LEAD_DE_PRIME
            advance_lead_deprime = FPTOI(FPMULT2(extruder_advance_k, ITOFP((int32_t)final_rate)) - KCONSTANT_0_5);
#endif
            if ( extruder_advance_k2 != 0 )
            {
                advance_pressure_relax = FPTOI(FPDIV(FPMULT3(extruder_advance_k2, ITOFP((int32_t)block->acceleration_st >> 4), KCONSTANT_100), ITOFP((int32_t)decelerate_steps)));
            }

#ifndef SIMULATOR
            //If we've overflowed, reset to 0
            if ( advance_pressure_relax < 0 ) advance_pressure_relax = 0;

            if (advance_lead_exit    < 0) advance_lead_exit = 0;
            if (advance_lead_deprime < 0) advance_lead_deprime = 0;
#endif
        }
#ifdef SIMULATOR
        if ( (advance_lead_entry < 0) || (advance_lead_exit < 0) || (advance_pressure_relax < 0) )
        {
            char buf[1024];
            snprintf(buf, sizeof(buf),
                     "*** calculate_trapezoid_for_block(): advance_lead_entry=%d, advance_lead_exit=%d, "
                     "advance_pressure_relax=%d; initial/nominal/maximum/final_rate=%d/%d/%d/%d; "
                     "accelerate_until/decelerate_after/step_events/plateau_steps=%d/%d/%d/%d; "
                     "i/n/f/a=%d/%d/%d/%d\n",
                     advance_lead_entry, advance_lead_exit, advance_pressure_relax,initial_rate, block->nominal_rate,
                     maximum_rate, final_rate, accelerate_steps, decelerate_after, block->step_event_count,
                     plateau_steps, initial_rate_sq, nominal_rate_sq, final_rate_sq, acceleration_doubled);
            if (block->message[0] != '\0')
                strlcat(block->message, "\n", sizeof(block->message));
            strlcat(block->message, buf, sizeof(block->message));
        }
#endif
    }

    CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
    if(block->busy == false)   // Don't update variables if block is busy.
    {
        block->accelerate_until = accelerate_steps;
        block->decelerate_after = decelerate_after;
        block->initial_rate = initial_rate;
        block->final_rate = final_rate;
        block->advance_lead_entry     = advance_lead_entry;
        block->advance_lead_exit      = advance_lead_exit;
        block->advance_lead_prime     = advance_lead_prime;
        block->advance_lead_deprime   = advance_lead_deprime;
        block->advance_pressure_relax = advance_pressure_relax;
    }
    CRITICAL_SECTION_END;
#ifdef SIMULATOR
    block->planned += 1;
#endif
}

#ifndef FIXSIGN

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
FORCE_INLINE FPTYPE max_allowable_speed(FPTYPE acceleration, FPTYPE target_velocity, FPTYPE distance)
{
#ifdef FIXED
    FPTYPE v2 = FPSQUARE(target_velocity) - FPMULT3(KCONSTANT_2, acceleration, distance);
#else
    FPTYPE v2 = FPSQUARE(target_velocity) - 2.0 * acceleration * distance;
#endif
    if (v2 < 0) return (0);
    else	return FPSQRT(v2);
}

#else

FORCE_INLINE FPTYPE initial_speed(FPTYPE acceleration, FPTYPE target_velocity, FPTYPE distance)
{
    FPTYPE v2 = FPSQUARE(target_velocity) - FPMULT3(KCONSTANT_2, acceleration, distance);
    if (v2 <= 0) return 0;
    else return FPSQRT(v2);
}

FORCE_INLINE FPTYPE final_speed(FPTYPE acceleration, FPTYPE initial_velocity, FPTYPE distance)
{
    FPTYPE v2 = FPSQUARE(initial_velocity) + FPMULT3(KCONSTANT_2, acceleration, distance);
    if (v2 <= 0) return 0;
    else return FPSQRT(v2);
}

#endif

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *current, block_t *next)
{
    if(!current)
    {
        return;
    }

    if (next)
    {
        // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
        // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
        // check for maximum allowable speed reductions to ensure maximum possible planned speed.
        if (VNEQ(current->entry_speed, current->max_entry_speed))
        {

            // If nominal length true, max junction speed is guaranteed to be reached. Only compute
            // for max allowable speed if block is decelerating and nominal length is false.
            if ((!current->nominal_length_flag) && next->speed_changed && (current->max_entry_speed > next->entry_speed))
            {
#ifndef FIXSIGN
                current->entry_speed = min( current->max_entry_speed,
                                            max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
#else
                // We want to know what speed to start at so that if we decelerate -- negative acceleration --
                // over distance current->millimeters, we end up at speed next->entry_speed

                // This call produces the same result as !defined(FIXSIGN) case
#ifdef SIMULATOR
                sblock = current;
#endif
                current->entry_speed = min( current->max_entry_speed,
                                            initial_speed(-current->acceleration,next->entry_speed,current->millimeters));
#endif
            }
            else
            {
                current->entry_speed = current->max_entry_speed;
                current->speed_changed = true;
            }
            current->recalculate_flag = true;

        }
    } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the reverse pass.
void planner_reverse_pass()
{
    uint8_t block_index = block_buffer_head;
    block_t *block[2] = { NULL, NULL};

    //Make a local copy of block_buffer_tail, because the interrupt can alter it
    CRITICAL_SECTION_START;
    unsigned char tail = block_buffer_tail;
    CRITICAL_SECTION_END;

    while(block_index != tail)
    {
        block_index = prev_block_index(block_index);
        block[1]= block[0];
        block[0] = &block_buffer[block_index];
        planner_reverse_pass_kernel(block[0], block[1]);
    }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current)
{
    if(!previous || !current->use_accel)
    {
        return;
    }

    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
    if (!previous->nominal_length_flag && previous->speed_changed)
    {
        if (VLT(previous->entry_speed, current->entry_speed))
        {
#ifndef FIXSIGN
            FPTYPE entry_speed = min( current->entry_speed,
                                      max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );
#else
            // We want to know what the terminal speed from the prior block would be if
            // it accelerated over the entire block with starting speed prev->entry_speed

            // This call produces the same result as !defined(FIXSIGN) case
#ifdef SIMULATOR
            sblock = previous;
#endif
            FPTYPE entry_speed = min( current->entry_speed,
                                      final_speed(previous->acceleration,previous->entry_speed,previous->millimeters) );
#endif

            // Check for junction speed change
            if (VNEQ(current->entry_speed, entry_speed))
            {
                current->entry_speed = entry_speed;
                current->recalculate_flag = true;
                current->speed_changed = true;
            }
        }
    }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the forward pass.
void planner_forward_pass()
{
    uint8_t block_index = block_buffer_tail;
    block_t *block[2] = { NULL, NULL };

    while(block_index != block_buffer_head)
    {
        block[0] = block[1];
        block[1] = &block_buffer[block_index];
        planner_forward_pass_kernel(block[0],block[1]);
        block_index = next_block_index(block_index);
    }
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the
// entry_factor for each junction. Must be called by planner_recalculate() after
// updating the blocks.
void planner_recalculate_trapezoids()
{
    uint8_t block_index = block_buffer_tail;
    block_t *current;
    block_t *next = NULL;

    while(block_index != block_buffer_head)
    {
        current = next;
        next = &block_buffer[block_index];
        if (current)
        {
            // Recalculate if current block entry or exit junction speed has changed.
            if (current->recalculate_flag || next->recalculate_flag)
            {
                // NOTE: Entry and exit factors always > 0 by all previous logic operations.
                calculate_trapezoid_for_block(current, FPDIV(current->entry_speed,current->nominal_speed),
                                              FPDIV(next->entry_speed,current->nominal_speed));
                current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
            }
        }
        block_index = next_block_index( block_index );
    }
    // Last/newest block in buffer. Exit speed is set with minimumPlannerSpeed. Always recalculated.
    if(next != NULL)
    {
        FPTYPE scaling = FPDIV(next->entry_speed,next->nominal_speed);
        calculate_trapezoid_for_block(next, scaling, scaling);
        // calculate_trapezoid_for_block(next,
        //    FPDIV(next->entry_speed,next->nominal_speed),
        //    FPDIV(minimumPlannerSpeed,next->nominal_speed));
        next->recalculate_flag = false;
    }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor)
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if
//     a. The speed increase within one block would require faster accelleration than the one, true
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate()
{
    planner_reverse_pass();
    planner_forward_pass();
    planner_recalculate_trapezoids();
}


void plan_init(FPTYPE extruderAdvanceK, FPTYPE extruderAdvanceK2, FPTYPE noodleDiameter, FPTYPE axis_steps_per_unit_e, bool zhold)
{
#ifndef SIMULATOR
    stepperInterface = Motherboard::getBoard().getStepperAllInterfaces();
#else
    if ( (E_AXIS+1) != NUM_AXIS ) abort();
    if ( (X_AXIS >= NUM_AXIS) ||
            (Y_AXIS >= NUM_AXIS) ||
            (Z_AXIS >= NUM_AXIS) ||
            (E_AXIS >= NUM_AXIS) ) abort();
#endif
    block_buffer_head = 0;
    block_buffer_tail = 0;
    memset(position, 0, sizeof(position)); // clear position
    previous_inverse_millimeters = 0;

    extruder_advance_k  = extruderAdvanceK;
    extruder_advance_k2 = extruderAdvanceK2;
#ifdef FIXED
    extrution_area = FPMULT3(KCONSTANT_0_25, FPSQUARE(noodleDiameter), PIk);
#else
    extrution_area = 0.25 * FPSQUARE(noodleDiameter) * M_PI;
#endif
    steps_per_cubic_mm_e = FPDIV(axis_steps_per_unit_e, extrution_area);
    acceleration_zhold = zhold;

#ifdef DEBUG_BLOCK_BY_MOVE_INDEX
    current_move_index = 0;
#endif
}



// Add a new linear movement to the buffer. steps x, y and z is the absolute position in
// steps. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
// Returns true if command was executed, otherwise false of the command was dopped (due to <dropsegments)
bool plan_buffer_line(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e,  FPTYPE feed_rate, const uint8_t &extruder, bool use_accel)
{
    if ( slowdown_limit && block_buffer_head == block_buffer_tail ) disable_slowdown = true;

    // Calculate the buffer head after we push this byte
    uint8_t next_buffer_head = next_block_index(block_buffer_head);

    int32_t target[NUM_AXIS];
    target[X_AXIS] = x;
    target[Y_AXIS] = y;
    target[Z_AXIS] = z;
    target[E_AXIS] = e;

    // Prepare to set up new block
    block_t *block = &block_buffer[block_buffer_head];
    block->use_accel = use_accel;

    // Mark block as not busy (Not executed by the stepper interrupt)
    block->busy = false;

#ifndef CMD_SET_POSITION_CAUSES_DRAIN
    memcpy(block->starting_position, position, sizeof(block->starting_position)); // starting_position[] = position[]
#endif

#ifdef DEBUG_BLOCK_BY_MOVE_INDEX
    block->move_index = ++ current_move_index;
#endif

#ifdef SIMULATOR
    block->planned = 0;
    block->message[0] = '\0';
    sblock = NULL;
    if (prior_dropped[0] != '\0')
    {
        strlcat(block->message, prior_dropped, sizeof(block->message));
        prior_dropped[0] = '\0';
    }
#endif

    // Number of steps for each axis
    block->steps[X_AXIS] = planner_steps[X_AXIS];
    block->steps[Y_AXIS] = planner_steps[Y_AXIS];
    block->steps[Z_AXIS] = planner_steps[Z_AXIS];
    block->steps[E_AXIS] = planner_steps[E_AXIS];
    block->step_event_count = planner_master_steps;

    //Figure out which is the master axis
    for ( uint8_t i = 0; i < NUM_AXIS; i ++ )
    {
        if ( (uint32_t)labs(block->steps[i]) == block->step_event_count )
            block->dda_master_axis_index = i;
    }

    // Bail if this is a zero-length block
    if (block->step_event_count <=dropsegments)
#ifndef SIMULATOR
        return false;
#else
    {
        snprintf(prior_dropped, sizeof(prior_dropped),
                 "*** Segment prior to this one was dropped: x/y/z/e steps = "
                 "%d/%d/%d/%d, distance=%f",
                 planner_steps[X_AXIS],
                 planner_steps[Y_AXIS],
                 planner_steps[Z_AXIS],
                 planner_steps[E_AXIS],
                 FPTOF(planner_distance));
        return false;
    }
#endif

    bool extruder_only_move = false;
    if ( block->steps[X_AXIS] == 0  &&  block->steps[Y_AXIS] == 0  &&  block->steps[Z_AXIS] == 0  &&  block->steps[E_AXIS] != 0 )
        extruder_only_move = true;

    if (block->steps[E_AXIS] == 0)
    {
        if(feed_rate<mintravelfeedrate) feed_rate=mintravelfeedrate;
    }
    else
    {
        if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
    }

#ifdef SIMULATOR
    // Save the original feed rate prior to modification by limits
    block->feed_rate = feed_rate;
#endif

    // Compute direction bits for this block
    block->direction_bits = 0;
    if (target[X_AXIS] < position[X_AXIS])
    {
        block->direction_bits |= (1<<X_AXIS);
    }
    if (target[Y_AXIS] < position[Y_AXIS])
    {
        block->direction_bits |= (1<<Y_AXIS);
    }
    if (target[Z_AXIS] < position[Z_AXIS])
    {
        block->direction_bits |= (1<<Z_AXIS);
    }
    if (target[E_AXIS] < position[E_AXIS])
    {
        block->direction_bits |= (1<<E_AXIS);
    }

    block->active_extruder = extruder;

#ifndef SIMULATOR
    //enable active axes
    if(block->steps[X_AXIS] != 0) stepperInterface[X_AXIS].setEnabled(true);
    if(block->steps[Y_AXIS] != 0) stepperInterface[Y_AXIS].setEnabled(true);
    if(block->steps[Z_AXIS] != 0) stepperInterface[Z_AXIS].setEnabled(true);

    // Enable all
    if(block->steps[E_AXIS] != 0)
    {
        stepperInterface[E_AXIS].setEnabled(true);
    }
#endif

    int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
#ifdef DEBUG_ZADVANCE
    //  if ( moves_queued < 2 )	zadvance += 1.0;
#endif
    if ( moves_queued == 0 )
    {
        memset(prev_speed, 0, sizeof(prev_speed));
        previous_inverse_millimeters = 0;
    }

// SLOWDOWN
    if ( slowdown_limit )
    {
        if (( disable_slowdown ) && ( moves_queued >= slowdown_limit ))	disable_slowdown = false;
        if ( moves_queued < slowdown_limit && (! disable_slowdown ) && moves_queued > 1)
            feed_rate = FPDIV( FPMULT2(feed_rate, ITOFP(moves_queued)), ITOFP((int32_t)slowdown_limit));
    }
// END SLOWDOWN

    if ( extruder_only_move )
    {
        block->millimeters = FPABS(delta_mm[E_AXIS]);
    }
    else
    {
        block->millimeters = planner_distance;
        if ((moves_queued < 1 ) && (minimumSegmentTime > 0) && ( block->millimeters > 0 ) &&
                ( feed_rate > 0 ) && (( FPDIV(block->millimeters, feed_rate) ) < minimumSegmentTime))
        {
            feed_rate = FPDIV(block->millimeters, minimumSegmentTime);
        }
    }
#ifdef FIXED
    FPTYPE inverse_millimeters = FPDIV(KCONSTANT_1, block->millimeters);  // Inverse millimeters to remove multiple divides
#else
    FPTYPE inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides
#endif
    // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
    FPTYPE inverse_second = FPMULT2(feed_rate, inverse_millimeters);

#ifdef NO_CEIL_FLOOR
    block->nominal_speed = feed_rate; // (mm/sec) Always > 0
#ifdef FIXED
    block->nominal_rate = (uint32_t)FPTOI((KCONSTANT_0_5 + FPMULT2(ITOFP((int32_t)block->step_event_count), inverse_second))); // (step/sec) Always > 0
#else
    block->nominal_rate = (uint32_t)(0.5 + block->step_event_count * inverse_second); // (step/sec) Always > 0
#endif
#else
    block->nominal_speed = FPMULT2(block->millimeters, inverse_second); // (mm/sec) Always > 0
    block->nominal_rate = (uint32_t)FPTOI(FPCEIL(FPMULT2(ITOFP((int32_t)block->step_event_count), inverse_second))); // (step/sec) Always > 0
#endif

    FPTYPE current_speed[NUM_AXIS];
    for(unsigned char i=0; i < NUM_AXIS; i++)
    {
        current_speed[i] = FPMULT2(delta_mm[i], inverse_second);
    }

    // Limit speed per axis
#ifdef FIXED
    FPTYPE speed_factor = KCONSTANT_1; //factor <=1 do decrease speed
#else
    FPTYPE speed_factor = 1.0; //factor <=1 do decrease speed
#endif
    if ( use_accel )
    {
        if ( !extruder_only_move )
        {
            for(unsigned char i=0; i < NUM_AXIS; i++)
                if(FPABS(current_speed[i]) > max_feedrate[i])
                    speed_factor = min(speed_factor, FPDIV(max_feedrate[i], FPABS(current_speed[i])));
        }
        else if(FPABS(current_speed[E_AXIS]) > extruder_only_max_feedrate)
            speed_factor = FPDIV(extruder_only_max_feedrate, FPABS(current_speed[E_AXIS]));
    }
    else
    {
        for(unsigned char i=0; i < NUM_AXIS; i++)
            if(FPABS(current_speed[i]) > max_speed_change[i])
                speed_factor = min(speed_factor, FPDIV(max_speed_change[i], FPABS(current_speed[i])));
    }

    // Correct the speed
#ifdef FIXED
    if( speed_factor < KCONSTANT_1 )
    {
#else
    if( speed_factor < 1.0)
    {
#endif
        for(unsigned char i=0; i < NUM_AXIS; i++)
        {
            current_speed[i] = FPMULT2(current_speed[i], speed_factor);
        }
        block->nominal_speed = FPMULT2(block->nominal_speed, speed_factor);
        block->nominal_rate = (uint32_t)FPTOI(FPMULT2(ITOFP((int32_t)block->nominal_rate), speed_factor));
    }

    // Compute and limit the acceleration rate for the trapezoid generator.
    FPTYPE steps_per_mm = FPDIV(ITOFP((int32_t)block->step_event_count), block->millimeters);
    if ( extruder_only_move )
    {
#ifdef NO_CEIL_FLOOR
#ifdef FIXED
        block->acceleration_st = (uint32_t)(FPTOI(p_retract_acceleration) * FPTOI(steps_per_mm)); // convert to: acceleration steps/sec^2
#else
        block->acceleration_st = (uint32_t)(0.5 + p_retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
#endif
#else
        block->acceleration_st = (uint32_t)(FPTOI(FPCEIL(FPMULT2(p_retract_acceleration, steps_per_mm)))); // convert to: acceleration steps/sec^2
#endif
    }
    else
    {
#ifdef NO_CEIL_FLOOR
#ifdef FIXED
        block->acceleration_st = (uint32_t)(FPTOI(p_acceleration) * FPTOI(steps_per_mm)); // convert to: acceleration steps/sec^2
#else
        block->acceleration_st = (uint32_t)(0.5 + p_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
#endif
#else
        block->acceleration_st = (uint32_t)(FPTOI(FPCEIL(FPMULT2(p_acceleration, steps_per_mm)))); // convert to: acceleration steps/sec^2
#endif
        if((block->steps[X_AXIS] != 0) &&
                ((block->acceleration_st * (uint32_t)block->steps[X_AXIS]) > (axis_steps_per_sqr_second[X_AXIS] * block->step_event_count)))
            block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
        if((block->steps[Y_AXIS] != 0) &&
                ((block->acceleration_st * (uint32_t)block->steps[Y_AXIS]) > (axis_steps_per_sqr_second[Y_AXIS] * block->step_event_count)))
            block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
        if((block->steps[Z_AXIS] != 0) &&
                ((block->acceleration_st * (uint32_t)block->steps[Z_AXIS]) > (axis_steps_per_sqr_second[Z_AXIS] * block->step_event_count)))
            block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
        if((block->steps[E_AXIS] != 0) &&
                ((block->acceleration_st * (uint32_t)block->steps[E_AXIS]) > (axis_steps_per_sqr_second[E_AXIS] * block->step_event_count)))
            block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    }
    block->acceleration = FPDIV(ITOFP((int32_t)block->acceleration_st), steps_per_mm);
#ifdef FIXED
    block->acceleration_rate = (int32_t)(((int64_t)block->acceleration_st * 137439) >> 14);
#else
    block->acceleration_rate = (int32_t)((FPTYPE)block->acceleration_st * 8.388608);
#endif

    FPTYPE scaling = KCONSTANT_1;
    bool docopy = true;
    if ( moves_queued == 0 )
    {
        vmax_junction = minimumPlannerSpeed;
        scaling = FPDIV(vmax_junction, block->nominal_speed);
    }
    else if ( (block->nominal_speed <= smallest_max_speed_change) || (!use_accel) )
    {
        vmax_junction = block->nominal_speed;
    }
    else
    {
        FPTYPE delta_v;
        for (uint8_t i = 0; i < NUM_AXIS; i++)
        {
            delta_v = FPABS(current_speed[i] - prev_speed[i]);
            if ( delta_v > max_speed_change[i] )
            {
                FPTYPE s = FPDIV(max_speed_change[i], delta_v);
                if ( s < scaling ) scaling = s;
            }
        }
        if (scaling != KCONSTANT_1)
        {
            vmax_junction = FPMULT2(block->nominal_speed, scaling);
            for (uint8_t i = 0; i < NUM_AXIS; i++)
                prev_speed[i] = FPMULT2(current_speed[i], scaling);
            docopy = false;
        }
        else
            vmax_junction = block->nominal_speed;
    }
    if ( docopy ) memcpy(prev_speed, current_speed, sizeof(prev_speed));

#ifndef FIXSIGN
    FPTYPE v_allowable = max_allowable_speed(-block->acceleration,minimumPlannerSpeed,block->millimeters);
    block->max_entry_speed = vmax_junction;
    if (use_accel) block->entry_speed = min(vmax_junction, v_allowable);
    else block->entry_speed = vmax_junction;
#else
#ifdef SIMULATOR
    sblock = block;
#endif
    FPTYPE v_allowable = final_speed(block->acceleration,minimumPlannerSpeed,block->millimeters);
    if (vmax_junction < minimumPlannerSpeed)
    {
        block->entry_speed = minimumPlannerSpeed;
        block->max_entry_speed = minimumPlannerSpeed;
    }
    else
    {
        block->entry_speed = vmax_junction;
        block->max_entry_speed = vmax_junction;
    }
#endif

    // Initialize planner efficiency flags
    // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
    // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
    // the current block and next block junction speeds are guaranteed to always be at their maximum
    // junction speeds in deceleration and acceleration, respectively. This is due to how the current
    // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
    // the reverse and forward planners, the corresponding block junction speed will always be at the
    // the maximum junction speed and may always be ignored for any speed reduction checks.
    if (block->nominal_speed <= v_allowable)
    {
        block->nominal_length_flag = true;
    }
    else
    {
        block->nominal_length_flag = false;
    }
    block->recalculate_flag = true; // Always calculate trapezoid for new block
    block->speed_changed = false;

    block->advance_pressure_relax = 0;
    if((block->steps[E_AXIS] == 0) || ( extruder_only_move ) || (( extruder_advance_k == 0 ) && ( extruder_advance_k2 == 0)) || ( !use_accel ))
    {
        block->use_advance_lead = false;
        block->advance_lead_entry   = 0;
        block->advance_lead_exit    = 0;
        block->advance_lead_prime   = 0;
        block->advance_lead_deprime = 0;
        block->advance_pressure_relax = 0;
    }
    else
    {
        block->use_advance_lead = true;
    }

    calculate_trapezoid_for_block(block, scaling, scaling);

    //if ( block->advance_lead_entry < 0 )		zadvance = block->advance_lead_entry;
    //if ( block->advance_lead_exit < 0 )		zadvance = block->advance_lead_exit;
    //if ( block->advance_lead_prime < 0 )		zadvance = block->advance_lead_prime;
    //if ( block->advance_lead_deprime < 0 )	zadvance = block->advance_lead_deprime;
    //zadvance2 = block->advance_pressure_relax;

    // Move buffer head
    block_buffer_head = next_buffer_head;

    // Update position
    memcpy(position, target, sizeof(position)); // position[] = target[]


#ifdef DEBUG_ZADVANCE
#ifdef FIXED
    //	zadvance = FPTOF(roundk(FTOFP(2.8934), 3));	//0 = 7 1,2 = 3.0 8 = 2.895
    //	zadvance2 = FPTOF(roundk(FTOFP(2.3846), 3));	//0 = 6 1,2 = 2.5 8 = 2.383
#endif
#endif

    planner_recalculate();

    return true;
}

void plan_set_position(const int32_t &x, const int32_t &y, const int32_t &z, const int32_t &e)
{
#ifndef CMD_SET_POSITION_CAUSES_DRAIN
    CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
#endif
    position[X_AXIS] = x;
    position[Y_AXIS] = y;
    position[Z_AXIS] = z;
    position[E_AXIS] = e;
#ifndef CMD_SET_POSITION_CAUSES_DRAIN
    CRITICAL_SECTION_END;  // Fill variables used by the stepper in a critical section
#endif

#ifdef CMD_SET_POSITION_CAUSES_DRAIN
    st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS]);
#endif
}

void plan_set_e_position(const int32_t &e)
{
    position[E_AXIS] = (int32_t)e;
    st_set_e_position(position[E_AXIS]);
}

uint8_t movesplanned()
{
    return (block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
}



//Figure out the acceleration stats by scanning through the command pipeline

void accelStatsGet(float *minSpeed, float *avgSpeed, float *maxSpeed)
{
    block_t *block;
    int32_t count = 0;
    uint8_t block_index = block_buffer_tail;

    FPTYPE smax = 0, savg = 0;
#ifdef FIXED
    FPTYPE smin = KCONSTANT_1000;
#else
    FPTYPE smin = 1000.0;
#endif

    while(block_index != block_buffer_head)
    {
        block = &block_buffer[block_index];

        smin = min(smin, block->entry_speed);
        smax = max(smax, block->nominal_speed);
        savg += block->nominal_speed;

        block_index = next_block_index(block_index);
        count ++;
    }

    if ( count )
    {
        //We have stats
        *minSpeed = FPTOF(smin);
        *maxSpeed = FPTOF(smax);
        *avgSpeed = FPTOF(FPDIV(savg, ITOFP(count)));
    }
    else
    {
        //We have no stats
        *minSpeed = 0.0;
        *maxSpeed = 0.0;
        *avgSpeed = 0.0;
    }
}

#endif
