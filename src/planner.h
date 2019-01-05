/*
   planner.h - buffers movement commands and manages the acceleration profile plan
   */
#pragma once

// The number of linear motions that can be in the plan at any give time
#ifndef BLOCK_BUFFER_SIZE
	#ifdef USE_LINE_NUMBERS
		#define BLOCK_BUFFER_SIZE 15
	#else
		#define BLOCK_BUFFER_SIZE 16
	#endif
#endif

// Returned status message from planner.
#define PLAN_OK true
#define PLAN_EMPTY_BLOCK false

// Define planner data condition flags. Used to denote running conditions of a block.
#define PL_COND_FLAG_RAPID_MOTION	bit(0)
#define PL_COND_FLAG_SYSTEM_MOTION	bit(1) // Single motion. Circumvents planner state. Used by home/park.
#define PL_COND_FLAG_NO_FEED_OVERRIDE	bit(2) // Motion does not honor feed override.
#define PL_COND_FLAG_INVERSE_TIME	bit(3) // Interprets feed rate value as inverse time when set.
#define PL_COND_FLAG_SPINDLE_CW		bit(4)
#define PL_COND_FLAG_SPINDLE_CCW	bit(5)
#define PL_COND_FLAG_COOLANT_FLOOD	bit(6)
#define PL_COND_FLAG_COOLANT_MIST	bit(7)
#define PL_COND_MOTION_MAST (PL_COND_FLAG_RAPID_MOTION|PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE)
#define PL_COND_SPINDLE_MASK (PL_COND_FLAG_SPINDLE_CW|PL_COND_FLAG_SPINDLE_CCW)
#define PL_COND_ACCESSORY_MASK (PL_COND_FLAG_SPINDLE_CW|PL_COND_FLAG_SPINDLE_CCW|PL_COND_FLAG_COOLANT_FLOOD|PL_COND_FLAG_COOLANT_MIST)

// This struct stores a linear movement of a g-code block motion with its critical "nominal" values
// are as specified in the source g-code.
typedef struct {
	// Fields used by the bresenham algorithm for tracing the line
	// NOTE: Used by stepper algorithm to execute the block correctly. Do not alter these values.
	uint32_t steps[N_AXIS];		// Step count along each axis
	uint32_t step_event_count; 	// The maximum step axis count and number of steps required to complete this block.
	uint8_t direction_bits;		// The direction bit set for this block

	// Block condition data to ensure correct execution depending on states and overrides.	
	uint8_t condition;		// Block bitflag variable defining block run conditions. Copied from pl_line_data.
#ifdef USE_LINE_NUMBERS
	int32_t line_number;		// Block line number for real-time reporting. Copied from pl_line_data.
#endif

	// Fields used by the motion planner to manage acceleration. Some of these values may be updated
	// by the stepper module during execution of special motion cases for replanning purposes.
	float entry_speed_sqr;		// The current planned entry speed at block junction in (mm/min)^2
	float max_entry_speed_sqr;	// Maximum allowable entry speed based on the minimum of junction limit and neighboring nominal speeds with overrides in (mm/min)^2
	float acceleration;		// Axis-limit adjusted line acceleration in (mm/min^2). Does not change.
	float millimeters;		// The remaining distance for this block to be executed in (mm).
	//Note: This value may be altered by stepper algorithm during execution.
	// Stored rate limiting data used by planner when changes occur.
	float max_junction_speed_sqr;	// Junction entry speed limit based on direction vectors in (mm/min)^2
	float rapid_rate;		// Axis-limit adjusted maximum rate for this block direction in (mm/min)
	float programmed_rate;		// Programmed rate of this block (mm/min).
} plan_block_t;

// Planner data prototype. Must be used when passing new motions to the planner.
typedef struct {
	float feed_rate;	// Desired feed rate for line motion. Value is ignored, if rapid motion.
	uint8_t condition;	// Bitflag variable to indicated planner conditions. See defines above.
#ifdef USE_LINE_NUMBERS
	int32_t line_number;	// Desired line number to report when executing.
#endif
} plan_line_data_t;

//Initialize and reset the motion plan subsystem
void plan_reset(); // Reset all
void plan_reset_buffer(); // Reset buffer only.

// Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position
// in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
uint8_t plan_buffer_line(float *target, plan_line_data_t *pl_data);

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the planner block for the special system motion cases. (Parking/Homing)
plan_block_t *plan_get_system_motion_block();

// Gets the current block. Returns NULL if buffer empty
plan_block_t *plan_get_current_block();

// Called periodically by step segment buffer. Mostly used internally by planner.
uint8_t plan_next_block_index(uint8_t block_index);

// Called by step segment buffer when computing executing block velocity profile.
float plan_get_exec_block_exit_speed_sqr();

// Called by main program during planner calculations and step segment buffer during initialization.
float plan_compute_profile_nominal_speed(plan_block_t *block);

// Re-calculates buffered motions profile parameters upon a motion-based override change.
void plan_update_velocity_profile_parameters();

// Reset the planner position vector (in steps)
void plan_sync_position();

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize();

// Returns the number of available blocks are in the planner buffer.
uint8_t plan_get_block_buffer_available();

// Returns the status of the block ring buffer. True, if buffer is full.
uint8_t plan_check_full_buffer();

void plan_get_planner_mpos(float *target);
