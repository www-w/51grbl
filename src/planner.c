/*
 * planner.h - buffers movement commands and manages the acceleration profile plan
 */

#pragma once
#include "planner.h"

static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];	// A ring buffer for motion instructions
static uint8_t block_buffer_tail;	// Index of the block to process now
static uint8_t block_buffer_head;	// Index of the next block to be pushed
static uint8_t next_buffer_head;	// Index of the next buffer head
static uint8_t block_buffer_planned;// Index of the optimally planned block

// Define planner variables
typedef struct {
	int32_t position[N_AXIS];	// The planner position of the tool in absolute steps. Kept separate from g-code position for movements requiring multiple line motions, i.e. arcs, canned cycles, and backlash compensation.
	float previous_unit_vec[N_AXIS];	// Unit vector of previous path line segment
	float previous_nominal_speed;	// Nominal speed of previous path line segment
} planner_t;

static planner_t pl;

// Returns the index of the next block in the ring buffer. Also called by stepper segment buffer.
uint8_t plan_next_block_index(uint8_t block_index){
	block_index++;
	if (block_index == BLOCK_BUFFER_SIZE) {
		block_index = 0;
	}
	return block_index;
}
