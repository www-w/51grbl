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

// Returns the index of the previous block in the ring buffer
static uint8_t plan_prev_block_index(uint8_t block_index){
	if(block_index == 0) {
		block_index = BLOCK_BUFFER_SIZE;
	}
	block_index--;
	return block_index;
}

/*                 PLANNER SPEED DEFINITION

                            +--------+  <- current->nominal_speed
                           /          \
 current->entry_speed ->  +            \
                          |             + <- next->entry_speed (aka exit speed)
						  +-------------+
						      time -->

  Recalculates the motion plan according to the following basic guidelines:
    1. Go over every feasible block sequentially in reverse order and calculate the junction speeds (i.e. current->entry_speed) such that:
      a. No junction speed exceeds the pre-computed maximum junction speed limit or nominal speeds of neighboring blocks.
      b. A block entry speed cannot exceed one reverse-computed from its exit speed (next->entry_speed) with a maximum allowable deceleration over the block travel distance.
      c. The last (or newest appended) block is planned from a complete stop (an exit speed of zero).
    2. Go over every block in chronological (forward) order and dial down junction speed values if
      a. The exit speed exceeds the one forward-compted from its entry speed with the maximum allowable acceleration over the block travel distance.
  When these stages are complete, the planner will have maximized the velocity profiles throughout the all of the planner blocks, where every block is operating at its maximum allowable acceleration limits. In other words, for all of the blocks in the planner, the plan is optimal and no further speed improvements are possible. If a new block is added to the buffer, the plan is recomputed according to the said guidelines for a new optimal plan.
  
  To increase computational efficiency of these guidelines, a set of planner block pointers have been created to indicate stop-compute points for when the planer guidelines cannot logically make any further changes or improvements to the plan when in normal operation and new blocks are streamed and added to the planner buffer. For example, if a subset of sequential blocks in the planner have been planned. and are bracketed by junction velocities at their maximums (or by the first planner block as well), no new block added to the planner will alter the velocity profiles within them. So we no longer have to compute them. Or, if a set of sequential blocks from the first block in the planner (or a optimal stop-compute point) are all accelerating, they are all optimal and can not be altered by a new block added to the planner buffer, as this will only further increase the plan speed to chronological blocks until a maximum junction velocity is reached. However, if the operational conditions of the plan changes from infrequently used feed holds or feedrate overrides, the stop-compute pointers will be reset and the entire plan is recomputed as stated in the general guidelines.

  Planner buffer index mapping:
  - block_buffer_tail: Points to the beginning of the planner buffer. First to be executed or being executed.
  - block_buffer_head: Points to the buffer block after the last block in the buffer. Used to indicate whether the buffer is full or empty. As described for standard ring buffers, this block is always empty.
  - next_buffer_head: Points to next planner buffer block after the buffer head block. When equal to the buffer tail, this indicates the buffer is full.
  - block_buffer_planned: Points to the first buffer block after the last optimally planned block for normal streaming operating conditions. Use for planning optimizations by avoiding recomputing parts of the planner buffer that don't change with the addition of a new block, as describe above. In addition, this block can never be less than block_buffer_tail and will always be pushed forward and maintain this requirement when encountered by the plan_discard_current_block() routine during a cycle.

*/



void plan_reset(){

	memset(&pl, 0, sizeof(planner_t)); //Clear planner struct
	plan_reset_buffer();
}

void plan_reset_buffer(){
	block_buffer_tail = 0;
	block_buffer_head = 0; // Empty = tail
	next_buffer_head = 1; // plan_next_block_index(block_buffer_head)
	block_buffer_planned = 0; // = block_buffer_tail;
}

void plan_discard_current_block(){
	if(block_buffer_head != block_buffer_tail) {
		// Discard non-empty buffer.
		uint8_t block_index = plan_next_block_index( block_buffer_tail );
		//Push block_buffer_planned pointer, if encountered.
		if (block_buffer_tail == block_buffer_planned) {
			block_buffer_planned = block_index;
		}
		block_buffer_tail = block_index;
	}
}

float plan_get_exec_block_exit_speed_sqr(){
	uint8_t block_index = plan_next_block_index(block_buffer_tail);
	if(block_index == block_buffer_head) {
		return (0.0);
	}
	return (block_buffer[block_index].entry_speed_sqr);
}

// Returns the availability status of the block ring buffer. True, if full.
uint8_t plan_check_full_buffer(){
	if( block_buffer_tail == next_buffer_head) {
		return true;
	}
	return false;
}

