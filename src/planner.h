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
