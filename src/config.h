/** 
  config.h - compile time configuration
*/

// The number of linear motion in the planner buffer to be planned at any give time. 
// The vast majority of RAM that Grbl uses is based on this buffer size. Only increase if there is extra avaliable RAM.
// #define BLOCK_BUFFER_SIZE 16 // Uncomment to override default in planner.h

// Allows GRBL to track and report gcode line numbers. Enabling this means that the planner buffer
// goes from 16 to 15 to make room for the additional line number data in the plan_block_t struct
// #define USE_LINE_NUMBERS // Disable by default. Uncomment to enable.
