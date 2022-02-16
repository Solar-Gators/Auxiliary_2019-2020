# Auxiliary_2021-2022
Code by the Auxiliary subteam.
Example CAN usage between boards.

There are two separate examples, one for sending Aux data between boards,
and one for communicating with the Mitsuba motor controller.

## Important To-Do
In projects that utilize CAN interrupts, ALWAYS delete/comment out the "CEC_CAN_IRQHandler(void)" method from Core/Src/stm32f0xx_it.c. 
We do this because we have overwritten this function in subsystem-can-driver/subsystem-data-module