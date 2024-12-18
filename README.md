# CS350
SNHU CS350 Thermostat Project

Summarize the project and what problem it was solving.

  The goal of this gpiointerrupt.c project was to design a low-level thermostat prototype on the TI CC3320x LAUNCHXL circuit board using Code Composer Studio. 
From the assignment: "For the prototype, you will use the TMP006 temperature sensor to read the room temperature (via I2C), an LED to indicate the output to the thermostat where LED on = heat on (via GPIO), two buttons to increase and decrease the set temperature (via GPIO interrupt), and the UART to simulate the data being sent to the server."

  The goal of the uart2echo.c file was to design code that would turn on an LED light if "ON" was input into the UART, and turn it off if "OFF" was input.

What did you do particularly well?

  Personally, I am most proud of my state machine code. I think I did a good job with the commenting, too, but I've been wrong about that before.

Where could you improve?

  I confess I don't completely understand why the DISPLAY(snprintf(... currentTemp, setTemp, heat, seconds) works as required at the end of the main loop and not in the updateOutputSM function, so if anything needs to improve, it's my understanding of the way the code works.

What tools and/or resources are you adding to your support network?

  The TI online documentation has been extremely useful, so that is going into the toolbelt; CCS is also a new tool that I look forward to experimenting with more on my own time.

What skills from this project will be particularly transferable to other projects and/or course work?

  Everything I have learned about embedded systems and the ability to program microcontrollers should be especially useful going forward.

How did you make this project maintainable, readable, and adaptable?

  The tried and true methods of adhering to best coding practices and making in-code documentation/commenting informative but not overwhelming. 
