# Drive-by-wire-Stm32
This project contains autonomous vehicle **drive-by-wire** implementation on stm32f407 board. This project is in a template from which can be fork.

## How it works
- İncoming data from computer has **speed** and **steering angle** values. 
- Stm32f407 reads and parse incoming data.
- After, functions which has **throttle-by-wire, brake-by-wire and steer-by-wire** implementations works.
- İn our example we have steer and brake **stepper motors** and **bldc motor** for throttle.
