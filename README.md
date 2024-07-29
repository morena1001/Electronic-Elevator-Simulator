# Electronic-Elevator-Simulator

This project simulates a single elevator that services a three floor building. A guest is able to request usage of the elevator in any floor, which will either be serviced immediately by the elevator, or be picked up while the elevator is on route to a destination in another floor. Inside the elevator, a guest is able to choose a not already chosen destination. This project was developed using an STM32F302R8 alongside a PCB housing the external buttons and LEDs.

In terms of inputs, there are a total of 7 input buttons. The first 4 correspond to the request buttons located outside of the elevator (one each for floors 1 and 3, which only go in one direction, and two for floor 2, which guests can choose to go upwards or downwards from). The other 3 inputs buttons  correspond to the destination buttons located inside of the elevator (one for each floor). 

In terms of outputs, there are a total of 12 LEDs. The first 3 display the current position of the elevator (one for each floor), the next 3 display the current destination of the elevator (one for each floor), the next 2 display the current direction of the elevator (one for each possible direction, up and down), the next 3 correspond to the request made to use the elevator (one for each floor), and the last LED blinks at regular intervals to show that a destination can be chosen.

## GPIO PERIPHERALS

**_PB0_** - CN7 pin 34, A3

Defined as II1 in the program, or _Inside Interface 1_, this pin connects to the external button in charge of setting floor 1 as a destination.

**_PB1_** - CN10 pin 24, D8

Defined as II2 in the program, this pin connects to the external button in charge of setting floor 2 as a destination.

**_PB2_** - CN10 pin 22, D7

Defined as II3 in the program, this pin connects to the external button in charge of setting floor 3 as a destination.

**_PB3_** - CN10 pin 31, D3

Defined as OI1U, or _Outside Interface 1 Up_, this pin connects to the external button in charge of requesting floor 1 with an upward direction.

**_PB4_** - CN10 pin 27, D5

Defined as OI2D, or _Outside Interface 2 Down_, this pin connects to the external button in charge of requesting floor 2 with a downward direction.

**_PB5** - CN10 pin 29, D4

Defined as OI2U, this pin connects to the external button in charge of requesting floor 2 with an upward direction.

**_PB6_** - CN10 pin 17, D10

Defined as OI3D, this pin connects to the external button in charge of requesting floor 3 with a downward direction.

**_PC0_** - CN7 pin 38, A5

Defined as PF1, or _Position Floor 1_, this pin connects to the external LED in charge of displaying floor 1 as the elevator's current position.

**_PC1_** - CN7 pin 36, A4

Defined as PF2, this pin connects to the external LED in charge of displaying floor 2 as the elevator's current position.

**_PC2_** - CN7 pin 35, A4

Defined as PF3, this pin connects to the external LED in charge of displaying floor 3 as the elevator's current position.

**_PC3_** - CN7 pin 37, A5

Defined as DF1, or _Destination Floor 1_, this pin connects to the external LED in charge of displaying floor 1 as a destination.

**_PC4_** - CN10 pin 34,D2

Defined as DF2, this pin connects to the external LED in charge of displaying floor 2 as a destination.

**_PC5_** - CN10 pin 6, D14

Defined as DF3, this pin connects to the external LED in charge of displaying floor 3 as a destination.

**_PC6_** - CN10 pin 4, D15

Defined as DU, or _Direction Up_, this pin connects to the external LED in charge of displaying the elevator's upward direction when moving.

**_PC7_** - CN10 pin 19, D9

Defined as DD, or _Direction Down_, this pin connects to the external LED in charge of displaying the elevator's downward direction when moving.

**_PC8_** - CN10 pin 2, above D15

Defined as FR1, or _Floor Request 1_, this pins connects to the external LED in charge of displaying floor 1's request to use the elevator.

**_PC9_** - CN10 pin 1, above D15

Defined as FR2, this pins connects to the external LED in charge of displaying floor 2's request to use the elevator.

**_PC10_** - CN7 pin 1

Defined as FR3, this pins connects to the external LED in charge of displaying floor 3's request to use the elevator.

**_PC11_** - CN7 pin 2

Defined as BCDI, or _Button Choose Destination Indicator_, this pin connects to the external LED in charge of blinking a certain pattern to show that a destination can be chosen.

## BUTTON AND LED GPIO CONFIGURATIONS

All of the external buttons are configured as follows. GPIO mode is set to "External Interrupt Mode with Rising edge trigger detection", and GPIO Pull-up/Pull-down is set to "Pull up". All of the external LEDs are configured as follows. GPIO output level is set to "Low", GPIO mode is set to "Output Push Pull", GPIO Pull-up/Pull-down is set to "No pull-up and no pull-down" and Maximum output speed is set to "Low".
