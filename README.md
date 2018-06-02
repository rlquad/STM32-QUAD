# STM32-QUAD
QUADCOPTER BASED ON STM32F303K8 NUCLEO BOARD

MOTOR CONTROL

MOTOR     PORT      PIN       TIMER     CHANNEL

1         PA11      D10

2         PA7       A6

3         PB0       D3

4         PB1       D6


RECEIVER CHANNEL

CHANNEL       FUNCTION       PORT        PIN      TIMER     CHANNEL

1             YAW            PA0         A0

2             PITCH          PA3         A2

3             THROTTLE       PA8         D9

4             ROLL           PA10        D0

5             ARM            PA1         A1


# CODE IS RUNNING AT 64MHz AND ANGLE CALCULATION CODE FOR SIMPLE COMPLIMENTARY FILTER TAKES ABOUT 500us

# MOTOR CONTROL IS CARRIED OUT EVERY 4000us BY MAKING USE OF TIMER INTERRUPT. IT IS INDEPENDENT OF THE ANGLE 
  CALCULATION CODE TIMING. SO GREATER FREEDOM CAN BE EXERCISED FOR ENABLING BETTER CODE FOR ANGLE CALCULATION AND PID
  WITHOUT THE FEAR OF INTERFERENCE WITH MOTOR CONTROL CODE.
    
# RECEIVER PWM MEASUREMENT IS CARRIED OUT THROUGH 5 INDEPENDENT TIMER CHANNELS. USE OF EXTERNAL INTERRUPTS FOR THE
  SAME DID NOT YIELD EXCEPTED RESULTS.
    
