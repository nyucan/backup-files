from RPIO import PWM
import time

# Setup RPIO Pins
RPIO.setup(26, RPIO.OUT, initial = RPIO.LOW) #ENA
RPIO.setup(11, RPIO.OUT, initial = RPIO.LOW) #ENB
RPIO.setup(19, RPIO.OUT, initial = RPIO.LOW) #IN1
RPIO.setup(13, RPIO.OUT, initial = RPIO.LOW) #IN2
RPIO.setup(6, RPIO.OUT, initial = RPIO.LOW)  #IN3
RPIO.setup(5, RPIO.OUT, initial = RPIO.LOW)  #IN4


PWM.setup()
PWM.init_channel(0)
RPIO.output(19, RPIO.HIGH)
RPIO.output(13, RPIO.LOW)
RPIO.output(6, RPIO.HIGH)
RPIO.output(5, RPIO.LOW)


try:
    PWM.add_channel_pulse(0, 26, 0, 10000)   #start at 0 and width = 10000us i.e 50% 
    PWM.add_channel_pulse(0, 11, 0, 10000)
    time.sleep(5)
    
except KeyboardInterrupt:
    PWM.clear_channel(0)

# PWM.clear_channel_gpio(0, 17)
PWM.cleanup()
RPIO.cleanup()



