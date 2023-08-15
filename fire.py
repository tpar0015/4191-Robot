from motorctl import motor_x, motor_y
from gpiozero import LED
from time import sleep

# launch projectile

magnet_pin = 7  #? change later

magnet = LED(magnet_pin)

def launch():
    # disable electromagnet
    magnet.off()



# reset to safe pose
def reset(aim_x, aim_y):

    # reset position

    ## centre x axis
    ### start turning toward centre
    if aim_x == 0:
        pass    # do nothing
    elif aim_x < 0:
        motor_x(0.1) # turn right
    elif aim_x > 0:
        motor_x(-0.1) # turn left

    ### stop turning after enough ticks on rotary encoder
    

    ## centre y axis
    ## retract launcher
    ### start turning

    ### stop turning after enough ticks on rotary encoder

    ## enable electromagnet
    magnet.on()