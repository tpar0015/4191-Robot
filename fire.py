from gpiozero import LED

# launch projectile

magnet_pin = 7 # change later

magnet = LED(magnet_pin)

def launch():
    # disable electromagnet
    magnet.off()



# reset to safe pose
def reset():

    # reset position
    # centre x axis
    # centre y axis
    # retract launcher
    # enable electromagnet
    magnet.on()