from gpiozero import Button

button_pin = 2 #? change later


button = Button(button_pin)

# Robot should be in a safe state.
# Needs to detect when package is loaded
# Then go to 'find target'

def load():

    # wait for button to be pressed
    button.wait_for_press()
    sleep(2)

    # return True when loaded
    return True