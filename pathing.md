# 1. Localisation (localisation.py)

1. Consider immediately previous event.
    * Driving? Then robot is in middle of arena
    * Picked up box? Then robot is in the loading zone
    * Just delivered box? Then robot is in the delivery zone near A/B/C

2. Check sensor data

3. Convert sensor data to X,Y

4. Run navigation.py


# 2. Navigation (navigation.py)

0. Set up array of legal nodes (that are within the arena and NOT near any obstacles)

1. Check target destination
    * What's on the box? A/B/C

2. Check current location

3. Get target location

4. Run Dijkstra's algorithm

5. Run motor control
    * Generate motor inputs, pass to motorctl.py

6. IF arrived at delivery location, run loading.py
    * Pass LOAD or UNLOAD

# 3. Motor Control (motorctl.py) 

1. Get motor inputs

2. Translate to duty cycle for PWM

3. Apply to pins


# 4. Pickup/Delivery (loading.py)

0. If passed UNLOAD, goto 1; else if passed LOAD, goto 2

1. Unload
    * Turn toward wall
    * Tip out tray

2. Load
    * Turn away from wall
    * Raise tray