# 4191-Robot

## Running loop

### 1. Load package

Robot should be in a safe state.
Needs to detect when package is loaded

### 2. Find target

Check sensors, apply computer vision if necessary.
Determine coordinates in warehouse.
    Possible method: find corners of the warehouse, centre for B and some angle for A/C. This relies on accuracy of robot placement and placement of bins

### 3. Aim

a. Aim horizontal
b. Aim vertical
c. Determine launch velocity

(confirm by checking sensors that target is centred)

### 4. Fire

Shoot projectile with predetermined parameters

### 5. Reset

Return to safe state