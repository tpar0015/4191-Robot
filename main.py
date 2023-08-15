from load import load
from target import localise, aim
from fire import launch, reset

x_pos, y_pos, x_target, y_target = None, None, None, None

# Run load process
load()

# Run aim process
x_pos, y_pos, x_target, y_target = localise()
aim()

# Run launch process
launch()
reset()