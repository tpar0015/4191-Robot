# Recieve position from rotary encoders
# Recieve readouts from ultrasonic sensor

class Map:
    """Generates map of arena, navigates shortest path. Online updating of path
    with detected obstacles factored in"""
    def __init__(self, args):
        """
        Generates nodal map of the arena, with robot starting positon
        """
        pass

    def djikstras(self, waypoint):
        """
        Designs shortest path
        """
        pass


    def update_location(self, encoder_readout):
        """
        Updates predicted location on nodal map
        """
        pass


    def remap(self, ultrasonic_readout):
        """
        Re calibrates map with object blocked out on nodes.
        """
        pass

    def update_path(self, ):
        """
        Updates path to avoid any new obstacles
        """
        pass
