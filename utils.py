def is_raspberry_pi():
    """Check if the code is running on a Raspberry Pi."""
    try:
        with open('/proc/device-tree/model') as f:
            return 'Raspberry Pi' in f.read()
    except:
        return False

def trigger_detected():
    # TODO: replace with real logic
    return True

def undock_trigger_detected():
    # TODO: replace with real logic
    return False
