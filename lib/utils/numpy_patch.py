"""
Monkey patch for numpy.disp which was removed in numpy 2.0
This adds back the disp function for compatibility with older code
"""
import numpy as np

# Add the deprecated disp function back to numpy
def disp(mesg):
    """
    Display a message on the screen.

    This function was deprecated in NumPy 1.7.0 and removed in NumPy 2.0.0
    This is a compatibility shim.

    Parameters
    ----------
    mesg : str
        Message to display.
    """
    print(mesg)

# Monkey-patch numpy
if not hasattr(np, 'disp'):
    np.disp = disp
