"""
    Utility to configure Tkinter for virtual environment.

    Used by the relevant scripts. DON'T RUN THIS DIRECTLY!!!
"""
import os
import sys
from pathlib import Path

def setup_tkinter():
    """Set up Tcl/Tk environment variables for virtual environments."""
    if hasattr(sys, 'base_prefix'):
        python_root = Path(sys.base_prefix)
    else:
        python_root = Path(sys.prefix)

    tcl_dir = python_root / 'tcl'
    tcl_subdirs = list(tcl_dir.glob('tcl8.*'))
    tk_subdirs = list(tcl_dir.glob('tk8.*'))

    if tcl_subdirs and tk_subdirs:
        os.environ['TCL_LIBRARY'] = str(tcl_subdirs[0])
        os.environ['TK_LIBRARY'] = str(tk_subdirs[0])
