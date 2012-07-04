
from .api import logint, logstring, logfloat, log

"""
rosdashboard exposes the log api, log is a convenience method
and detects the type of the message itself.
"""
__all__ = [
    'logint',
    'logstring',
    'logfloat',
    'log'
    ]
