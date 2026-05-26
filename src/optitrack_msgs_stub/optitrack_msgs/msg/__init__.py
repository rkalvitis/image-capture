"""Placeholder for optitrack_msgs.msg.

`optitrack_ros_py` imports this module unconditionally, but the actual
message classes (Description, Data, CameraDescription, etc.) are only
*instantiated* when the driver's `publish_description` or `publish_data`
parameters are true. Both default to false, so a stub that just satisfies
the import is sufficient.

If Jerome later publishes the real `optitrack_msgs` package, drop this stub
and install the real one — no other code changes needed.
"""


class _Stub:
    pass


def __getattr__(_name):  # PEP 562: module-level __getattr__
    return _Stub
