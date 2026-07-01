"""Quick test: confirms pyorbbecsdk can open the Orbbec Femto Mega.
Run with:  sudo pixi run -e default python scripts/test_camera.py
"""
from pyorbbecsdk import Config, OBFormat, OBSensorType, Pipeline

p = Pipeline()
dev = p.get_device()
di  = dev.get_device_info()
print(f'Device  : {di.get_name()}')
print(f'Serial  : {di.get_serial_number()}')
print(f'Firmware: {di.get_firmware_version()}')
p.stop()
print('OK — camera is accessible')
