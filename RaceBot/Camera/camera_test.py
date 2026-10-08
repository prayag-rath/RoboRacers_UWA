#!/usr/bin/env python3

from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()

# us / frame (50000 --> 20 fps)
freq = 50000

config = picam2.create_preview_configuration(
    main={"size": (640, 480)},
    controls={"FrameDurationLimits": (freq, freq)}
)

picam2.configure(config)

picam2.start_preview(Preview.QT)
picam2.start()

try:
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    picam2.stop()
    picam2.stop_preview()
