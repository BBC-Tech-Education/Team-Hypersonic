import csi
import time
from machine import LED, UART
from math import sqrt

ROBOT_1 = True
CALIBRATION = False

FRAME_HEIGHT = 480 if ROBOT_1 else 480
FRAME_WIDTH = 480 if ROBOT_1 else 480

#        ROBOT_1,            ROBOT_2
FRAME_CX = 245 if ROBOT_1 else 225
FRAME_CY = 197 if ROBOT_1 else 190
FRAME_R = 223 if ROBOT_1 else 210

GOAL_MIN_AREA = 200
GOAL_MIN_PIXELS = 100
BALL_MIN_AREA = 5
BALL_MIN_PIXELS = 5

BALL = 1
YELLOW = 2
BLUE = 4

THRESHOLDS = [
    (37, 62, 24, 60, 30, 55),  # Ball
    (26, 39, -4, 10, 20, 33),  # Yellow
    (25, 35, -16, -8, -12, 5),  # Blue
]
# (100, 100, 0, 0, 0, 0)

led = LED("LED_GREEN")
led.on()

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize(csi.VGA)
cam.window((FRAME_WIDTH, FRAME_HEIGHT))
cam.snapshot()

if CALIBRATION:
    # cam.auto_exposure(True)
    # cam.auto_gain(True)
    cam.auto_exposure(False, exposure_us=8328)
    cam.auto_gain(False, gain_db=20.1079)
    cam.auto_whitebal(True)
    print("Calibration Mode")

else:
    cam.auto_exposure(False, exposure_us=8328)  # 120fps: 8328;   60fps: 16648
    cam.auto_gain(False, gain_db=20.1079)
    cam.auto_whitebal(False, rgb_gain_db=(1.9382, 0.0, 7.9588))

clock = time.clock()
clock.reset()

uart = UART(3, 115200)
uart.init(115200)
led.off()


if CALIBRATION:
    while (True):
        clock.tick()
        img = cam.snapshot()
        img.draw_circle((FRAME_CX, FRAME_CY, FRAME_R))
        print("Gain:", cam.gain_db(), "Exposure:", cam.exposure_us(), "WB:", cam.rgb_gain_db())

else:
    while (True):
        clock.tick()
        # time2 = time.ticks_ms()
        img = cam.snapshot()
        # print(time.ticks_ms() - time2, end="\t")

        blobs = img.find_blobs(
            THRESHOLDS,
            x_stride=5,
            y_stride=5,
            area_threshold=BALL_MIN_AREA,
            pixels_threshold=BALL_MIN_PIXELS,
            merge=False,
            margin=5,
        )
        # print(time.ticks_ms() - time2, len(blobs), end="\t")

        # Start bytes, ball, yellow, blue
        data = [255, 255, 500, 500, 500, 500, 500, 500]

        rawBlobs = [None, None, None]

        for blob in blobs:

            blobX = blob.cx - FRAME_CX
            blobY = blob.cy - FRAME_CY
            mag = sqrt(blobX * blobX + blobY * blobY)

            if mag > FRAME_R:
                continue

            if blob.code == BALL:
                if rawBlobs[0] is None:
                    rawBlobs[0] = blob
                else:
                    if blob.area > (rawBlobs[0]).area:
                        rawBlobs[0] = blob

            elif blob.code == YELLOW:
                if rawBlobs[1] is None:
                    if blob.area > GOAL_MIN_AREA and blob.pixels > GOAL_MIN_PIXELS:
                        rawBlobs[1] = blob
                else:
                    if blob.area > (rawBlobs[1]).area:
                        rawBlobs[1] = blob

            elif blob.code == BLUE:
                if rawBlobs[2] is None:
                    if blob.area > GOAL_MIN_AREA and blob.pixels > GOAL_MIN_PIXELS:
                        rawBlobs[2] = blob
                else:
                    if blob.area > (rawBlobs[2]).area:
                        rawBlobs[2] = blob

        if rawBlobs[0] is not None:
            # print(rawBlobs[0].area)
            data[2] = (rawBlobs[0]).cx
            data[3] = (rawBlobs[0]).cy

        if rawBlobs[1] is not None:
            data[4] = (rawBlobs[1]).cx
            data[5] = (rawBlobs[1]).cy

        if rawBlobs[2] is not None:
            data[6] = (rawBlobs[2]).cx
            data[7] = (rawBlobs[2]).cy

        # img.draw_line((FRAME_CX - 5, FRAME_CY, FRAME_CX + 5, FRAME_CY)) # horizontal
        # img.draw_line((FRAME_CX, FRAME_CY - 5, FRAME_CX, FRAME_CY + 5)) # vertical

        # img.draw_line((FRAME_CX, FRAME_CY, data[2], data[3]))  # draws line from center to ball
        # img.draw_line((FRAME_CX, FRAME_CY, data[4], data[5]))  # draws line from center to ygoal
        # img.draw_line((FRAME_CX, FRAME_CY, data[6], data[7]))  # draws line from center to bgoal

        uart.writechar(data[0])
        uart.writechar(data[1])
        uart.writechar(data[2] & 255)
        uart.writechar((data[2] >> 8) & 255)
        uart.writechar(data[3] & 255)
        uart.writechar((data[3] >> 8) & 255)
        uart.writechar(data[4] & 255)
        uart.writechar((data[4] >> 8) & 255)
        uart.writechar(data[5] & 255)
        uart.writechar((data[5] >> 8) & 255)
        uart.writechar(data[6] & 255)
        uart.writechar((data[6] >> 8) & 255)
        uart.writechar(data[7] & 255)
        uart.writechar((data[7] >> 8) & 255)
        # print(clock.fps())
        # print(time.ticks_ms() - time2)

        # print(data[2], data[2] & 255, (data[2] >> 8) & 255)
