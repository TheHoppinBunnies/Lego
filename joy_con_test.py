import time

from pyjoycon import ButtonEventJoyCon, get_R_id, get_L_id
from gpiozero import Robot, DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
import math
from time import sleep
from pygame import mixer
from gtts import gTTS
import os

mixer.init()
print("Mixer initialized")


def clamp(x, min, max):
    if x < min:
        return min
    if x > max:
        return max
    return x


class RumbleJoyCon(ButtonEventJoyCon):
    def __init__(self, *args, **kwargs):
        ButtonEventJoyCon.__init__(self, *args, **kwargs)

    def send_rumble(self, data=b'\x00\x00\x00\x00\x00\x00\x00\x00'):
        self._RUMBLE_DATA = data
        self._write_output_report(b'\x10', b'', b'')

    def enable_vibration(self, enable=True):
        """Sends enable or disable command for vibration. Seems to do nothing."""
        self._write_output_report(b'\x01', b'\x48', b'\x01' if enable else b'\x00')

    def rumble_simple(self):
        """Rumble for approximately 1.5 seconds (why?). Repeat sending to keep rumbling."""
        self.send_rumble(b'\x98\x1e\xc6\x47\x98\x1e\xc6\x47')

    def rumble_stop(self):
        """Instantly stops the rumble"""
        self.send_rumble()


class RumbleData:
    def __init__(self):
        self.h_f = None
        self.amp = None
        self.l_f = None
        self.t = None
        self.timed_rumble = None

    def set_vals(self, low_freq, high_freq, amplitude, time=0):
        self.h_f = high_freq
        self.amp = amplitude
        self.l_f = low_freq
        self.timed_rumble = False
        self.t = 0
        if time != 0:
            self.t = time / 1000.0
            self.timed_rumble = True

    def __init__(self, low_freq, high_freq, amplitude, time=0):
        self.set_vals(low_freq, high_freq, amplitude, time)

    def GetData(self):
        rumble_data = [None] * 8
        if self.amp == 0.0:
            rumble_data[0] = 0x0
            rumble_data[1] = 0x1
            rumble_data[2] = 0x40
            rumble_data[3] = 0x40
        else:
            l_f = clamp(self.l_f, 40.875885, 626.286133)
            amp = clamp(self.amp, 0.0, 1.0)
            h_f = clamp(self.h_f, 81.75177, 1252.572266)
            hf = int((round(32.0 * math.log(h_f * 0.1, 2)) - 0x60) * 4)
            lf = int(round(32.0 * math.log(l_f * 0.1, 2)) - 0x40)
            hf_amp = None
            if amp == 0:
                hf_amp = 0
            elif amp < 0.117:
                hf_amp = int(((math.log(amp * 1000, 2) * 32) - 0x60) / (5 - pow(amp, 2)) - 1)
            elif amp < 0.23:
                hf_amp = int(((math.log(amp * 1000, 2) * 32) - 0x60) - 0x5c)
            else:
                hf_amp = int((((math.log(amp * 1000, 2) * 32) - 0x60) * 2) - 0xf6)

            assert hf_amp is not None
            lf_amp = int(round(hf_amp) * .5)
            parity = int(lf_amp % 2)
            if parity > 0:
                lf_amp -= 1

            lf_amp = int(lf_amp >> 1)
            lf_amp += 0x40
            if parity > 0:
                lf_amp |= 0x8000

            rumble_data[0] = int(hf & 0xff)
            rumble_data[1] = int((hf >> 8) & 0xff)
            rumble_data[2] = lf
            rumble_data[1] += hf_amp
            rumble_data[2] += int((lf_amp >> 8) & 0xff)
            rumble_data[3] = int(lf_amp & 0xff)
        for i in range(4):
            rumble_data[4 + i] = rumble_data[i]
        # Debug.Log(string.Format("Encoded hex freq: {0:X2}", encoded_hex_freq))
        # Debug.Log(string.Format("lf_amp: {0:X4}", lf_amp))
        # Debug.Log(string.Format("hf_amp: {0:X2}", hf_amp))
        # Debug.Log(string.Format("l_f: {0:F}", l_f))
        # Debug.Log(string.Format("hf: {0:X4}", hf))
        # Debug.Log(string.Format("lf: {0:X2}", lf))
        return bytes(rumble_data)


run = True

raspberry_pi_ip = '192.168.2.38'
factory = PiGPIOFactory(host=raspberry_pi_ip)
print("Raspberry Pi connected")

robby = Robot(left=(6, 5), right=(13, 12), pin_factory=factory)
print("Robby connected")


distance_sensor = DistanceSensor(echo=11, trigger=7, max_distance=1, threshold_distance=0.1, pin_factory=factory)
print("Distance sensor connected")

r_joycon_id = get_R_id()
l_joycon_id = get_L_id()

r_joycon = RumbleJoyCon(*r_joycon_id)
l_joycon = RumbleJoyCon(*l_joycon_id)

print("Joy-cons connected", r_joycon_id, l_joycon_id)


def rumble(joyconR, joyconL):
    freq = 320
    amp = 0.2
    data = RumbleData(freq/2, freq, amp)
    b = data.GetData()
    joyconR.send_rumble(b)
    joyconL.send_rumble(b)
    sleep(1.5)
    joyconR.rumble_stop()
    joyconL.rumble_stop()


def talk(text):
    speech = gTTS(text=text, lang='en')
    filename = 'speech.mp3'
    speech.save(filename)
    os.system(f'afplay {filename}')
    os.remove(filename)


def when_in_range():
    talk("Object detected")
    robby.stop()
    rumble(r_joycon, l_joycon)
    sleep(2)
    robby.backward()
    talk("Going backward")


def stop():
    robby.stop()
    print("stop")


def joy_con_mouvement(rjoycon, ljoycon):
    print("Program started")

    global run

    while run:
        lx = ljoycon.get_stick_left_vertical()
        ly = ljoycon.get_stick_left_horizontal()
        up = ljoycon.get_button_up()
        down = ljoycon.get_button_down()
        right = ljoycon.get_button_right()
        left = ljoycon.get_button_left()
        b = rjoycon.get_button_b()
        plus = rjoycon.get_button_plus()
        minus = ljoycon.get_button_minus()
        capture = ljoycon.get_button_capture()
        home = rjoycon.get_button_home()

        print('Distance: ', distance_sensor.distance * 100)

        if not distance_sensor.distance * 100 <= 10:
            if lx > 3000 or up or b:
                robby.forward()
                talk("Forward")

            else:
                stop()

            if lx < 2000 or down:
                robby.backward()
                talk("backward")

            else:
                stop()

            if ly > 3000 or right:
                robby.right()
                talk("right")

            else:
                stop()

            if ly < 1000 or left:
                robby.left()
                talk("left")

            else:
                stop()

            if plus or minus:
                robby.stop()
                mixer.music.stop()
                talk("Program stopped")
                run = False

            else:
                stop()

            if capture:
                mixer.music.load("music.mp3")
                mixer.music.play()

            else:
                stop()

            if home:
                mixer.music.stop()

            else:
                stop()

        else:
            when_in_range()


try:
    while run:
        joy_con_mouvement(r_joycon, l_joycon)

    # pause()
except KeyboardInterrupt:
    run = False
    robby.stop()
    print("Program stopped")
