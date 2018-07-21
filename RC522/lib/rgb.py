import pycom
__version__ = '1.0.0'

# from https://forum.pycom.io/topic/3312/lopy-pytrack-deepsleep-wake-then-lock-until-power-removed

class RGBLED:

	RED=const(0)
	ORANGE=const(30)
	ORANGE_YELLOW=const(45)
	YELLOW=const(60)
	YELLOW_GREEN=const(90)
	GREEN=const(120)
	GREEN_CYAN=const(165)
	CYAN=const(180)
	CYAN_BLUE=const(210)
	BLUE=const(240)
	BLUE_MAGENTA=const(275)
	MAGENTA=const(300)
	PINK=const(350)
	WHITE=const(360)

	def __init__(self, brightness):
		pycom.heartbeat(False)
		self.brightness = brightness
		self.saturation = 100

	def hsv_to_rgb(self,h,s,v):
		if s == 0.0:
		    return v, v, v
		i = int(h*6.0)
		f = (h*6.0) - i
		p = v*(1.0 - s)
		q = v*(1.0 - s*f)
		t = v*(1.0 - s*(1.0-f))
		i = i%6
		if i == 0:
		    return v, t, p
		if i == 1:
		    return q, v, p
		if i == 2:
		    return p, v, t
		if i == 3:
		    return p, q, v
		if i == 4:
		    return t, p, v
		if i == 5:
		    return v, p, q

	def hsv(self,h,s,v):
		r, g, b = self.hsv_to_rgb(h/360.0, s/100.0, v/100.0)
		pycom.rgbled(int(r*255)<<16 | int(g*255)<<8 | int(b*255))

	def hl(self,h,v):
		self.hsv(h,self.saturation,v)

	def h(self,h):
		self.hsv(h,self.saturation, self.brightness)

	def off(self):
		pycom.rgbled(0)
