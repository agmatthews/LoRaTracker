#
class L76GNSS:

    GPS_I2CADDR = const(0x10)

    def __init__(self, pytrack=None, sda='P22', scl='P21'):
        if pytrack is not None:
            self.i2c = pytrack.i2c
        else:
            from machine import I2C
            self.i2c = I2C(0, mode=I2C.MASTER, pins=(sda, scl))

        self.reg = bytearray(1)
        self.i2c.writeto(GPS_I2CADDR, self.reg)

    def _read(self):
        self.reg = self.i2c.readfrom(GPS_I2CADDR, 1) # was 64
        return self.reg
