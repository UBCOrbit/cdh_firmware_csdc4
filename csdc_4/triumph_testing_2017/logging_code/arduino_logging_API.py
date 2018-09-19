import serial

class uart_connection(object):
    __OUTPUT_PINS = -1

    def __init__(self, port, baudrate=115200):
        self.serial = serial.Serial(port, baudrate, timeout=1)

    def __str__(self):
        return "Arduino is on port %s at %d baudrate" % (self.serial.port, self.serial.baudrate)

    def getData(self):

        line = ''
        try:
            line = self.serial.readline().replace("\r\n", "")
        except Exception, e:
            return ''
        if line:
            return line
        else:
            return ''
