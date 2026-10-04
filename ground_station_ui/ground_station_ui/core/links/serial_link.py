"""Link qua cong serial (radio SiK cam USB)."""

import serial

from ground_station_ui.core.links.base import Link


class SerialLink(Link):
    kind = 'serial'

    def __init__(self, name, cfg, on_messages, on_state):
        super().__init__(name, cfg, on_messages, on_state)
        self.port = cfg.get('port', '')
        self.baud = int(cfg.get('baud', 57600))

    def configure(self, port, baud):
        self.port, self.baud = port, int(baud)

    def describe(self):
        return f'{self.port} @ {self.baud}'

    def _open(self):
        if not self.port:
            raise OSError('no port selected')
        try:
            return serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.5)
        except serial.SerialException as e:
            raise OSError(str(e)) from e

    def _read(self, ser):
        try:
            return ser.read(ser.in_waiting or 1)
        except serial.SerialException as e:
            raise OSError(str(e)) from e

    def _write(self, ser, data):
        try:
            ser.write(data)
        except serial.SerialException as e:
            raise OSError(str(e)) from e

    def _close(self, ser):
        ser.close()
