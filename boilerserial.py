import serial
import logging
import collections

class BoilerSerial:
    dev: serial.Serial

    status_field_dict = {
        'temperature_C': float,
        'boiler_state': int,
        'day_on_total_s': float,
        'month_on_total_s': float,
        'day_on_total_l': float,
        'month_on_total_l': float,
        'on_counter': int,
    }

    measurement_field_dict = {
        'boiler_fuel_consumed_l': float,
        'boiler_on_time_s': float
    }

    StatusFields = collections.namedtuple("StatusFields", status_field_dict.keys())
    MeasurementFields = collections.namedtuple("MeasurementFields", measurement_field_dict.keys())

    status: StatusFields
    measurement: MeasurementFields

    def __init__(self, port):
        self.log = logging.getLogger("boilerserial")
        self.log.info("Initializing boilerserial with port %s" % port)

        self.dev = serial.Serial(port, baudrate=1000000, timeout=0.1)

        self.temp_status_fields = {}
        self.temp_measurement_fields = {}

        self.status = ()
        self.measurement = ()

    def _set_temp_measurement_field(self, field, value):
        # convert value to field type
        value = self.measurement_field_dict[field](value)
        self.temp_measurement_fields[field] = value
        if len(self.temp_measurement_fields.keys()) == len(self.measurement_field_dict.keys()):
            self.measurement = self.MeasurementFields(**self.temp_measurement_fields)
            self.temp_measurement_fields = {}
            return self.measurement
        return None

    def _set_temp_status_field(self, field, value):
        # convert value to field type
        value = self.status_field_dict[field](value)
        self.temp_status_fields[field] = value
        if len(self.temp_status_fields.keys()) == len(self.status_field_dict.keys()):
            self.status = self.StatusFields(**self.temp_status_fields)
            self.temp_status_fields = {}
            return self.status
        return None

    def read(self):

        ret = []
        while self.dev.in_waiting > 0:
            line = self.dev.readline(128)
            if not line:
                break

            line = line.decode('ascii').strip()
            logging.debug("received: %s" % line)

            try:
                if "=" not in line:
                    continue

                field, value = line.split("=")
                if field in self.measurement_field_dict.keys():
                    m = self._set_temp_measurement_field(field, value)
                    if m:
                        ret.append(m)
                elif field in self.status_field_dict.keys():
                    s = self._set_temp_status_field(field, value)
                    if s:
                        ret.append(s)
                else:
                    # log un-processed lines as that is debug information
                    self.log.info("received: %s" % line)

            except ValueError as e:
                self.log.error("Exception on handling '%s'" % line, exc_info=e)

        return ret


if __name__ == '__main__':
    import time
    logging.basicConfig(level=logging.INFO)
    bs = BoilerSerial("/dev/ttyUSB0")

    while True:
        try:
            ret = bs.read()
            for v in ret:
                logging.info(v)
            time.sleep(0.1)
        except KeyboardInterrupt:
            break

