import time
from influxdb import InfluxDBClient
from boilerserial import BoilerSerial
import argparse
import datetime

import logging
logging.basicConfig(format='%(asctime)s %(levelname)s %(message)s', level=logging.DEBUG)


class BoilerDaemon:
    bs: BoilerSerial
    log: logging.Logger
    influxclient: InfluxDBClient

    def __init__(self):
        self.log = logging.getLogger("boilerdaemon")
        self._parse_arguments()

    def _parse_arguments(self):
        parser = argparse.ArgumentParser(description="Boiler serial client to infludxdb daemon")
        parser.add_argument("--port", type=str, help="Serial port", required=True)

        parser.add_argument("--influxdb-hostname", type=str, default="127.0.0.1")
        parser.add_argument("--influxdb-username", type=str)
        parser.add_argument("--influxdb-password", type=str)
        parser.add_argument("--influxdb-db", type=str, default='boiler')

        args = parser.parse_args()

        self.log.info("Boiler connection: %s" % args.port)
        self.log.info("InfluxDb connection: db: %s, host: %s, user: %s, password:***" %
                      (args.influxdb_db, args.influxdb_hostname, args.influxdb_username))

        self.bs = BoilerSerial(args.port)
        self.influxclient = InfluxDBClient(host=args.influxdb_hostname,
                                           username=args.influxdb_username,
                                           password=args.influxdb_password)
        self.influxclient.switch_database(args.influxdb_db)

        self.log.debug("arguments parsed")

    def run(self):
        """Main loop"""
        prev_insertion_dt = datetime.datetime.now()
        prev_boiler_state = None
        while True:
            try:
                ret = self.bs.read()
                for v in ret:
                    logging.info(v)

                    if isinstance(v, BoilerSerial.MeasurementFields):
                        self.log.info("Inserting measurement point")
                        p = [
                            {"measurement": "boiler_consumption",
                             "time": None,
                             "fields": {
                                 "on_time_s": v.boiler_on_time_s,
                                 "fuel_consumed_l": v.boiler_fuel_consumed_l
                             }
                             },
                        ]
                        self.influxclient.write_points(p)
                    if isinstance(v, BoilerSerial.StatusFields):
                        # insert a new measurement either every 5 minutes or when boiler state changes
                        if prev_boiler_state != v.boiler_state or (datetime.datetime.now() - prev_insertion_dt).total_seconds() > 60 * 5:
                            self.log.info("Inserting status point")
                            p = [
                                {"measurement": "boiler_status",
                                 "time": None,
                                 "fields": {
                                     "outside_temperature": v.temperature_C,
                                     "boiler_state": v.boiler_state,
                                     "on_counter": v.on_counter
                                     }
                                 },
                            ]
                            self.influxclient.write_points(p)
                            prev_insertion_dt = datetime.datetime.now()
                            prev_boiler_state = v.boiler_state
                time.sleep(0.1)
            except KeyboardInterrupt:
                break


if __name__ == "__main__":
    bd = BoilerDaemon()
    bd.run()
