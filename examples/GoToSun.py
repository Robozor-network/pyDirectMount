from pydirectmount.drive import drive

from astropy.time import Time
from astropy.coordinates import get_sun
import time

def main():
    try:
        mount = drive(profile = 'HEQ5', mode = "eq", connectMethod = 'pymlab',
            obs_lat = 48.986976, obs_lon = 14.467532, obs_alt = 300, port = '/dev/ttyUSB0')

        mount.run()
        mount.UnPark()
        mount.Slew(get_sun(Time.now()))    

        time.sleep(60)

        mount.GoPark()

    except KeyboardInterrupt:
        mount.hardStop()
        print "Konec", Time.now()

if __name__ == '__main__':
    main()