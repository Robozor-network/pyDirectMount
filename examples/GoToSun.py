from pydirectmount.drive import drive

from astropy.time import Time
from astropy.coordinates import get_sun
import time

from astropy.utils import iers
print "IERS sould be updated from", iers.IERS_A_URL
iers.IERS.iers_table = iers.IERS_A.open("/home/odroid/finals2000A.all")
#iers.IERS.iers_table = iers.IERS_A.open(iers.IERS_A_URL)


def main():
    try:
        mount = drive(profile = 'HEQ5', mode = "eq", connectMethod = 'pymlab',
            obs_lat = 48.986976, obs_lon = 14.467532, obs_alt = 300, port = '/dev/ttyUSB0')

        mount.run()
        mount.UnPark()
        mount.Slew(get_sun(Time.now()))    

        time.sleep(60)

        mount.GoPark()
        
        while True:
            pass

    except KeyboardInterrupt:
        mount.hardStop()
        print "Konec", Time.now()

if __name__ == '__main__':
    main()
