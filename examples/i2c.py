#!/usr/bin/python

#uncomment for debbug purposes
#import logging
#logging.basicConfig(level=logging.DEBUG) 

import sys
import time
from pymlab import config

from pydirectmount.drive import drive

from astropy.time import Time
from astropy import units as u
from astropy.coordinates import get_sun, EarthLocation, SkyCoord
import time

from astropy.utils import iers
#print "IERS sould be updated from", iers.IERS_A_URL
#iers.IERS.iers_table = iers.IERS_A.open(iers.IERS_A_URL)
print("LOADED modules")

cfg = config.Config(
    i2c = {
        "port": 0,
        "device": 'hid'
    },

    bus = [
        {
            "name":"spi",
            "type":"i2cspi"
        },
    ],
)

cfg.initialize()

spi = cfg.get_device("spi")

try:
    print "SPI configuration.."
    spi.SPI_config(spi.I2CSPI_MSB_FIRST| spi.I2CSPI_MODE_CLK_IDLE_LOW_DATA_EDGE_LEADING| spi.I2CSPI_CLK_461kHz)
    print("config")
    #time.sleep(2)

    mount = drive(profile = 'HEQ5', mode = "eq", connectMethod = 'pymlab',  connectInstance = spi,
            obs_lat = 48.986976, obs_lon = 14.467532, obs_alt = 300)

    mount.run()
    mount.UnPark()
    
    #mount.Slew(get_sun(Time.now()))

    observatory = EarthLocation(lat=49*u.deg, lon=15*u.deg, height=300*u.m)
    
    pos = SkyCoord(alt = 0, az = 90*3, obstime = Time.now(), frame = 'altaz', unit="deg", location = observatory).icrs
    mount.Slew(pos)

    #mount.set_tracking(mode='custom', ra = 0, dec=0)
    #mount.set_tracking(mode='custom', ra = 0, dec=30000)

    #mount.m_dec.Run(100)

    while True:
        time.sleep(1)
        #print(mount.get_motor_steps())
        #(a, b) = mount.get_motor_status()
        #print(a.get('POSITION', "--"), a.get('VIRTUAL_POSITION', "--"), a.get('VIRTUAL_ROUND', "--"), b.get('POSITION', "--"),b.get('VIRTUAL_POSITION', "--"), b.get('VIRTUAL_ROUND', "--"))
    #time.sleep(10)

    #mount.GoPark()

finally:
    print "stop"
