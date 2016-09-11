import sys
import sys
import time
import math
from pymlab import config
from support import axis
from threading import Thread
import numpy as np

from astropy import units as u
from astropy.time import Time
from astropy.coordinates import SkyCoord  # High-level coordinates
from astropy.coordinates import ICRS, Galactic, FK4, FK5, AltAz  # Low-level frames
from astropy.coordinates import Angle, Latitude, Longitude  # Angles
from astropy.coordinates import EarthLocation

from astropy.utils import iers
#iers.IERS.iers_table = iers.IERS_A.open(iers.IERS_A_URL)


class drive(object):

    def __init__(self, profile = False, mode = "eq", steps_per_revA = 0, reverseA = False,
                steps_per_revB = 0, reverseB = False, speedMode = 'off', connectMethod = 'pymlab',
                connect = True, obs_lat = 48.986976, obs_lon = 14.467532, obs_alt = 300, port = '/dev/ttyUSB0'):

        self.observatory = EarthLocation(lat=obs_lat*u.deg, lon=obs_lon*u.deg, height=obs_alt*u.m)
        self.port = port
        self.profile = profile # profil montaze - pokud je preddefinovany nemusi se znovu nastavovat
        self.mount_mode = mode # 'eq' - equatorial, 'az' - azimuthal
        self.mount_steps = (steps_per_revA, steps_per_revB)
        self.mount_direction = (reverseA, reverseB)
        self.speedMode = speedMode # 'sidereal' - rychlost pohybu hvezd, 'off' - vypnuty hodinovy stroj, 'custom' - vlastni rychlost, 'solar', 'lunal', ...
        self.speedModeVelocity = 0
        self.connectMethod = connectMethod
        self.mount_position = SkyCoord(0, 0, frame = "icrs", unit="deg")
        self.home_position = SkyCoord(alt = obs_lat, az = 0, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.observatory)
        self.mount_position = self.home_position.icrs
        self.mount_target = False
        self.mount_slew = False
        self.park = True


        if connect:
            self.connect()

        #TODO moznost nastavit profil a pak teprve upravit parametry ze zadain
        if profile == 'HEQ5':
            self.mount_mode = 'eq'
            self.mount_steps = (2250429, 2250429) # osa RA zhruba zmerena na 13 238 879 kroku. bez nastaveni microsteppingu (tzn. 1/128), 
                                                  # 2250429 pri 1/16 microstep tzn. - cely registr je cca 1.9 otacky (mereno na RA)
            self.mount_direction = (False, False) # smer otaceni os

    def beep(self, duration = 500):
        self.spi.GPIO_write(0b11111111)
        time.sleep(duration/1000)
        self.spi.GPIO_write(0b00000000)

    def getObs(self):
        return self.observatory

    def movementMode(self, mode = False, velocity = 0):
        if mode:
            self.speedMode = mode
            self.speedModeVelocity = velocity
        print "movement Mode is", self.speedMode, " (", self.speedModeVelocity, ")"
        return (self.speedMode, self.speedModeVelocity)

    def mThread(self):
        while getattr(self, "run", True):
            AxA_mountDir, AxB_mountDir = self.mount_direction
            if self.park:
                print "park"
                self.AxA.Float()
                self.AxB.Float()
                self.AxA.GoHome(False)
                self.AxB.GoHome(False)
                while self.AxA.IsBusy() | self.AxB.IsBusy():
                    pass
            
            elif self.mount_slew:
                print ""
                print "slew mode"
                print "============"
                print "puvodni souradnice:", self.mount_position
                print "pozadovane souradnice:", self.mount_target

                position = self.mount_position
                target = self.mount_target
                self.mount_target = False

                ra = target.ra.degree - position.ra.degree
                dec = target.dec.degree - position.dec.degree
                
                ra = (ra + 180) % 360 - 180
                dec = (dec + 180) % 360 - 180

                print "korekce:", ra, dec

                self.AxA.Float()
                self.AxB.Float()
                time.sleep(0.5)

                pos_req = (ra*(self.mount_steps[0]/360.0), dec*(self.mount_steps[1]/360.0))

                deltaB = pos_req[0]
                deltaA = pos_req[1]

                print "DELTA #1", deltaB, deltaA

                self.AxB.Move(int(deltaA))
                self.AxA.Move(int(deltaB))



                while self.AxA.IsBusy() | self.AxB.IsBusy():
                    print "blaB", self.AxA.IsBusy(), self.AxB.IsBusy()
                print "##################################################################"

                while self.AxA.IsBusy() or self.AxB.IsBusy():
                    reg_a = self.AxA.ReadStatusReg()
                    reg_b = self.AxB.ReadStatusReg()
                    print reg_a, reg_b, ">>>>>", self.AxA.IsBusy(), self.AxB.IsBusy()
                    #print  "\t\t ", bin(reg_a[0])[2:].zfill(8), bin(reg_a[1])[2:].zfill(8), '\t\t', bin(reg_b[0])[2:].zfill(8), bin(reg_b[1])[2:].zfill(8)

                #print "puvodni poloha enkonderu je ", pos_old, "nova", pos_req
               

                #print 'steps:', ra *self.mount_steps[0]/360, dec*self.mount_steps[1]/360

                #while self.AxA.IsBusy() or self.AxB.IsBusy():
                #    pass

                print "Slew dokonceno"
                self.mount_slew = False
                self.mount_position = target

                self.AxA.Float()
                self.AxB.Float()
                time.sleep(0.5)
                self.speedMode = 'sidereal'

            elif self.speedMode == 'off':
                self.AxA.Float()
                self.AxB.Float()
                lastA = 0
                lastB = 0
                #print "off"

            elif self.speedMode == 'sidereal':
                newA = 50
                newB = 0
                #if lastA != newA or lastB != newB:
                self.AxA.Run(int(AxA_mountDir), 3)
                self.AxB.Float()
                #self.AxA.ReadStatusReg()
                #self.AxB.ReadStatusReg()
                lastA = 50
                lastB = 0
                reg_a = self.AxA.ReadStatusReg()
                reg_b = self.AxB.ReadStatusReg()
                print reg_a, reg_b
                #print bin(reg_a1), bin(reg_a2), bin(reg_b1), bin(reg_b2), "<<<"
                #print  "\t\t ", bin(reg_a[0])[2:].zfill(8), bin(reg_a[1])[2:].zfill(8), '\t\t', bin(reg_b[0])[2:].zfill(8), bin(reg_b[1])[2:].zfill(8)

                    #print self.AxA.Run(1, self.mount_steps[0]/(24*60*60))

    def run(self):
        print 'Zapinam vlakno'
        self.t1 = Thread(target = self.mThread)
        self.t1.setDaemon(True)
        self.t1.start()

    def connect(self):
        cfg = config.Config(
            i2c = {
                "port": self.port,
                "device": 'serial',
            },

            bus = [
                { 
                "name":"spi", 
                "type":"i2cspi"
                },
            ],
        )

        cfg.initialize()
        self.spi = cfg.get_device("spi")

        self.spi.route()
        self.spi.SPI_config(self.spi.I2CSPI_MSB_FIRST| self.spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| self.spi.I2CSPI_CLK_461kHz)

        time.sleep(.25)

        self.AxA = axis.axis(self.spi, self.spi.I2CSPI_SS1, 1, 1)    # set Number of Steps per axis Unit and set Direction of Rotation
        self.AxA.Reset()
        self.AxA.MaxSpeed(0x30ffff)                      # set maximal motor speed 


        self.AxB = axis.axis(self.spi, self.spi.I2CSPI_SS0, 1, 1)    
        self.AxB.Reset()
        self.AxB.MaxSpeed(0x30ffff)                      # set maximal motor speed


        self.AxB.MoveWait(2000)
        self.AxA.MoveWait(2000)
        self.AxB.MoveWait(-2000)
        self.AxA.MoveWait(-2000)

        

    def Slew(self, target):
        print "Slew to", target
        self.mount_target = target
        self.mount_slew = True

    def GoPark(self):
        self.park = True

    def UnPark(self):
        self.park = False

    def hardStop(self):
        self.speedMode = 'off'
        self.AxA.Run(0, 0)
        self.AxB.Run(0, 0)
        self.AxA.Float()
        self.AxB.Float()

    def setMode(self, mode):
        self.speedMode = mode
        return True



def main():
    try:
        driver = drive(profile = 'HEQ5')
        driver.run()
        driver.UnPark()

        driver.movementMode('sidereal')
        print "A"
        time.sleep(1)
        print "B"

        print "*************************"
        #driver.Slew(SkyCoord(alt = 1, az = 90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        driver.Slew(SkyCoord(alt = 5, az = 90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        time.sleep(50)

        driver.Slew(SkyCoord(alt = 10, az = 180+90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        time.sleep(5)

        driver.GoPark()
        #driver.Slew(SkyCoord(alt = 1, az = 90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        #driver.movementMode('sidereal')

        while True:
            pass

    except KeyboardInterrupt:
        print "Ukoncuji"
        driver.hardStop()
        print "Konec", Time.now()
    #drive.connect()


if __name__ == '__main__':
    main()