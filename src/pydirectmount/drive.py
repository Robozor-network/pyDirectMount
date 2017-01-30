#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import sys
import time
import datetime
import math
from pymlab import config
from support import axis
from threading import Thread
import numpy as np
import rospy
from arom.srv import *
from arom.msg import *

import ephem
from astropy import units as u
from astropy.time import Time
from astropy.coordinates import SkyCoord  # High-level coordinates
from astropy.coordinates import ICRS, Galactic, FK4, FK5, AltAz  # Low-level frames
from astropy.coordinates import Angle, Latitude, Longitude  # Angles
from astropy.coordinates import EarthLocation
from astropy.coordinates import get_sun

#from astropy.utils import iers
#print "IERS sould be updated from", iers.IERS_A_URL
#iers.IERS.iers_table = iers.IERS_A.open("/home/odroid/finals2000A.all")

#from astroplan import download_IERS_A
#download_IERS_A()



class drive(object):

    def __init__(self, profile = False, mode = "eq", steps_per_revA = 0, reverseA = False,
                steps_per_revB = 0, reverseB = False, speedMode = 'off', connectMethod = 'pymlab',
                connect = True, obs_lat = 48.986976, obs_lon = 14.467532, obs_alt = 300, port = '/dev/ttyUSB0'):

        self.driver_action_buffer = [{'home':True}]
        self.driver_action_now = 0 # kolikata akce v predchozim poli se prave vykonava
        self.driver_action_status = 1 # v jakem stavu je aktualni akce - 0=dokonceno; 1=prubeh ale lze ihned prerusit (napr. tracking, home); 2=prubet a je zamceno cizi aktivitou (napr. fotak), 3=prubeh a nelze prerusit (napr slew)


        self.observatory = EarthLocation(lat=obs_lat*u.deg, lon=obs_lon*u.deg, height=obs_alt*u.m)
        self.port = port
        self.profile = profile # profil montaze - pokud je preddefinovany nemusi se znovu nastavovat
        self.mount_mode = mode # 'eq' - equatorial, 'az' - azimuthal
        self.mount_steps = (steps_per_revA, steps_per_revB)
        self.mount_direction = (reverseA, reverseB)
        self.speedMode = speedMode # 'sidereal' - rychlost pohybu hvezd, 'off' - vypnuty hodinovy stroj, 'custom' - vlastni rychlost, 'solar', 'lunal', ...
        self.trackingSpeed = [90, 0]
        self.speedModeVelocity = 0
        self.connectMethod = connectMethod
        self.mount_position = SkyCoord(0, 0, frame = "icrs", unit="deg")
        self.home_position = SkyCoord(alt = obs_lat, az = 0, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.observatory)
        self.mountCoord = self.mount_position
        self.mount_position = self.home_position.icrs
        self.mount_target = False
        self.mount_slew = False
        self.park = True
        self.realAxisPos = (0,0)


        self.ra_kp = 0.0425
        self.ra_ip = 0.08
        self.Kd = 0.08
        self.ra_Derivator=0
        self.ra_Integrator=0
        self.ra_Integrator_max = 10
        self.ra_Integrator_min = 10
        self.ra_error=0.0

        self.dec_kp = 0.0425
        self.dec_ip = 0.08
        self.Kd = 0.08
        self.dec_Derivator=0
        self.dec_Integrator=0
        self.dec_Integrator_max = 10
        self.dec_Integrator_min = 10
        self.dec_error=0.0


        if connect:
            self.connect()

        #TODO moznost nastavit profil a pak teprve upravit parametry ze zadain
        if profile == 'HEQ5':
            self.mount_mode = 'eq'
            self.mount_steps = (2250429, 2250429) # osa RA zhruba zmerena na 13 238 879 kroku. bez nastaveni microsteppingu (tzn. 1/128), 
                                                  # 2250429 pri 1/16 microstep tzn. - cely registr je cca 1.9 otacky (mereno na RA)
            self.mount_direction = (False, False) # smer otaceni os

        self.mount_stepsperdeg = (self.mount_steps[0]/360.0, self.mount_steps[1]/360.0)

    def new_action(self, action, data=None):
        self.driver_action_buffer.append({action: data})

    def beep(self, duration = 500):
        self.spi.GPIO_write(0b11111111)
        time.sleep(duration/1000)
        self.spi.GPIO_write(0b00000000)

    def getObs(self):
        return self.observatory

    def movementMode(self, mode = False, velocity = 0):
        self.driver_action_buffer.append({'tracking': {'mode': mode} })
        if mode:
            self.speedMode = mode
            self.speedModeVelocity = velocity
        print "movement Mode is", self.speedMode, " (", self.speedModeVelocity, ")"
        return (self.speedMode, self.speedModeVelocity)

    def getStepperPosition(self, coordinates):
        pass

    def GetHaDecCoordinates(self, coordinates):                
        #now = datetime.datetime.utcnow()
        #midnight = now.replace(hour=0, minute=0, second=0, microsecond=0)
        #HAvariable = ((now - midnight).seconds/(24*60*60.0))/360.0 - self.observatory.longitude.degree

        t2 = Time(str(Time.now()), scale='utc', location=self.observatory)
        HAvariable = t2.sidereal_time('mean').degree

        ha = Angle((HAvariable - coordinates.ra.degree), unit="deg").wrap_at('180d')
        dec = Angle(coordinates.dec.degree, unit="deg").wrap_at('180d')

        if ha.is_within_bounds('-180d', '0d'):
            #print "mode: -180 ~ 0"
            ha =  - (ha.degree + 90)
            dec = - (dec.degree - 90)

        elif ha.is_within_bounds('0d', '180d'):
            #print "mode: 0 ~ 180"
            ha  = - (ha.degree - 90)
            dec =  (dec.degree - 90)
        else:
            print "Err --- out of range", ha, dec
            ha = None
            dec = None

        return (ha, dec)



    def run(self):
        print 'Zapinam vlakno'
        self.t1 = Thread(target = self.mThread)
        self.t1.setDaemon(True)
        self.t1.start()

    def connect(self):
        
        self.pymlab = rospy.ServiceProxy('pymlab_drive', PymlabDrive)
        
        print eval(self.pymlab(device="telescope_spi", method="route").value)
        print eval(self.pymlab(device="telescope_spi", method="SPI_config", parameters="0b1001").value) # self.spi.I2CSPI_MSB_FIRST| self.spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| self.spi.I2CSPI_CLK_461kHz

        time.sleep(.25)
        print self.pymlab
        self.AxA = axis.axis(self.pymlab, 0b0001, 1, 1, protocol = 'arom')    # set Number of Steps per axis Unit and set Direction of Rotation
        self.AxA.Reset()
        #self.AxA.MaxSpeed(0x33ffff)                      # set maximal motor speed 
        self.AxA.MaxSpeed(0x33ff)

        self.AxB = axis.axis(self.pymlab, 0b0010, 1, 1, protocol = 'arom')    
        self.AxB.Reset()
        #self.AxB.MaxSpeed(0x33ffff)                      # set maximal motor speed
        self.AxB.MaxSpeed(0x33ff)

        print "Movement Test"
        self.AxA.MoveWait(2000)
        self.AxA.MoveWait(-2000)
        self.AxB.MoveWait(2000)
        self.AxB.MoveWait(-2000)
        

    def Slew(self, target):
        self.driver_action_buffer.append({'slew': {'target': target} })
        print "Slew to", target
        self.mount_target = target
        self.mount_slew = True

    #TODO: support for TLE tracking
    '''
    def GetCoordByTLE(self, name):

        degrees_per_radian = 180.0 / math.pi
        home = ephem.Observer()
        home.lon = self.observatory.longitude.degree
        home.lat = self.observatory.latitude.degree
        home.elevation = 300 # meters 

        # Always get the latest ISS TLE data from:
        # http://spaceflight.nasa.gov/realdata/sightings/SSapplications/Post/JavaSSOP/orbit/ISS/SVPOST.html
        iss = ephem.readtle('ISS',
            '1 39433U 13066T   16320.81582129 +.00000905 +00000-0 +96151-4 0  9996',
            '2 39433 097.6817 023.9463 0030935 203.5994 156.3807 14.91124701161032'
        )

        home.date = datetime.datetime.utcnow()
        iss.compute(home)
        icrs = SkyCoord(alt = iss.alt*degrees_per_radian, az = iss.az*degrees_per_radian, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.observatory).icrs

        #self.driver_action_buffer.append({'slewTLE': {'object': name, 'SkyCoord': altaz} })
        return icrs

    def StartTrackingTLE(self, name):
        print "TLE tracking"

        #self.GetCoordByTLE(name)
        self.mount_target = self.GetCoordByTLE('iss')

        home = ephem.Observer()
        home.lon = self.observatory.longitude.degree
        home.lat = self.observatory.latitude.degree
        home.elevation = 300 # meters 

        # Always get the latest ISS TLE data from:
        # http://spaceflight.nasa.gov/realdata/sightings/SSapplications/Post/JavaSSOP/orbit/ISS/SVPOST.html
        iss = ephem.readtle('ISS',
            '1 39433U 13066T   16320.81582129 +.00000905 +00000-0 +96151-4 0  9996',
            '2 39433 097.6817 023.9463 0030935 203.5994 156.3807 14.91124701161032'
        )

        home.date = datetime.datetime.utcnow()
        iss.compute(home)

        degrees_per_radian = 180.0 / math.pi
        issNow = (iss.alt * degrees_per_radian, iss.az * degrees_per_radian)
        print "ISS alt, az:", issNow
        print "ISS rise, transit, set:", iss.rise_time, iss.transit_time, iss.set_time
        print iss
        print iss.range_velocity/1000, "km/s ", iss.range/1000, "km"

        if issNow[0] < 10:
            print "ISS je pod obzorem"
        else:
            print "ISS je nad obzorem"

        self.speedMode = 'orbit'

        self.driver_action_buffer.append({'tracking': {'mode': 'tle', 'object': name, 'position': issNow} })
        '''

    def GoPark(self):
        self.park = True
        self.driver_action_buffer.append({'home': True})

    def UnPark(self):
        self.park = False
        #self.driver_action_buffer.append({'tracking': False})

    def hardStop(self):
        self.driver_action_buffer.append({'stop': {'mode': 'hard'}})
        self.speedMode = 'off'
        self.AxA.Run(0, 0)
        self.AxB.Run(0, 0)
        self.AxA.Float()
        self.AxB.Float()

    def setMode(self, mode):
        self.driver_action_buffer.append({'tracking_mode': {'mode': mode}})
        self.speedMode = mode
        return True

    def tracking(self, state):
        print "tracking:", state
        if state:
            self.driver_action_status = 1
            self.driver_action_buffer.append({'tracking': {'mode': 'sidereal'} })
        else:
            self.hardStop()

    def setTrackingSpeed(self, ra, dec):
        print "tracking speed:", ra, dec
        self.trackingSpeed[0] = float(dec)
        self.trackingSpeed[1] = float(ra)

    def setMaxSpeed(self, spd):
        print "maximal speed:", spd
        self.AxA.MaxSpeed(int(spd))
        self.AxB.MaxSpeed(int(spd))

    def getCoordinates(self, mode = "RaDec"):
        try:
            return self.mountCoord
            #return self.mount_position
        except Exception, e:
            return None

    def getAxisPosition(self):
        axa = self.AxA.ReadPosition()
        axb = self.AxB.ReadPosition()


        print "*******************************************"
        print "raw", (axa, axb)
        print "count1C ", ( Angle(90*u.deg)-Angle(axa/self.mount_stepsperdeg[0], unit="deg").wrap_at('360d') , Angle(90, unit="deg")-(Angle(axb/self.mount_stepsperdeg[1], unit="deg").wrap_at('360d')) ),              Angle((90*u.deg)-Angle(axa/self.mount_stepsperdeg[0], unit="deg").wrap_at('360d')).to_string(unit=u.hour)
        print "count1D ", ( Angle(90*u.deg)-Angle(axa/self.mount_stepsperdeg[0], unit="deg").wrap_at('360d')+(180*u.deg) , Angle(90, unit="deg")-(Angle(axb/self.mount_stepsperdeg[1], unit="deg").wrap_at('360d')) ),        Angle((90*u.deg)-Angle(axa/self.mount_stepsperdeg[0], unit="deg").wrap_at('360d')+(180*u.deg)).to_string(unit=u.hour)

        out_a = Angle(0*u.deg)
        out_b = Angle(0*u.deg)


        if axa < (2**22)/4:
            x_ha = axa/self.mount_stepsperdeg[0]
            if axb < (2**22)/4:
                print "SEKTOR I (0-90)"
                out_a = Angle(-1*(x_ha+90)*u.deg).wrap_at('360d')
            else:
                print "SEKTOR III (180-270)"
                out_a = Angle(-1*(x_ha-90)*u.deg).wrap_at('360d')
        else:
            x_ha = (2**22-axa)/self.mount_stepsperdeg[0]
            if axb < (2**22)/4:
                print "SEKTOR II (90-180)"
                out_a = Angle(x_ha*u.deg+270*u.deg)
            else:
                print "SEKTOR IV (270-360)"
                out_a = Angle(x_ha*u.deg)



        LST = Time(str(Time.now()), scale='utc', location=self.observatory).sidereal_time('mean')
        print LST
        print Angle(LST)
        print out_a
        print Angle(out_a)
        try:
            print Angle(LST) - Angle(out_a)
            self.mountCoord = SkyCoord(ra = Angle(Angle(LST) - Angle(out_a)), dec = self.mount_position.dec, obstime = Time.now(), unit="deg", frame = "icrs", location = self.observatory)
            print Angle(out_b)

        except Exception, e:
            print "ERRRRRRRRRR"
            print e
            self.mountCoord = SkyCoord(ra = 0, dec = 0, obstime = Time.now(), unit="deg", frame = "icrs", location = self.observatory)
        
        print self.mountCoord
        return self.mountCoord

    def realAxisCoord(self, AxisPos):
        ra, dec = AxisPos
        ha_c = ra/self.mount_stepsperdeg[0]
        dec_c= dec/self.mount_stepsperdeg[1]

        return (ha_c, dec_c)

    def getStatus(self):
        pass

    def mThread(self):
        ha = 0
        while getattr(self, "run", True):
            print "ActBuff:", len(self.driver_action_buffer), self.driver_action_now,  self.driver_action_buffer[self.driver_action_now-1]
            driver_action, driver_action_params = self.driver_action_buffer[self.driver_action_now-1].items()[0]

            if self.driver_action_status in [0,1] and len(self.driver_action_buffer) > self.driver_action_now:
                # kdyz je status akce je 0 nebo 1 (dokonceno nebo lze prerusit) a v bufferu je nejaka dalsi akce
                # tak prejdu na dalsi akci
                self.driver_action_now += 1
                print "Mam novou akci - prechazim na dalsi:", self.driver_action_now, self.driver_action_buffer[self.driver_action_now-1]

            AxA_mountDir, AxB_mountDir = self.mount_direction

            self.realAxisPos = (self.AxA.ReadPosition(),self.AxB.ReadPosition())
            axis_ha, axis_ra = self.realAxisCoord(self.realAxisPos)
            

            if driver_action == 'home':
                self.driver_action_status = 3
                print "park"
                self.AxA.GoHome(False)
                self.AxB.GoHome(False)
                while self.AxA.IsBusy() | self.AxB.IsBusy():
                    time.sleep(0.35)
                self.driver_action_status = 0
            
            elif driver_action == 'slew':
                self.driver_action_status = 3 # pracuji - neprerusovat
                print ""
                print ""
                print "slew mode"
                print "============"
                print "pozadovane souradnice: ", self.mount_target.to_string('hmsdms')
                print "pozadovane souradnice: ", self.mount_target.to_string('dms')
                print "AzAlt                : ", self.mount_target.transform_to(AltAz(obstime = Time.now(), location=self.observatory)).to_string('dms')

                target = self.mount_target

                ha, dec = self.GetHaDecCoordinates(target)

                print "Ha [deg]  [h]        : ", ha, ha/15
                print "Pozice montaze ra,dec:", ha *self.mount_stepsperdeg[0], dec *self.mount_stepsperdeg[1]
                print ""

                self.AxA.GoTo(int(ha *self.mount_stepsperdeg[0]))
                self.AxB.GoTo(int(dec*self.mount_stepsperdeg[1]))

                while self.AxA.IsBusy() or self.AxB.IsBusy():
                    reg_a = self.AxA.ReadStatusReg()
                    reg_b = self.AxB.ReadStatusReg()
                    print reg_a, reg_b
                    self.getAxisPosition()
                    time.sleep(0.5)


                print "Slew dokonceno"
                self.mount_position = target

                #self.AxA.Float()
                #self.AxB.Float()
                #time.sleep(0.5)
                #self.speedMode = 'sidereal'

                self.driver_action_status = 1
                self.driver_action_buffer.append({'tracking': {'mode': 'sidereal'} })

            elif driver_action == 'tracking' and self.driver_action_status >= 1:
                print "tracking", driver_action_params['mode'], driver_action, self.driver_action_status >= 1
                tracking_mode = driver_action_params['mode']
                self.driver_action_status = 1
                if tracking_mode == False:
                    lastA = 0
                    lastB = 0

                elif tracking_mode == 'sidereal':
                    newA = 50
                    newB = 0
                    #if lastA != newA or lastB != newB:
                    #self.AxA.Run(int(AxA_mountDir), 1.2)
                    #self.AxB.Float()
                    lastA = 50
                    lastB = 0
                    

                    if self.mount_position is not None:
                        print self.driver_action_status

                        time.sleep(1)
                        '''
                        ha, dec = self.GetHaDecCoordinates(target)
                        self.error = -((int(ha*self.mount_stepsperdeg[0])+2**21)-(self.AxA.ReadPosition()-2**21))*10
                        print "pozadovana pozice", ha, "HA:", int(ha*self.mount_stepsperdeg[0])+2**21, "realna pozice", self.AxA.ReadPosition()-2**21, "rozdil:", self.error

                        
                        self.ra_P_val = self.ra_kp * self.error
                        self.ra_Derivator = self.error

                        self.ra_Integrator = self.ra_Integrator + self.error

                        if self.ra_Integrator > self.ra_Integrator_max:
                            self.ra_Integrator = self.ra_Integrator_max
                        elif self.ra_Integrator < self.ra_Integrator_min:
                            self.ra_Integrator = self.ra_Integrator_min

                        self.ra_I_val = self.ra_Integrator * self.ra_ip

                        self.ra_D_val = self.ra_kp * (self.error - self.ra_Derivator)
                        
                        ra_PID = self.ra_P_val + self.ra_I_val + self.ra_D_val

                        ra_PID = ra_PID
                        print ra_PID, self.ra_P_val, self.ra_I_val 

                        if ra_PID > 100:
                            ra_PID = 100
                            dire = AxA_mountDir
                        elif ra_PID >= 0:
                            dire = AxA_mountDir
                        elif ra_PID < -100:
                            ra_PID = -100
                            dire = not AxA_mountDir
                        elif ra_PID < 0:
                            dire = not AxA_mountDir

                        ra_PID = 60
                        print AxA_mountDir, abs(ra_PID)
                        '''
                        print int(AxA_mountDir), self.trackingSpeed
                        #self.AxA.Run(AxA_mountDir, 0b1000000000000)
                        self.AxA.Run(AxA_mountDir, self.trackingSpeed[0])
                        self.AxB.Run(AxA_mountDir, self.trackingSpeed[1])
                        self.driver_action_status = 0

                elif tracking_mode == 'orbit':

                    if self.mount_position is not None and True:

                        time.sleep(1)

                        '''
                        ha, dec = self.GetHaDecCoordinates(self.GetCoordByTLE('ISS'))
                        self.ra_error = -((int(ha*self.mount_stepsperdeg[0])+2**21)-(self.AxA.ReadPosition()-2**21))*10
                        self.dec_error = -((int(ha*self.mount_stepsperdeg[1])+2**21)-(self.AxB.ReadPosition()-2**21))*10

                        print "pozadovana pozice", ha, "HA:", int(ha*self.mount_stepsperdeg[0])+2**21, "realna pozice", self.AxA.ReadPosition()-2**21, "rozdil:", self.ra_error

                        self.ra_P_val = self.ra_kp * self.ra_error
                        self.ra_Derivator = self.ra_error

                        self.ra_Integrator = self.ra_Integrator + self.ra_error

                        if self.ra_Integrator > self.ra_Integrator_max:
                            self.ra_Integrator = self.ra_Integrator_max
                        elif self.ra_Integrator < self.ra_Integrator_min:
                            self.ra_Integrator = self.ra_Integrator_min

                        self.ra_I_val = self.ra_Integrator * self.ra_ip

                        self.ra_D_val = self.ra_kp * (self.ra_error - self.ra_Derivator)
                        
                        ra_PID = self.ra_P_val + self.ra_I_val + self.ra_D_val

                        ra_PID = ra_PID
                        print ra_PID, self.ra_P_val, self.ra_I_val 

                        if ra_PID > 1000:
                            ra_PID = 1000
                            dire = AxA_mountDir
                        elif ra_PID >= 0:
                            dire = AxA_mountDir
                        elif ra_PID < -1000:
                            ra_PID = -1000
                            dire = not AxA_mountDir
                        elif ra_PID < 0:
                            dire = not AxA_mountDir
                        print AxA_mountDir, abs(ra_PID)
                        self.AxA.Run(int(dire), abs(ra_PID)/10)




                        self.dec_P_val = self.dec_kp * self.dec_error
                        self.dec_Derivator = self.dec_error

                        self.dec_Integrator = self.dec_Integrator + self.dec_error

                        if self.dec_Integrator > self.dec_Integrator_max:
                            self.dec_Integrator = self.dec_Integrator_max
                        elif self.dec_Integrator < self.dec_Integrator_min:
                            self.dec_Integrator = self.dec_Integrator_min

                        self.dec_I_val = self.dec_Integrator * self.dec_ip

                        self.dec_D_val = self.dec_kp * (self.dec_error - self.dec_Derivator)
                        
                        dec_PID = self.dec_P_val + self.dec_I_val + self.dec_D_val

                        dec_PID = dec_PID
                        print dec_PID, self.dec_P_val, self.dec_I_val 

                        if dec_PID > 90000:
                            dec_PID = 90000
                            dire = AxA_mountDir
                        elif dec_PID >= 0:
                            dire = AxA_mountDir
                        elif dec_PID < -90000:
                            dec_PID = -90000
                            dire = not AxA_mountDir
                        elif dec_PID < 0:
                            dire = not AxA_mountDir
                        print AxB_mountDir, abs(dec_PID)
                        self.AxB.Run(int(dire), abs(dec_PID)/10)
                        '''


def main():
    try:
        driver = drive(profile = 'HEQ5')
        driver.run()
        driver.UnPark()

        driver.movementMode('sidereal')
        print "A"
        time.sleep(1)
        print "B"

        #driver.Slew(SkyCoord(alt = 1, az = 90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)
        #driver.Slew(SkyCoord(ra = 1.3, dec = 0, obstime = Time.now(), frame = 'icrs', unit="deg", location = driver.getObs()))
        driver.Slew(SkyCoord(alt = 45, az = 45, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        time.sleep(1)
        driver.GoPark()
        
        while True:
            pass

    except KeyboardInterrupt:
        print "Ukoncuji"
        driver.hardStop()
        print "Konec", Time.now()
    #drive.connect()


if __name__ == '__main__':
    main()
