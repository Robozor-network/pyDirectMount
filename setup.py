#!/usr/bin/python
# -*- coding: utf8 -*-

from setuptools import setup, find_packages
import sys
import os
import os.path as path


os.chdir(path.realpath(path.dirname(__file__)))
#sys.path.insert(1, 'src')
#mport pyDirectMount


setup(
    name             = 'pydirectmount',
    version          = '0.0.1',
    author           = 'Roman Dvorak',
    author_email     = 'roman-dvorak@email.cz',
    description      = 'DirectMount controll software.',
    long_description = "",
    url              = 'https://github.com/Robozor-network/pyDirectMount',
    
    packages    = ['pydirectmount'],
    #packages    = find_packages("src"),
    package_dir = {'': 'src'},
    provides    = ['pyDirectMount'],
    install_requires = [ 'hidapi' ],
    keywords = ['mount'],
    license     = 'Lesser General Public License v3',
    download_url = 'https://github.com/Robozor-network/pyDirectMount',
    
    #test_suite = 'pyDirectMount.tests',
    
    classifiers = [
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
        'Natural Language :: Czech',
        'Programming Language :: Python :: 2.6',
        'Programming Language :: Python :: 2.7',
    ]
)

