#!/usr/bin/env python

from distutils.core import setup

setup(name='Distutils',
      version='1.0',
      description='Python OpenHand Utilities',
      author='Carlos Rubert',
      author_email='cescuder@uji.es',
      packages=['oh_utils', 'oh_gui'],
      package_dir={'': 'src'},
     )
