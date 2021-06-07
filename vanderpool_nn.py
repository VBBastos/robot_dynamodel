# -*- coding: utf-8 -*-
"""
Created on Mon Aug 19 02:01:23 2019

@author: basto
"""

import os
import sys
module_path = os.path.abspath(os.path.join('..'))
if module_path not in sys.path:
    sys.path.append(module_path)
    
from pySINDy.sindy import SINDy
import numpy as np
import matplotlib.pyplot as plt
from pySINDy.utils.generator import van_der_pol_generator