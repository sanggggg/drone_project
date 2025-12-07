#!/usr/bin/env python3
import math
import time
import threading
from typing import Optional, Dict

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32, Float32MultiArray, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_srvs.srv import Trigger
from mini_drone_interfaces.srv import RunTrajectory

# ---- cflib 라이브러리 (High-Level Trajectory용) ----
try:
    from cflib.crazyflie.mem import MemoryElement, Poly4D
except ImportError:
    Poly4D = None
    MemoryElement = None

# ---- Figure 8 데이터 ----
# FIGURE8_DATA = [
#     [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
# ]

Figure0_DATA = [
    [1.18726, 0, 0, 0, 0, 2.72206, -5.18268, 3.49059, -0.814637, 0, 0, 0, 0, 0.454008, -0.923662, 0.632077, -0.148396, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.16942, 0.25, 0.16848, -0.035585, 0.0539442, 0.0246711, -0.109238, 0.0566124, -0.00861245, 0, -0.0999729, -0.131128, -0.0273956, -0.379988, 0.433723, -0.164074, 0.0211938, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.57258, 0.25, -0.277241, -0.0225275, 0.0104607, -0.262988, 0.400257, -0.211985, 0.0385146, -1, -0.379325, 0.117071, 0.0140198, 0.0437299, 0.121611, -0.125753, 0.0301616, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.17839, -0.25, -0.277241, 0.0225275, 0.0104607, 0.189685, -0.169661, 0.0561014, -0.00667962, -1, 0.428497, 0.117071, -0.0283945, 0.254814, -0.32126, 0.130191, -0.017555, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.14294, -0.25, 0.226089, 0.0222406, -0.0536123, 2.11638, -4.75339, 3.62545, -0.934043, 0, 0.0862193, -0.131128, 0.0273956, -0.247023, 0.642942, -0.51219, 0.133903, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure1_DATA = [
    [1.60383, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.562414, 0.590316, -0.230998, 0.0325465, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.62329, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, -0.581999, 3.23234e-15, 0.0539809, 0.0678949, 0.0773259, -0.104529, 0.0263757, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure1_END = [
    [1.62329, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0.512651, -0.51878, 0.195179, -0.0263757, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.60383, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, 0.581999, -1.32331e-15, -0.0539809, -0.0419814, -0.125511, 0.134395, -0.0325465, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure2_DATA = [
    [1.49589, 0, 0, 0, 0, 0.171213, -0.0783253, -0.0181129, 0.0108778, 0, 0, 0, 0, 1.2386, -1.89971, 1.02558, -0.191744, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.40515, 0.25, 0.370607, 0.0275178, -0.0344327, -0.149728, 0.00999205, 0.074034, -0.025005, 0.25, 0.0746121, -0.0910662, -0.0433394, -0.0914948, 0.033204, 0.0381954, -0.0160871, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.20448, 0.5, -0.136859, -0.161409, 0.0174586, -0.222917, 0.289005, -0.115156, 0.015153, 0, -0.417582, -0.0535152, 0.0223688, -0.0486256, 0.080841, -0.0363409, 0.00514382, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.72479, 0, 0.182205, 0.188338, -0.0517206, 0.754037, -1.19522, 0.620761, -0.107497, -0.75, -0.0846, 0.0871605, -0.0124266, 0.0655868, -0.115494, 0.0614797, -0.0106969, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure2_END = [
    [1.51452, 0.5, 0, 0, 0, -0.383729, 0.44434, -0.192149, 0.0299362, -0.75, 0, 0, 0, 0.575593, -0.666509, 0.288223, -0.0449044, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.5299, 0.25, -0.30085, -1.76442e-15, 0.0298251, 0.00514538, 0.110803, -0.105868, 0.0256971, -0.375, 0.451274, -2.63121e-15, -0.0447376, -0.00771808, -0.166205, 0.158802, -0.0385456, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure3_DATA = [
    [1.8501, 0, 0, 0, 0, 0.892241, -1.06816, 0.453096, -0.0669952, 0, 0, 0, 0, 0.116092, -0.15487, 0.0695166, -0.010661, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.83495, 0.5, 0.148919, -0.186833, -0.0448152, -0.347496, 0.528646, -0.243963, 0.0376555, 0, -0.083404, -0.0591227, -0.00906398, -0.0859789, 0.0757445, -0.0218671, 0.00202542, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.83455, 0.25, 1.63393e-13, 0.170909, 2.3802e-15, 0.362296, -0.559743, 0.265659, -0.0418225, -0.5, -0.411377, 4.06757e-15, 0.011838, 0.0240998, 0.0255108, -0.0266341, 0.00563153, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.74396, 0.5, -0.164495, -0.186833, 0.0448152, -0.760111, 1.18725, -0.607827, 0.103869, -1, -0.0697938, 0.0591227, -0.00906398, 0.0895678, -0.134484, 0.0667159, -0.0111285, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure3_END = [
    [1.62329, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0.512651, -0.51878, 0.195179, -0.0263757, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.60383, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, 0.581999, -1.32331e-15, -0.0539809, -0.0419814, -0.125511, 0.134395, -0.0325465, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure4_DATA = [
    [2.33642, 0, 0, 0, 0, -0.00452504, 0.00328775, -0.00065358, 3.22467e-05, 0, 0, 0, 0, 0.640396, -0.596159, 0.197548, -0.0229215, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.04197, 0, 0.0226998, 0.026122, 0.00409871, 0.152634, -0.216731, 0.0952083, -0.0138588, 1, 0.268982, -0.27224, -0.0737452, -0.106817, 0.189234, -0.0794291, 0.0108821, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.74938, 0, -0.213869, -0.0862585, -0.0067107, -0.23125, 0.379101, -0.190886, 0.0319061, 0.5, -0.350169, 0.114297, 0.0403407, -0.25431, 0.437096, -0.239712, 0.0426364, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.64332, -0.5, -0.14029, 0.0894795, -0.00324897, 0.308813, -0.469618, 0.240302, -0.0418142, 0.5, 0.43204, 0.0509432, -0.053403, 0.393899, -0.727725, 0.422715, -0.079931, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure4_END = [
    [1.70624, -0.5, 0, 0, 0, 0.193935, -0.177898, 0.0600726, -0.00720464, 1, 0, 0, 0, -0.387871, 0.355796, -0.120145, 0.0144093, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.67468, -0.25, 0.282451, -3.50577e-15, -0.0246811, -0.0313537, -0.0311165, 0.0423937, -0.0104429, 0.5, -0.564903, 1.29002e-14, 0.0493622, 0.0627074, 0.0622329, -0.0847874, 0.0208857, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure5_DATA = [
    [1.69561, 0, 0, 0, 0, -1.33287, 1.76771, -0.827889, 0.13442, 0, 0, 0, 0, 0.322854, -0.475341, 0.235225, -0.0396211, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.69561, -0.5, -0.191194, 0.0967481, -0.0469056, -0.1413, 0.326238, -0.192629, 0.0359353, 0, -0.160237, -0.0996069, -0.00515961, -0.46765, 0.711608, -0.355214, 0.0599752, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.4623, -0.5, 0.239345, 0.0175064, -0.0104588, 1.15954, -1.90757, 1.08766, -0.212385, -0.5, -0.145287, 0.107382, 0.000602307, 0.871508, -1.50014, 0.862961, -0.168639, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.66266, 0, 0.213752, -0.0212198, 0.0125878, -0.214124, 0.174547, -0.0545523, 0.00616112, -0.5, -0.145287, -0.107382, -0.000261627, -0.562503, 0.855418, -0.432512, 0.0742048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.69561, 0, -0.267413, -0.0967481, 0.0469056, -0.794145, 1.24175, -0.648783, 0.113698, -1, -0.160237, 0.0996069, -0.00515961, 0.323103, -0.474441, 0.235048, -0.0396211, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure5_END = [
    [1.70624, -0.5, 0, 0, 0, 0.193935, -0.177898, 0.0600726, -0.00720464, -1, 0, 0, 0, 0.387871, -0.355796, 0.120145, -0.0144093, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.67468, -0.25, 0.282451, -3.50577e-15, -0.0246811, -0.0313537, -0.0311165, 0.0423937, -0.0104429, -0.5, 0.564903, -1.29002e-14, -0.0493622, -0.0627074, -0.0622329, 0.0847874, -0.0208857, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure6_DATA = [
    [1.96946, 0, 0, 0, 0, -0.689501, 0.776378, -0.309298, 0.0427956, 0, 0, 0, 0, -0.278694, 0.254503, -0.087134, 0.0107604, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.63283, -0.5, -0.172091, 0.0903256, -0.0380624, -0.302439, 0.5818, -0.334497, 0.0627644, -0.5, -0.466356, -0.0125316, 0.0297127, 0.120032, -0.0708895, 0.000174661, 0.00425919, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.4623, -0.5, 0.262089, 0.0262566, -0.010301, 0.690859, -1.13032, 0.642803, -0.125432, -1, -0.121786, 0.0238268, -0.00148194, -0.298743, 0.649063, -0.419492, 0.0880898, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.66266, 0, 0.275094, -0.023548, -0.013529, -0.301608, 0.299533, -0.112114, 0.0152999, -1, 0.240818, 0.077223, -0.00634252, 0.38588, -0.61962, 0.323082, -0.0565429, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.69561, 0, -0.299926, -0.0955454, 0.0470695, -0.66533, 1.07003, -0.567591, 0.10038, -0.5, 0.132691, -0.0856151, 0.00702432, -0.263159, 0.385777, -0.191217, 0.0322492, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure6_END = [
    [1.33801, -0.5, 0, 0, 0, 0.681659, -0.925458, 0.468658, -0.0853341, -0.5, 0, 0, 0, 0.681659, -0.925458, 0.468658, -0.0853341, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.33801, -0.25, 0.331896, 1.878e-14, -0.0400442, 0.0786593, -0.371232, 0.330585, -0.0853341, -0.25, 0.331896, 9.45903e-15, -0.0400442, 0.0786593, -0.371232, 0.330585, -0.0853341, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure7_DATA = [
    [1.82476, 0, 0, 0, 0, 0.974219, -1.17133, 0.496857, -0.0734243, 0, 0, 0, 0, 0.253156, -0.31818, 0.133516, -0.0192774, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.53376, 0.5, 0.0815551, -0.270534, -0.00621137, -0.0936665, 0.152793, -0.0604026, 0.0074848, 0, -0.260288, -0.260278, -0.00257623, -0.119674, 0.19775, -0.0794603, 0.00996226, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure7_END = [
    [1.62329, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0.512651, -0.51878, 0.195179, -0.0263757, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.60383, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, 0.581999, -1.32331e-15, -0.0539809, -0.0419814, -0.125511, 0.134395, -0.0325465, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure8_DATA = [
    [1.98749, 0, 0, 0, 0, 0.817674, -0.946173, 0.384441, -0.05397, 0, 0, 0, 0, 0.427081, -0.444351, 0.168014, -0.0223629, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.03482, 0.5, 0.107702, -0.0715211, 0.0150497, -0.429931, 0.438373, -0.162355, 0.0211365, 0.5, 0.358627, -0.00296594, 0.0109629, 0.178829, -0.278747, 0.128949, -0.0194194, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.93964, 0, -0.388178, 0.00679336, -0.00827282, -0.21231, 0.347849, -0.169755, 0.0269401, 1, -0.0438579, -0.0445628, 0.00105347, -0.511846, 0.616436, -0.259657, 0.0376922, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.84115, -0.5, 0.0445387, 0.040516, 0.00217466, 0.615515, -0.772347, 0.339887, -0.0516606, 0.5, -0.241312, -0.0067735, -0.00198885, -0.351099, 0.486385, -0.227469, 0.0360356, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.84115, 0, 0.265846, 2.43092e-15, 0.00280715, 0.601442, -0.842744, 0.395787, -0.0628223, 0, -0.155555, -2.15853e-15, -0.00072022, -0.451842, 0.569186, -0.252285, 0.0385969, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.93964, 0.5, 0.00106141, -0.040516, 0.00217466, -0.352489, 0.376292, -0.145624, 0.0198242, -0.5, -0.241312, -0.0067735, -0.00198885, -0.493609, 0.655426, -0.29171, 0.0439009, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.03482, 0, -0.388178, -0.0139903, -0.00827282, -0.305442, 0.457259, -0.208294, 0.0310732, -1, -0.000320154, 0.0445628, 0.0011466, 0.271368, -0.278526, 0.102681, -0.0132804, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.98749, -0.5, 0.196555, 0.0715211, -0.0150497, 0.470056, -0.628134, 0.2786, -0.0414829, -0.5, 0.358627, 0.00296594, 0.0109629, 0.178376, -0.295854, 0.143108, -0.0223629, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure9_DATA = [
    [1.65612, 0, 0, 0, 0, -1.44154, 1.95428, -0.936377, 0.155599, 0, 0, 0, 0, -0.265813, 0.399486, -0.201841, 0.0347431, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.62548, -0.5, -0.206586, 0.0955454, -0.0470695, -0.199787, 0.465226, -0.285149, 0.0553886, 0, 0.126471, 0.0856151, 0.00702432, 0.632462, -0.942008, 0.475961, -0.0822794, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.47754, -0.5, 0.274754, 0.0186292, -0.013529, 0.647735, -1.04749, 0.590081, -0.114052, 0.5, 0.226896, -0.077223, 0.00883085, -0.391287, 0.488255, -0.223888, 0.0364888, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.6331, 0, 0.262089, -0.0262566, -0.0104645, -0.113387, 0.00240342, 0.0462487, -0.0132719, 0.5, -0.158986, -0.0238431, -0.00148194, -0.1035, 0.0663195, -0.00890643, -0.00114725, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.91574, 0, -0.336352, -0.0903256, 0.0375866, -0.174822, 0.308446, -0.159554, 0.0263415, 0, -0.466356, -0.0289018, 0.0297127, 0.044059, 0.0359708, -0.0421067, 0.00898008, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure9_END = [
    [1.33801, -0.5, 0, 0, 0, 0.681659, -0.925458, 0.468658, -0.0853341, -0.5, 0, 0, 0, 0.681659, -0.925458, 0.468658, -0.0853341, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.33801, -0.25, 0.331896, 1.878e-14, -0.0400442, 0.0786593, -0.371232, 0.330585, -0.0853341, -0.25, 0.331896, 9.45903e-15, -0.0400442, 0.0786593, -0.371232, 0.330585, -0.0853341, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

# ---- Trajectory Registry: type_name -> (data, traj_id) ----
TRAJECTORY_REGISTRY = {
    'figure0': (Figure0_DATA, None, 0),         # 0번은 복귀 궤적 없음 (예시)
    'figure1': (Figure1_DATA, Figure1_END, 1),  # 1번은 복귀 궤적 있음
    'figure2': (Figure2_DATA, Figure2_END, 2),
    'figure3': (Figure3_DATA, Figure3_END, 3),
    'figure4': (Figure4_DATA, Figure4_END, 4),
    'figure5': (Figure5_DATA, Figure5_END, 5),
    'figure6': (Figure6_DATA, Figure6_END, 6),
    'figure7': (Figure7_DATA, Figure7_END, 7),
    'figure8': (Figure8_DATA, None, 8),         # 8번은 제자리로 돌아오므로 없음
    'figure9': (Figure9_DATA, Figure9_END, 9),
}

def _qos_ctrl() -> QoSProfile:
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.RELIABLE   # 제어/HL은 반드시 RELIABLE
    q.history = HistoryPolicy.KEEP_LAST
    return q


def _qos_sense() -> QoSProfile:
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.BEST_EFFORT  # 상태 브로드캐스트/센싱은 BEST_EFFORT 허용
    q.history = HistoryPolicy.KEEP_LAST
    return q


class ControlManager:
    """
    Crazyflie 제어 브리지.
    """

    def __init__(self, node,
                 cmd_rate_hz: float = 50.0,
                 hover_timeout_s: float = 0.5,
                 hl_durations: Dict[str, float] = None):
        self.node = node

        # ---------- 파라미터 get/declare 헬퍼 (이미 선언돼 있으면 재선언하지 않음) ----------
        def get_or_declare(name: str, default):
            if hasattr(self.node, "has_parameter") and self.node.has_parameter(name):
                return self._param_value(self.node.get_parameter(name))
            # 이미 선언돼 있지 않으면 선언
            p = self.node.declare_parameter(name, default)
            return self._param_value(p)

        # ---------- Parameters ----------
        self.dry_run = bool(get_or_declare('dry_run', False))
        # 타 모듈이 미리 선언했을 가능성이 높은 항목들: get_or_declare 로 안전하게 처리

        self._hl_active_until = 0.0

        # 파라미터 변경 콜백
        self.node.add_on_set_parameters_callback(self._on_set_params)

        self.node.get_logger().info(f'dry_run initial={self.dry_run}')

        # 내부 상태
        self.cf = None
        self._lock = threading.Lock()

        self.cmd_rate_hz = float(cmd_rate_hz)
        self.hl_durations = hl_durations or {
            'takeoff': 2.0,
            'land': 2.0,
            'goto': 1.0,
        }

        # ---- E-STOP latch state ----
        self.estop_latched = False

        # ---- Publishers ----
        self.pub_estop = self.node.create_publisher(Bool, '/cf/estop', _qos_sense())

        # ---- Subscriptions (HL) ----
        self.node.create_subscription(Float32, '/cf/hl/takeoff', self._on_hl_takeoff, _qos_ctrl())
        self.node.create_subscription(Float32, '/cf/hl/land',    self._on_hl_land,    _qos_ctrl())
        self.node.create_subscription(PoseStamped, '/cf/hl/goto', self._on_hl_goto,   _qos_ctrl())

        # ---- Services ----
        self.node.create_service(Trigger, '/cf/stop',         self._srv_stop_cb)
        self.node.create_service(Trigger, '/cf/estop_reset',  self._srv_estop_reset)
        self.node.create_service(Trigger, '/cf/notify_stop',  self._srv_notify_cb)

        # ---- Trajectory Service (단일 서비스, type 인자로 선택) ----
        self.node.create_service(RunTrajectory, '/cf/traj/run', self._srv_traj_run)

        if self.dry_run:
            self.node.get_logger().info('[DRY-RUN] 실제 전송 없이 로그만 출력합니다. (-p dry_run:=false 로 전송 허용 가능)')

    # ---------- utils ----------
    @staticmethod
    def _param_value(param_or_declared):
        # rclpy.Parameter 또는 ParameterValue 래핑 모두 안전하게 값만 뽑아내기
        if hasattr(param_or_declared, "value"):
            return param_or_declared.value
        if hasattr(param_or_declared, "get_parameter_value"):
            v = param_or_declared.get_parameter_value()
            # bool_value / double_value / integer_value / string_value 중 채워진 걸 사용
            for attr in ("bool_value", "double_value", "integer_value", "string_value"):
                if hasattr(v, attr):
                    vv = getattr(v, attr)
                    # rclpy는 미설정이면 0/False/''가 올 수 있으므로 그냥 반환
                    return vv
        return param_or_declared

    # ========== Public API ==========
    def attach_cf(self, cf):
        with self._lock:
            self.cf = cf
        self.node.get_logger().info('ControlManager attached to CF')

    def detach_cf(self):
        with self._lock:
            self.cf = None

    # ========== Param updates ==========
    def _on_set_params(self, params):
        for p in params:
            if p.name == 'dry_run':
                self.dry_run = bool(p.value)
                self.node.get_logger().warn(f'[PARAM] dry_run -> {self.dry_run}')
        return SetParametersResult(successful=True)

    # ========== E-STOP helpers ==========
    def _publish_estop_state(self):
        self.pub_estop.publish(Bool(data=self.estop_latched))

    def _send_notify_stop(self):
        if self.dry_run or self.cf is None:
            self.node.get_logger().info("[SIM notify_setpoint_stop]")
            return
        try:
            self.cf.commander.send_notify_setpoint_stop()
        except Exception:
            pass

    def _send_stop(self):
        if self.dry_run or self.cf is None:
            self.node.get_logger().info("[SIM STOP setpoint]")
            return
        try:
            self.cf.commander.send_stop_setpoint()
        except Exception:
            pass

    def _enable_hl(self) -> bool:
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] HL enable 차단됨')
            return False
        if self.dry_run or self.cf is None:
            self.node.get_logger().info("[SIM HL enable]")
            return True
        try:
            self.cf.param.set_value('commander.enHighLevel', '1')
            time.sleep(0.05)
            self.cf.commander.send_notify_setpoint_stop()
            return True
        except Exception as e:
            self.node.get_logger().warn(f'Enable HL skipped/failed: {e}')
            return False

    def _hl_takeoff(self, z: float, dur: float):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] takeoff 차단됨')
            return
        if self.dry_run or self.cf is None:
            self.node.get_logger().info(f"[SIM HL takeoff] z={z:.2f} m, dur={dur:.2f} s")
            return
        from cflib.crazyflie.high_level_commander import HighLevelCommander
        try:
            HighLevelCommander(self.cf).takeoff(z, dur)
        except Exception as e:
            self.node.get_logger().error(f'HL takeoff failed: {e}')

    def _hl_land(self, z: float, dur: float):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] land 차단됨')
            return
        if self.dry_run or self.cf is None:
            self.node.get_logger().info(f"[SIM HL land] z={z:.2f} m, dur={dur:.2f} s")
            return
        from cflib.crazyflie.high_level_commander import HighLevelCommander
        try:
            HighLevelCommander(self.cf).land(z, dur)
        except Exception as e:
            self.node.get_logger().error(f'HL land failed: {e}')

    def _hl_goto(self, x: float, y: float, z: float, yaw_rad: float, dur: float):
        if self.estop_latched:
            self.node.get_logger().warn('[E-STOP] goto 차단됨')
            return
        if self.dry_run or self.cf is None:
            self.node.get_logger().info(f"[SIM HL goto] x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw_rad:.2f} rad, dur={dur:.2f} s")
            return
        from cflib.crazyflie.high_level_commander import HighLevelCommander
        try:
            HighLevelCommander(self.cf).go_to(x, y, z, yaw_rad, dur, relative=False, linear=False)
        except Exception as e:
            self.node.get_logger().error(f'HL goto failed: {e}')

    # ========== High-level (takeoff/land/goto) ==========
    def _on_hl_takeoff(self, msg: Float32):
        if not self._enable_hl():
            return
        
        self._hl_takeoff(float(msg.data), float(self.hl_durations['takeoff']))
        self._hl_active_until = time.time() + float(self.hl_durations['takeoff']) + 0.5

    def _on_hl_land(self, msg: Float32):
        if not self._enable_hl():
            return

        self._hl_land(float(msg.data), float(self.hl_durations['land']))
        import time
        self._hl_active_until = time.time() + float(self.hl_durations['land']) + 0.5

    def _on_hl_goto(self, msg: PoseStamped):
        if not self._enable_hl():
            return
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)) if (qw or qx or qy or qz) else 0.0
        self._hl_goto(float(msg.pose.position.x),
                      float(msg.pose.position.y),
                      float(msg.pose.position.z),
                      float(yaw),
                      float(self.hl_durations['goto']))
        self._hl_active_until = time.time() + float(self.hl_durations['goto']) + 0.5

    # ========== Services ==========
    def _srv_stop_cb(self, req, res):
        self.estop_latched = True
        self._publish_estop_state()
        self._send_stop()
        res.success = True
        res.message = 'E-STOP latched; motors stop'
        return res

    def _srv_estop_reset(self, req, res):
        self.estop_latched = False
        self._publish_estop_state()
        res.success = True
        res.message = 'E-STOP reset; command gate open'
        return res

    def _srv_notify_cb(self, req, res):
        self._send_notify_stop()
        res.success = True
        res.message = 'notify_setpoint_stop (or SIM) sent'
        return res

# ========== Trajectory Service Implementation ==========
    def _upload_trajectory(self, traj_data, traj_id: int) -> tuple[bool, str, float]:
        """
        궤적 데이터를 드론 메모리에 업로드하는 공통 헬퍼.
        Returns: (success, message, total_duration)
        """
        if self.cf is None:
            return False, "CF not connected", 0.0
        
        if Poly4D is None:
            return False, "cflib not installed", 0.0

        try:
            traj_mem = self.cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
            traj_mem.trajectory = []
            
            total_duration = 0.0
            for row in traj_data:
                duration = row[0]
                x = Poly4D.Poly(row[1:9])
                y = Poly4D.Poly(row[9:17])
                z = Poly4D.Poly(row[17:25])
                yaw = Poly4D.Poly(row[25:33])
                traj_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
                total_duration += duration
            
            if not traj_mem.write_data_sync():
                return False, "Upload failed", 0.0
            
            self.cf.high_level_commander.define_trajectory(traj_id, 0, len(traj_mem.trajectory))
            return True, f"Uploaded ID {traj_id} (dur={total_duration:.1f}s)", total_duration
            
        except Exception as e:
            return False, str(e), 0.0

    def _start_trajectory(self, traj_id: int, duration: float) -> tuple[bool, str]:
        """
        업로드된 궤적을 실행하는 공통 헬퍼.
        time_scale 인자 제거 -> 기본 1.0 (정속) 사용
        """
        if not self._enable_hl():
            return False, "Enable HL failed"

        try:
            # time_scale=1.0 으로 고정
            self.cf.high_level_commander.start_trajectory(traj_id, 1.0, relative=True)
            
            # 예상 종료 시간 (정속이므로 duration 그대로 사용)
            self._hl_active_until = time.time() + duration + 2.0
            
            return True, f"Started Trajectory ID {traj_id}"
        except Exception as e:
            return False, str(e)

    def _srv_traj_run(self, req, res):
        traj_type = req.trajectory_type.lower().strip()
        
        if traj_type not in TRAJECTORY_REGISTRY:
            res.success = False
            res.message = f"Unknown: {traj_type}"
            return res
        
        # Unpack: (메인 데이터, 종료 데이터, ID)
        main_data, end_data, traj_id = TRAJECTORY_REGISTRY[traj_type]
        
        self.node.get_logger().info(f"[TRAJ] Running '{traj_type}'...")

        # ---------------------------------------------------------
        # 1. Main Trajectory 실행 (ID = traj_id)
        # ---------------------------------------------------------
        # 1-1. Upload
        ok, msg, main_duration = self._upload_trajectory(main_data, traj_id=traj_id)
        if not ok:
            res.success = False; res.message = f"Main upload failed: {msg}"; return res
        
        # 1-2. Start (time_scale 없이 기본 속도 1.0)
        ok, msg = self._start_trajectory(traj_id=traj_id, duration=main_duration)
        if not ok:
            res.success = False; res.message = f"Main start failed: {msg}"; return res
            
        total_msg = f"Main started (dur={main_duration:.1f}s)"

        # ---------------------------------------------------------
        # 2. End Trajectory 실행 (있을 경우만)
        # ---------------------------------------------------------
        if end_data is not None:
            # 1. 메인 비행 시간만큼 대기
            # (main_duration이 float인지 확실하지 않으면 float()로 변환)
            sleep_time = float(main_duration)
            time.sleep(sleep_time)
            
            # 2. 1초 정지 (Hovering)
            self.node.get_logger().info("[TRAJ] Main finished. Waiting 1s before return...")
            time.sleep(1.0)
            
            # 3. End 궤적 업로드 (ID 충돌 방지: traj_id + 100)
            end_id = traj_id + 100
            ok, msg, end_dur = self._upload_trajectory(end_data, traj_id=end_id)
            
            if ok:
                # 4. End 궤적 실행
                self._start_trajectory(traj_id=end_id, duration=end_dur)
                self.node.get_logger().info(f"[TRAJ] Return started (dur={end_dur:.1f}s)")
            else:
                self.node.get_logger().error(f"[TRAJ] Return upload failed: {msg}")

        res.success = True
        res.message = total_msg
        self.node.get_logger().info(res.message)
        return res