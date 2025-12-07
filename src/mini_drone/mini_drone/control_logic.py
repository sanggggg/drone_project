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
    [1.03695, 0, 0, 0, 0, 1.46057, -2.92222, 2.12216, -0.54184, 0, 0, 0, 0, 2.17737, -4.53931, 3.39604, -0.889904, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.941936, 0.125, 0.171233, 0.00277891, 0.0339483, 1.85377, -4.9363, 4.4453, -1.36188, 0.15, 0.154407, -0.0736637, -0.0584392, -0.279379, 0.241263, 0.113525, -0.105646, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.917254, 0.325, 0.153148, -0.00290535, 4.24018e-05, 0.927875, -2.72279, 2.61695, -0.843071, 0.15, -0.135681, -0.0143041, 0.0162908, -0.841905, 2.15596, -1.9501, 0.60671, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.824189, 0.45, 0.0556065, -0.039849, 0.000339984, -0.482729, 1.13297, -0.963344, 0.289829, 0, -0.156779, -0.0149572, -0.000947146, -0.918429, 2.66483, -2.66993, 0.917875, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.20999, 0.45, -0.0387725, -0.0164764, -0.000988187, -0.512367, 0.701689, -0.357264, 0.0651314, -0.15, -0.170393, 0.00948314, 0.00103106, -0.448644, 0.818116, -0.537689, 0.123156, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.62328, 0.225, -0.322108, -0.0299396, 0.0208129, -0.255137, 0.55739, -0.334399, 0.0638077, -0.4, -0.215725, -0.0151006, -0.00345696, -0.0444385, 0.116286, -0.0712259, 0.0136636, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.63922, 0, 0.288654, 0.154226, -0.0490271, 0.416384, -0.803945, 0.469758, -0.0888826, -0.7, -0.0821309, 0.055299, -0.000545741, 0.168463, -0.262011, 0.135431, -0.0237202, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure2_END = [
    [1.51452, 0.5, 0, 0, 0, -0.383729, 0.44434, -0.192149, 0.0299362, -0.75, 0, 0, 0, 0.575593, -0.666509, 0.288223, -0.0449044, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.5299, 0.25, -0.30085, -1.76442e-15, 0.0298251, 0.00514538, 0.110803, -0.105868, 0.0256971, -0.375, 0.451274, -2.63121e-15, -0.0447376, -0.00771808, -0.166205, 0.158802, -0.0385456, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure3_DATA = [
    [1.65394, 0, 0, 0, 0, 1.39304, -1.88777, 0.9006, -0.149027, 0, 0, 0, 0, 0.17973, -0.270842, 0.137211, -0.0236756, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.30363, 0.45, 0.102895, -0.190235, 0.0378061, -1.31004, 2.48817, -1.5962, 0.348582, 0, -0.0842531, -0.0560718, -0.00477955, -0.489152, 0.885867, -0.553949, 0.119094, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.07313, 0.325, 0.0388766, 0.134222, 0.00183088, 0.960684, -2.47657, 2.01002, -0.544756, -0.25, -0.219184, -0.0183834, -0.0069986, 0.0836043, 0.084016, -0.181137, 0.0678659, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.07313, 0.45, -1.30566e-14, -0.123618, 3.77417e-14, -1.92453, 4.53904, -3.5692, 0.952361, -0.45, -0.13363, 1.03828e-15, -0.00752329, -0.500521, 0.97548, -0.689071, 0.171203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.30363, 0.325, 0.023326, 0.134222, -0.00140603, 0.783612, -1.66314, 1.1116, -0.248145, -0.65, -0.219184, -0.0183834, -0.0069986, -0.182235, 0.471365, -0.341875, 0.0800326, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.65394, 0.45, -0.12075, -0.190235, -0.00910555, -0.853519, 1.4671, -0.79821, 0.144139, -0.9, -0.0842531, 0.0560718, -0.00477955, 0.179021, -0.269279, 0.136896, -0.0236756, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure3_END = [
    [1.62329, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0.512651, -0.51878, 0.195179, -0.0263757, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.60383, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, 0.581999, -1.32331e-15, -0.0539809, -0.0419814, -0.125511, 0.134395, -0.0325465, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure4_DATA = [
    [1.83122, 0, 0, 0, 0, -0.974112, 1.21414, -0.529055, 0.0796793, 0, 0, 0, 0, -0.511197, 0.588694, -0.24597, 0.0362211, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.61359, -0.4, 0.00444137, 0.175594, -0.0364728, 0.316951, -0.424958, 0.198818, -0.0323213, -0.4, -0.286394, 0.0389177, 0.043589, 0.122501, -0.0653274, -0.00489381, 0.00520329, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.00577, 0, 0.26387, -0.110581, -0.0183618, -0.187132, 0.226625, -0.0900867, 0.0122614, -0.4, 0.345695, 0.116689, -0.0432835, 0.0212982, -0.0879527, 0.046381, -0.00706777, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.35676, 0, -0.0600583, 0.0370738, 0.00467469, 0.0170792, -0.0259109, 0.0102829, -0.0013138, 0, -0.326131, -0.284946, 0.0707016, -0.243205, 0.320629, -0.130093, 0.0171076, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure4_END = [
    [1.70624, -0.5, 0, 0, 0, 0.193935, -0.177898, 0.0600726, -0.00720464, 1, 0, 0, 0, -0.387871, 0.355796, -0.120145, 0.0144093, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.67468, -0.25, 0.282451, -3.50577e-15, -0.0246811, -0.0313537, -0.0311165, 0.0423937, -0.0104429, 0.5, -0.564903, 1.29002e-14, 0.0493622, 0.0627074, 0.0622329, -0.0847874, 0.0208857, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure5_DATA = [
    [1.56282, 0, 0, 0, 0, -1.40429, 1.96563, -0.97922, 0.169918, 0, 0, 0, 0, 0.364377, -0.583775, 0.314322, -0.0575695, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.4873, -0.45, -0.257154, 0.0951956, -0.0477159, 0.0854674, 0.129707, -0.154883, 0.0399511, 0, -0.138044, -0.0884004, -0.00447981, -0.736514, 1.24227, -0.700673, 0.13435, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.39215, -0.45, 0.271462, 0.0216762, -0.0192056, 0.132439, -0.309114, 0.21441, -0.0478586, -0.4, -0.145999, 0.0915868, -0.00161549, 0.613618, -1.05035, 0.613018, -0.122843, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.25578, -0.125, 0.132376, -0.0337282, 0.00566126, 0.553312, -1.14262, 0.791845, -0.185253, -0.4, -0.0308314, -0.0698097, -0.0111286, -1.12782, 1.98103, -1.2279, 0.264865, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.17534, 0, 0.00153071, -0.0157631, 0.0041467, 0.249155, -0.846458, 0.726788, -0.195144, -0.7, -0.296802, 0.0141266, -0.0150273, -0.32102, 0.982425, -0.821241, 0.218751, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.26354, -0.125, -0.297042, -0.0667848, 0.0492486, -0.912024, 2.10331, -1.54478, 0.37333, -0.95, -0.0520292, 0.0469623, -0.0101188, 0.253717, -0.490984, 0.325274, -0.0734979, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
    [1.68774, 0, 0, 0, 0, 1.43788, -1.94385, 0.921926, -0.151386, 0, 0, 0, 0, 0.236287, -0.304008, 0.128338, -0.0184645, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.39888, 0.45, 0.0477463, -0.205556, -0.0110388, 0.015788, 0.046346, -0.0263432, 0.00385289, 0, -0.231983, -0.267053, 0.0316714, -0.309545, 0.388271, -0.150405, 0.0190854, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure7_END = [
    [1.62329, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0.512651, -0.51878, 0.195179, -0.0263757, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.60383, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, 0.581999, -1.32331e-15, -0.0539809, -0.0419814, -0.125511, 0.134395, -0.0325465, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure8_DATA = [
    [1.38053, 0, 0, 0, 0, 1.50991, -2.45602, 1.41246, -0.282173, 0, 0, 0, 0, 0.62301, -0.829742, 0.411167, -0.0730983, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.38506, 0.25, 0.108747, -0.111047, -0.0169171, -0.611937, 0.886758, -0.455948, 0.0826668, 0.25, 0.315983, -0.00237448, -0.0299952, 0.139644, -0.4207, 0.321, -0.07559, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.30461, 0, -0.342021, 0.0160878, 0.0225464, -0.179283, 0.589223, -0.477992, 0.119398, 0.5, -0.0386429, -0.0691903, -0.00509588, -0.81548, 1.45524, -0.902264, 0.192877, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.28516, -0.25, 0.0392427, 0.0629071, -0.00594997, 0.814226, -1.40889, 0.863956, -0.184121, 0.25, -0.212618, -0.00191534, -0.00541372, -0.203702, 0.477266, -0.344454, 0.0815721, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.28516, 0, 0.234235, 9.37237e-15, 0.00768052, 0.954605, -2.02471, 1.39675, -0.322302, 0, -0.137058, 1.21144e-14, -0.00453654, -0.427151, 0.732416, -0.448585, 0.0957467, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.30461, 0.25, -0.0234906, -0.0629071, -0.00306795, -0.200622, 0.186553, -0.0454067, -0.0008486, -0.25, -0.212618, -0.0105169, -0.00544159, -0.868744, 1.79219, -1.20755, 0.272779, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.36692, 0, -0.342021, -0.021722, 0.0226349, -0.467873, 1.15443, -0.816774, 0.18533, -0.5, 0.00724554, 0.0691903, -0.00509588, 0.265677, -0.310897, 0.130159, -0.0189633, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.43003, -0.25, 0.173184, 0.111047, -0.0411768, 0.480056, -1.00931, 0.661773, -0.141904, -0.25, 0.315983, 0.00198593, -0.0299952, 0.00548225, -0.185787, 0.178054, -0.0452011, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
            'goto': 2.5,
        }

        # ---- E-STOP latch state ----
        self.estop_latched = False

        # ---- Publishers ----
        self.pub_estop = self.node.create_publisher(Bool, 'estop', _qos_sense())

        # ---- Subscriptions (HL) ----
        self.node.create_subscription(Float32, 'hl/takeoff', self._on_hl_takeoff, _qos_ctrl())
        self.node.create_subscription(Float32, 'hl/land',    self._on_hl_land,    _qos_ctrl())
        self.node.create_subscription(PoseStamped, 'hl/goto', self._on_hl_goto,   _qos_ctrl())

        # ---- Services ----
        self.node.create_service(Trigger, 'stop',         self._srv_stop_cb)
        self.node.create_service(Trigger, 'estop_reset',  self._srv_estop_reset)
        self.node.create_service(Trigger, 'notify_stop',  self._srv_notify_cb)

        # ---- Trajectory Service (단일 서비스, type 인자로 선택) ----
        self.node.create_service(RunTrajectory, 'traj/run', self._srv_traj_run)

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