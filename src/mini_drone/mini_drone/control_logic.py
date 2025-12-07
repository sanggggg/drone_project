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
    [1.71718, 0, 0, 0, 0, 1.09528, -1.41438, 0.650363, -0.104032, 1, 0, 0, 0, 0.242038, -0.34479, 0.16517, -0.0270587, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [3.08926, 0.5, 0.28584, -0.0347819, 0.0200031, -0.0425599, 0.0126812, -0.000648628, -0.000100106, 1, -0.146951, -0.11819, -0.0122245, -0.198764, 0.156797, -0.0417649, 0.00380995, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.33065, 0.5, -0.304572, -0.0253245, 0.00275226, -0.279854, 0.290331, -0.103831, 0.0127052, -1, -0.58839, 0.0681444, 0.0142667, 0.0784513, -6.21061e-06, -0.0177389, 0.00361219, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [3.04526, -0.5, -0.304572, 0.0220984, -0.00486588, 0.118197, -0.080605, 0.020326, -0.00181405, -1, 0.666933, 0.0893697, -0.015854, 0.129867, -0.11846, 0.0346196, -0.00336275, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.72145, -0.5, 0.198408, 0.0347819, -0.0200031, 1.14403, -1.66573, 0.828726, -0.140057, 1, 0.128928, -0.11819, 0.0122245, -0.135038, 0.222046, -0.114732, 0.0196619, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure1_DATA = [
    [2.08819, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -0.740133, 0.743242, -0.270409, 0.0345543, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.01956, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.661771, -7.18146e-15, -0.0217773, -0.454015, 0.678105, -0.311997, 0.047113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure2_DATA = [
    [2.00298, -0.5, 0, 0, 0, 0.298816, -0.28301, 0.0996862, -0.0125465, 0.5, 0, 0, 0, 0.769955, -0.885748, 0.358599, -0.050204, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.02661, 0, 0.440346, 0.0241393, -0.00689801, 0.277885, -0.433965, 0.202462, -0.0307184, 1, 0.138073, -0.054285, -0.0161928, -0.232475, 0.19868, -0.0608346, 0.00656729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [3.03883, 0.5, -0.265944, -0.115324, -0.00976583, -0.0858965, 0.0914407, -0.0275027, 0.00268868, 0.5, -0.557514, -0.0225541, 0.00944397, -0.0756983, 0.0673388, -0.0195489, 0.0018959, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.5524, -0.5, 0.168089, 0.148314, -0.0206657, 0.427209, -0.437839, 0.149859, -0.0172704, -1, -0.162468, 0.0775975, -0.00533112, 0.0846566, -0.0838382, 0.0278493, -0.00313626, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure3_DATA = [
    [2.46071, -0.5, 0, 0, 0, 0.664979, -0.616568, 0.201175, -0.0227281, 1, 0, 0, 0, 0.0684694, -0.0697839, 0.0239128, -0.00278834, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.59048, 0.5, 0.182367, -0.130711, -0.00466612, -0.307782, 0.291892, -0.0931372, 0.0101436, 1, -0.10082, -0.0418336, -0.00373085, -0.169717, 0.142596, -0.0425078, 0.00442044, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.59048, 0, -4.679e-14, 0.118991, -1.97799e-15, 0.264715, -0.268167, 0.0886355, -0.0098632, 0, -0.48849, -9.53569e-17, -0.00494383, -0.0879767, 0.10484, -0.0376495, 0.00442044, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.46071, 0.5, -0.182799, -0.130711, 0.0179359, -0.522526, 0.549019, -0.193706, 0.0230706, -1, -0.10082, 0.0418336, -0.00373085, 0.0723062, -0.0712853, 0.0241162, -0.00278834, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure4_DATA = [
    [3.26498, 0.5, 0, 0, 0, -0.0108116, 0.00823745, -0.0021081, 0.000183566, -1, 0, 0, 0, 0.347397, -0.234664, 0.0563801, -0.00473037, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [3.05935, 0.5, 0.0388814, 0.012398, -0.000424612, 0.00441082, -0.0070449, 0.00222304, -0.00021682, 1, 0.429425, -0.208404, -0.0464465, -0.104785, 0.091753, -0.0242551, 0.00216994, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.48024, 0.5, -0.14743, -0.060195, 0.00249311, -0.423589, 0.414472, -0.13922, 0.0159949, 0, -0.511529, 0.0943605, 0.0216555, -0.0663175, 0.108395, -0.0467805, 0.00620693, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.36094, -0.5, -0.166095, 0.0620092, -0.00129211, 0.143367, -0.147715, 0.0519702, -0.00625085, 0, 0.588516, 0.0259025, -0.0223263, 0.223444, -0.277419, 0.109647, -0.0142383, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure5_DATA = [
    [2.23328, 0.5, 0, 0, 0, -0.741082, 0.730294, -0.256699, 0.0314427, 1, 0, 0, 0, 0.185338, -0.209786, 0.079775, -0.0102983, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.17094, -0.5, -0.442873, 0.0665521, -0.00623648, 0.260793, -0.206588, 0.0600293, -0.00617524, 1, -0.187948, -0.0685187, -0.00206602, -0.730821, 0.824194, -0.317816, 0.0418552, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.94089, -0.5, 0.330279, 0.0107078, 0.00507825, 0.962151, -1.2099, 0.523394, -0.0773692, 0, -0.170412, 0.0738672, -0.000242987, 0.522266, -0.66582, 0.287593, -0.0423361, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.09703, 0.5, 0.263459, -0.014162, 0.00507825, -0.0801546, 0.0136423, 0.0122654, -0.00327072, 0, -0.170412, -0.0738672, -0.000242987, -0.887274, 1.02985, -0.410158, 0.0558485, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.32331, 0.5, -0.32913, -0.0665521, -0.018923, -0.52049, 0.605408, -0.229759, 0.0292267, -1, -0.187948, 0.0685187, -0.00203501, 0.176305, -0.18327, 0.0653284, -0.00797168, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure6_DATA = [
    [3.02108, 0.5, 0, 0, 0, -0.25831, 0.191325, -0.0500848, 0.00454851, 1, 0, 0, 0, -0.113343, 0.0699277, -0.015947, 0.00129215, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.20166, -0.5, -0.220566, 0.065537, -0.0180095, 0.0494414, -0.0142738, -0.0029914, 0.00110682, 0, -0.578202, -0.0227527, -0.0143184, -0.0769145, 0.152228, -0.0715509, 0.0104389, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.20794, -0.5, 0.193717, 0.00930122, -0.00276486, 0.62386, -0.659165, 0.24426, -0.0311981, -1, -0.171825, 0.0141734, -5.62477e-05, -0.040939, 0.0862102, -0.0414802, 0.00612146, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.34121, 0.5, 0.328016, -0.0149984, 0.0051735, -0.154846, 0.102217, -0.0251117, 0.00219309, -1, 0.27646, 0.0541468, 0.00331871, 0.39022, -0.421396, 0.153047, -0.018865, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.43946, 0.5, -0.320371, -0.031287, 0.0189396, -0.525417, 0.54122, -0.190614, 0.0227864, 0, 0.157425, -0.059424, 0.00279151, -0.121602, 0.120984, -0.0411998, 0.00479723, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

Figure7_DATA = [
    [3.00278, -0.5, 0, 0, 0, 0.345744, -0.262217, 0.0694706, -0.00636962, 1, 0, 0, 0, 0.11594, -0.0913307, 0.024091, -0.00217599, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [3.46729, 0.5, -0.0748791, -0.225808, 0.00308053, -0.0219632, 0.0375029, -0.0117677, 0.0011077, 1, -0.448076, -0.231925, 0.0110852, -0.0892131, 0.0899887, -0.0253778, 0.00228615, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
    [2.42139, 0.5, 0, 0, 0, -0.706833, 0.66891, -0.222757, 0.0256467, 0, 0, 0, 0, -0.116807, 0.121427, -0.04242, 0.00503618, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.36718, -0.5, -0.233687, 0.0663447, -0.0189396, -0.0957994, 0.140878, -0.0582018, 0.00770705, 0, 0.157425, 0.059424, 0.00279151, 0.404186, -0.40961, 0.142735, -0.0170525, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.24563, -0.5, 0.332628, 0.0149984, -0.00239274, 0.595408, -0.662456, 0.251331, -0.032422, 1, 0.27646, -0.0541468, 0.00331871, -0.173265, 0.151555, -0.0484661, 0.005491, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.1934, 0.5, 0.113593, -0.0154618, 0.00486254, 0.0912676, -0.130488, 0.0551259, -0.00758433, 1, -0.171825, -0.0141734, 3.18151e-05, -0.305024, 0.280334, -0.0935281, 0.0110035, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [2.79483, 0.5, -0.252986, -0.065537, 0.0180095, -0.283749, 0.25981, -0.080658, 0.00846493, 0, -0.578202, 0.0227527, -0.0143184, -0.0525683, 0.076002, -0.027782, 0.00318218, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

# ---- Trajectory Registry: type_name -> (data, traj_id) ----
TRAJECTORY_REGISTRY = {
    'figure0': (Figure0_DATA, 0),
    'figure1': (Figure1_DATA, 1),
    'figure2': (Figure2_DATA, 2),
    'figure3': (Figure3_DATA, 3),
    'figure4': (Figure4_DATA, 4),
    'figure5': (Figure5_DATA, 5),
    'figure6': (Figure6_DATA, 6),
    'figure7': (Figure7_DATA, 7),
    'figure8': (Figure8_DATA, 8),
    'figure9': (Figure9_DATA, 9),
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
        Returns: (success, message)
        """
        if not self._enable_hl():
            return False, "Enable HL failed"

        try:
            self.cf.high_level_commander.start_trajectory(traj_id, 1.0, relative=True)
            self._hl_active_until = time.time() + duration + 2.0
            return True, f"Started Trajectory ID {traj_id}"
        except Exception as e:
            return False, str(e)

    def _srv_traj_run(self, req, res):
        """
        단일 Trajectory 서비스: trajectory_type 인자로 궤적 선택
        지원 타입: 'figure8', 'vertical_a'
        """
        traj_type = req.trajectory_type.lower().strip()
        
        # Registry에서 조회
        if traj_type not in TRAJECTORY_REGISTRY:
            available = ', '.join(TRAJECTORY_REGISTRY.keys())
            res.success = False
            res.message = f"Unknown trajectory type '{traj_type}'. Available: {available}"
            self.node.get_logger().warn(res.message)
            return res
        
        traj_data, traj_id = TRAJECTORY_REGISTRY[traj_type]
        self.node.get_logger().info(f"[TRAJ] Running '{traj_type}' (upload + start)...")
        
        # 1. Upload
        ok, msg, duration = self._upload_trajectory(traj_data, traj_id=traj_id)
        if not ok:
            self.node.get_logger().error(f"[TRAJ] Upload failed: {msg}")
            res.success = False
            res.message = f"Upload failed: {msg}"
            return res
        
        self.node.get_logger().info(f"[TRAJ] {msg}")
        
        # 2. Start
        ok, msg = self._start_trajectory(traj_id=traj_id, duration=duration)
        if not ok:
            self.node.get_logger().error(f"[TRAJ] Start failed: {msg}")
            res.success = False
            res.message = f"Start failed: {msg}"
            return res
        
        res.success = True
        res.message = f"'{traj_type}' started (dur={duration:.1f}s)"
        self.node.get_logger().info(res.message)
        return res