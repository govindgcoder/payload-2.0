import numpy
import serial
from numpy import arccos, linalg
from vpython import box, canvas, color, vector
from vpython.rate_control import rate

scene = canvas()

fused_obj = box(pos=vector(2, 0, 0), length=3, width=1, height=0.5, color=color.green)
drift_obj = box(pos=vector(-2, 0, 0), length=3, width=1, height=0.5, color=color.red)


def update(obj: box, q0, q1, q2, q3):
    obj.axis = vector(1, 0, 0)
    obj.up = vector(0, 0, 1)
    ang = 2*arccos(q0)
    m = linalg.norm(numpy.array([q1, q2, q3]))
    x, y, z = q1 / m, q2 / m, q3 / m
    obj.rotate(axis=vector(x, y, z), angle=ang)

try:
    ser = serial.Serial('COM8', 115200, timeout=1)
    print("Serial Port Connected")
    
except:
    print("Error: Could not open Serial Port. Check COM number and close other terminals.")
    exit()

def readq(c, i):
    word = ""
    while c[i] != ",":
        word += c[i]
        i += 1
    return float(word)


# ser.open()
scene.range = 3
# while 1:
#     rate(60)
#     line = ser.readline().decode('utf-8', errors='replace').strip()
#     # Expected format: S,q0,q1,q2,q3,D,dq0,dq1,dq2,dq3,E
#     parts = line.split(',')

#     # Basic validation
#     if len(parts) >= 11 and parts[0] == 'S':

#         # Extract Fused Data (Indices 1-4)
#         fq0 = float(parts[1])
#         fq1 = float(parts[2])
#         fq2 = float(parts[3])
#         fq3 = float(parts[4])

#         # Extract Drift Data (Indices 6-9)
#         dq0 = float(parts[6])
#         dq1 = float(parts[7])
#         dq2 = float(parts[8])
#         dq3 = float(parts[9])

#         # Update Graphics
#         update(fused_obj, fq0, fq1, fq2, fq3)
#         update(drift_obj, dq0, dq1, dq2, dq3)
ser.close()
#b'S,0.9916,-0.0514,0.0065,-0.1183,D,0.9886,-0.0849,-0.0343,-0.1195,E\n'
