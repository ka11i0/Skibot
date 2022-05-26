from numpy import angle
import pandas as pd
from math import sqrt
from datahandling.DataHandler import *


folder = "0/"
xyfile = folder+"xy.csv"
rxy_file = folder+"r_xy.csv"
angles_file = folder+"angles.csv"
rangles_file = folder+"r_angles.csv"
t_file = folder+"t.csv"

xyerrorpath = folder+"xy_error.csv"
angleerror = folder+"a_error.csv"
dtxy = folder+"dt_xy.csv"
dtangle = folder+"dt_angles.csv"

datahandler=DataHandler()

xy_file = pd.read_csv(xyfile)
r_xy_file = pd.read_csv(rxy_file)
anglesfile = pd.read_csv(angles_file)
r_anglesfile = pd.read_csv(rangles_file)
tfile = pd.read_csv(t_file)
indexes = []
for index, row in anglesfile.iterrows():
    indexes.append(int(row["i1"]))

r_xy = []
for index, row in r_xy_file.iterrows():
    r_xy.append((row[0], row[1]))

errors = []
for index, xy in xy_file.iterrows():
    er = sqrt( (r_xy[indexes[index]][0]-xy[0])**2+(r_xy[indexes[index]][1]-xy[1])**2 )
    el = sqrt( (r_xy[indexes[index]][0]-xy[2])**2+(r_xy[indexes[index]][1]-xy[3])**2 )
    errors.append((er,el))

datahandler.save_to_csv(errors, ["er", "el"], xyerrorpath)


r_angles = []
for index, row in r_anglesfile.iterrows():
    r_angles.append((row[0], row[1]))

errors = []
for index, angles in anglesfile.iterrows():
    er0 = angles[0]-r_angles[indexes[index]][0]
    er1 = angles[1]-r_angles[indexes[index]][1]
    el0 = angles[2]-r_angles[indexes[index]][0]
    el1 = angles[3]-r_angles[indexes[index]][1]
    errors.append((er0,er1,el0,el1))

t = []
for index, t_s in tfile.iterrows():
    t.append(t_s[0])

dtime = []
for index, t_s in tfile.diff(periods=1,axis=0).iterrows():
    dtime.append(t_s[0])

xy_file['t'] = tfile['t']
dt = []
for index, xy in xy_file.diff(periods=1,axis=0).rolling(window=30).mean().iterrows():
    if(index==0):
        continue
    dt_r = sqrt(xy[0]**2+xy[1]**2)/dtime[index]
    dt_l = sqrt(xy[2]**2+xy[3]**2)/dtime[index]
    dt.append((t[index-1],dt_r,dt_l))

datahandler.save_to_csv(dt, ["t", "dt_r", "dt_l"], dtxy)

dt = []
anglesfile['t'] = tfile['t']
for index, angles in anglesfile.diff(periods=1,axis=0).rolling(window=30).mean().iterrows():
    if(index==0):
        continue
    dt_r0 = angles[0]/dtime[index]
    dt_r1 = angles[1]/dtime[index]
    dt_l0 = angles[2]/dtime[index]
    dt_l1 = angles[3]/dtime[index]
    dt.append((t[index-1],dt_r0,dt_r1,dt_l0,dt_l1))

datahandler.save_to_csv(dt, ["t", "dt_r0", "dt_r1", "dt_l0", "dt_l1"], dtangle)