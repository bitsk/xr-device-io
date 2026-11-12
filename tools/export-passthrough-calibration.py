#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Export a private CV camera-from-XR-body calibration for IMX passthrough.

Rig maps reference-camera points into each camera; SFConfig Stateinit supplies
reference-camera-to-IMU-body rotation/translation. This does not calibrate the
runtime's body-to-eye origin or provide scene depth.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def transpose(a): return [list(row) for row in zip(*a)]
def mm(a, b): return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mv(a, v): return [sum(a[i][k]*v[k] for k in range(3)) for i in range(3)]
def vec(node, key, n):
    value = [float(x) for x in node.get(key, '').split()]
    if len(value) != n or not all(math.isfinite(x) for x in value):
        raise ValueError('Invalid calibration attribute: '+key)
    return value


def rodrigues(v):
    angle = math.sqrt(sum(x*x for x in v))
    if angle < 1e-12: return [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    x,y,z = [x/angle for x in v]
    k = [[0.,-z,y],[z,0.,-x],[-y,x,0.]]
    k2 = mm(k,k)
    return [[float(i==j)+math.sin(angle)*k[i][j]+(1-math.cos(angle))*k2[i][j] for j in range(3)] for i in range(3)]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--calibration', type=Path, required=True)
    p.add_argument('--sensor-config', type=Path, required=True)
    p.add_argument('--rig-convention', choices=('rig-to-camera',), required=True)
    p.add_argument('--mode', choices=('1728x1728','1928x1088'), default='1728x1728')
    p.add_argument('--directory', default='/run/qiyu-passthrough')
    p.add_argument('--output', type=Path, required=True)
    a=p.parse_args()
    if a.output.exists(): p.error('Choose a new private output path')
    root=ET.parse(a.calibration).getroot();modules=json.loads(a.sensor_config.read_text())
    state=root.find('SFConfig/Stateinit')
    if state is None: p.error('Missing camera-to-body calibration')
    body_from_reference=rodrigues(vec(state,'ombc',3)); tbc=vec(state,'tbc',3)
    # QVR portrait-body -> XR: C = Rz(+90 degrees).
    xr_from_body=[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]
    width,height=[int(x) for x in a.mode.split('x')]
    current_origin=(300.,10.) if a.mode=='1728x1728' else (212.,312.)
    eyes=[]
    for slot in (4,5):
        camera=next(x for x in root.findall('Camera') if x.get('id')==str(slot));intr=camera.find('Calibration')
        if intr.get('model')!='FISHEYE_4_PARAMETERS' or vec(intr,'size',2)!=[1728.,1728.]:p.error('Unsupported stock camera calibration')
        module=modules[f'com.qti.sensormodule.qtech_imx471_cam{slot}.bin'];origins=[]
        for table in ('22','206','390'):
            reg={int(x['address'],16):x['data'][0] for x in module['register_tables'][table]};u16=lambda address:reg[address]*256+reg[address+1]
            if reg[0x901]!=0x22 or [u16(0x34c),u16(0x34e)]!=[1728,1728]:p.error('Stock mode disagrees with calibration')
            origins.append((u16(0x344)/2+u16(0x408),u16(0x346)/2+u16(0x40a)))
        if len(set(origins))!=1:p.error('Stock modes use different optical crops')
        shift=[origins[0][i]-current_origin[i] for i in range(2)]
        focal=vec(intr,'focal_length',2);centre=vec(intr,'principal_point',2);distortion=vec(intr,'radial_distortion',len(intr.get('radial_distortion').split()))
        if len(distortion) not in (4,6) or any(distortion[4:]):p.error('Unsupported radial coefficient layout')
        distortion=distortion[:4]
        rig=camera.find('Rig');flat=vec(rig,'rowMajorRotationMat',9);rotation=[flat[i*3:(i+1)*3] for i in range(3)];translation=vec(rig,'translation',3)
        camera_from_view=mm(mm(rotation,transpose(body_from_reference)),transpose(xr_from_body))
        centre_reference=mv(transpose(rotation),[-x for x in translation]);centre_body=[x+y for x,y in zip(mv(body_from_reference,centre_reference),tbc)];centre_xr=mv(xr_from_body,centre_body)
        eyes.append(dict(slot=slot,intrinsics=[focal[0]/2,focal[1]/2,(centre[0]+shift[0])/2,(centre[1]+shift[1])/2],distortion=distortion,camera_from_view=camera_from_view,camera_position_in_xr_body_m=centre_xr,crop_shift_pixels=shift))
    # Physical left/right follows camera centres in XR, not device node or ID.
    eyes.sort(key=lambda eye:eye['camera_position_in_xr_body_m'][0])
    separation=eyes[1]['camera_position_in_xr_body_m'][0]-eyes[0]['camera_position_in_xr_body_m'][0]
    if not .03<separation<.09:p.error('Implausible stereo baseline')
    config=dict(schema='org.openkylin.xr.passthrough',version=2,directory=a.directory,source_size=[width,height],eyes=eyes,colour_gain=[1,1,1],white_balance='gray-world',black_level_10bit=64,
        calibration_sha256=hashlib.sha256(a.calibration.read_bytes()).hexdigest(),rig_convention=a.rig_convention,physical_extrinsics_verified=False,colour_calibrated=False,
        camera_axes='CV +X right, +Y down, +Z forward',geometry_scope='rotation at infinity; OEM camera/body translation exported but not applied without verified body-to-eye origin and scene depth')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(config,indent=2)+'\n');a.output.chmod(0o600)
    print('Wrote private CV camera/body calibration; body-to-eye and wearer acceptance remain separate')


if __name__=='__main__':main()
