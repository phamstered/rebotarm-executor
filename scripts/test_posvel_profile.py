#!/usr/bin/env python3
"""No hardware: configured limits, adapter stops, isolated build and flash routing."""
from pathlib import Path
import os
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
def run(args, ok=True, **kw):
    p=subprocess.run(args,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,**kw)
    if (p.returncode==0)!=ok: raise AssertionError(p.stdout)
    return p.stdout
with tempfile.TemporaryDirectory() as d:
    d=Path(d)
    for enabled in (0,1):
        exe=str(d/f'cap-{enabled}')
        run(['cc','-std=c11','-Wall','-Wextra','-Werror','-IUser',f'-DMOTOR_POSVEL_URDF_PROFILE={enabled}',
             'scripts/test_motor_capabilities.c','User/motor_control_v2.c','-lm','-o',exe])
        print(run([exe]))
    exe=str(d/'adapter')
    run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Iscripts/host_stubs','-IUser','-DMOTOR_POSVEL_URDF_PROFILE=1',
         'scripts/test_motor_bridge_v2.c','User/motor_control_v2.c','User/damiao_protocol.c','-lm','-o',exe])
    print(run([exe]))
    source=d/'source';source.mkdir()
    (source/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.22)\nproject(test NONE)\n'+
        f'include("{ROOT}/cmake/posvel-profile.cmake")\n')
    for name,args,ok in [('default',[],True),('explicit',['-DMOTOR_POSVEL_URDF_PROFILE=ON'],True),
                         ('Debug',['-DMOTOR_POSVEL_URDF_PROFILE=ON'],False),
                         ('Release',['-DMOTOR_POSVEL_URDF_PROFILE=ON'],False),
                         ('conflict',['-DMOTOR_POSVEL_URDF_PROFILE=ON','-DMOTOR_BENCH_JOINT7_POSVEL=ON'],False)]:
        run(['cmake','-S',str(source),'-B',str(d/name),*args],ok)
    for args in (['--posvel-seven'],['--posvel-seven','--bench-joint7','--build-only'],
                 ['--posvel-seven','--disable-overspeed-stop','--build-only'],
                 ['--posvel-seven','--disable-effort-stop','--build-only'],
                 ['--posvel-seven','--position-min','0','--build-only']):
        run(['bash','scripts/flash.sh',*args],False)
print('seven-axis build and flash guards passed')
