import bpy
import csv
import math
import os
import random
import subprocess

OBJ_NAME = 'algae_cube'  # OR 'algae_plane'
NUM_OBJ = 200

sim_dir = subprocess.run(['rospack', 'find', 'algae_world'],
                         stdout=subprocess.PIPE)
sim_dir = sim_dir.stdout.decode('utf-8').strip()
OUTDIR = f'{sim_dir}/data/algae/{OBJ_NAME}'
if not os.path.exists(OUTDIR):
    os.makedirs(OUTDIR)
OBJ_INFO_PATH = os.path.join(OUTDIR, 'algae_info.csv')
min_displace, max_displace = -2, 2
min_deform, max_deform = 0, .3

view_layer = bpy.context.view_layer
algae_obj = bpy.data.objects['algae_cube']
mod_dx = algae_obj.modifiers['displace_x']
mod_dy = algae_obj.modifiers['displace_y']
mod_deform = algae_obj.modifiers['simple_deform']
deform_methods = ['TWIST', 'BEND', 'TAPER', 'STRETCH']
deform_axis = ['X', 'Y', 'Z']
# Reduce number of faces
mod_decimate = algae_obj.modifiers['decimate']


def generate_algae_obj(filepath):
    mod_dx.strength = random.uniform(min_displace, max_displace)
    mod_dy.strength = random.uniform(min_displace, max_displace)

    mod_deform.deform_method = deform_methods[random.randint(
        0,
        len(deform_methods) - 1)]
    mod_deform.deform_axis = deform_axis[random.randint(
        0,
        len(deform_axis) - 1)]
    mod_deform.factor = random.uniform(min_deform, max_deform)

    # Reduce number of faces
    if random.random() > 0.5:
        mod_decimate.decimate_type = 'DISSOLVE'
        mod_decimate.angle_limit = random.randint(3, 10) / math.pi
    else:
        mod_decimate.decimate_type = 'UNSUBDIV'
        mod_decimate.iterations = random.randint(5, 10)

    algae_obj_dim = {
        'x': random.uniform(.05, .25),
        'y': random.uniform(.01, .1),
        'z': random.uniform(2, 5)
    }
    algae_obj.dimensions[0] = algae_obj_dim['x']
    algae_obj.dimensions[1] = algae_obj_dim['y']
    algae_obj.dimensions[2] = algae_obj_dim['z']

    bpy.ops.object.select_all(action='DESELECT')
    algae_obj.select_set(state=True)
    view_layer.objects.active = algae_obj
    bpy.ops.export_scene.obj(filepath=filepath,
                             use_selection=True,
                             use_mesh_modifiers=True,
                             use_materials=True,
                             use_triangles=True)
    return algae_obj_dim


obj_info = dict()
for i in range(NUM_OBJ):
    filepath = os.path.join(OUTDIR, f'{i}.obj')
    algae_obj_dim = generate_algae_obj(filepath)
    algae_obj_dim['name'] = i
    obj_info[i] = algae_obj_dim
    print(f'written: {filepath}')

with open(OBJ_INFO_PATH, 'w') as f:
    csv_writer = csv.DictWriter(f,
                                delimiter=',',
                                fieldnames=['name', 'x', 'y', 'z'])
    csv_writer.writeheader()
    csv_writer.writerows(obj_info.values())
