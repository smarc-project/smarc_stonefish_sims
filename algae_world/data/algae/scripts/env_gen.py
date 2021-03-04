import csv
import math
import os
import random

from operator import add


def generate_env_tag():
    return ('<environment>\n'\
            '\t<ned latitude="$(param latitude)" longitude="$(param longitude)"/>\n'\
            '\t<sun azimuth="20.0" elevation="50.0"/>\n'\
            '\t<ocean enabled="true" waves="0.0"/>\n'\
            '</environment>\n')


def generate_material_tag():
    return ('<materials>\n'\
            '\t<material name="Neutral" density="1000.0" restitution="0.5"/>\n'\
            '\t<material name="Rock" density="3000.0" restitution="0.8"/>\n'\
            '\t<material name="Fiberglass" density="1500.0" restitution="0.3"/>\n'\
            '\t<material name="Aluminium" density="2710.0" restitution="0.7"/>\n'\
            '\t<friction_table>\n'\
                '\t\t<friction material1="Neutral" material2="Neutral" static="0.5" dynamic="0.2"/>\n'\
                '\t\t<friction material1="Neutral" material2="Rock" static="0.2" dynamic="0.1"/>\n'\
                '\t\t<friction material1="Neutral" material2="Fiberglass" static="0.5" dynamic="0.2"/>\n'\
                '\t\t<friction material1="Neutral" material2="Aluminium" static="0.5" dynamic="0.2"/>\n'\
                '\t\t<friction material1="Rock" material2="Rock" static="0.9" dynamic="0.7"/>\n'\
                '\t\t<friction material1="Rock" material2="Fiberglass" static="0.6" dynamic="0.4"/>\n'\
                '\t\t<friction material1="Rock" material2="Aluminium" static="0.6" dynamic="0.3"/>\n'\
                '\t\t<friction material1="Fiberglass" material2="Fiberglass" static="0.5" dynamic="0.2"/>\n'\
                '\t\t<friction material1="Fiberglass" material2="Aluminium" static="0.5" dynamic="0.2"/>\n'\
                '\t\t<friction material1="Aluminium" material2="Aluminium" static="0.8" dynamic="0.5"/>\n'\
            '\t</friction_table>\n'\
            '</materials>\n')


def generate_seafloor_tag():
    return ('\t<static name="Seafloor" type="plane">\n'\
            '\t<material name="Rock"/>\n'\
            '\t<look name="seabed"/>\n'\
            '\t<world_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 15.0"/>\n'\
            '\t</static>\n')


def generate_look_tag(algae_texture_dir=None):
    looks = ['<looks>\n']
    static_looks = (
        '\t<look name="black" gray="0.05" roughness="0.2"/>\n'\
        '\t<look name="yellow" rgb="1.0 0.9 0.0" roughness="0.3"/>\n'\
        '\t<look name="seabed" rgb="0.7 0.7 0.5" roughness="0.9"/>\n'\
        '\t<look name="manipulator" rgb="0.2 0.15 0.1" roughness="0.6" metalness="0.8"/>\n'\
        '\t<look name="orange" rgb="1.0 0.3 0.0" roughness="0.5"/>\n'\
    )
    looks.append(static_looks)

    if algae_texture_dir:
        for texture in os.listdir(algae_texture_dir):
            algae_texture = (f'\t<look name="{texture}" gray="0.1" roughness="0.3" '\
                             f'texture="$(param algae_texture_dir)/{texture}"/>\n')
            looks.append(algae_texture)
    looks.append('</looks>\n')

    return ''.join(looks)


def generate_static_obj_def(name,
                            look,
                            mesh,
                            world_xyz=[0.0, 0.0, 0.0],
                            world_rpy=[0.0, 0.0, 0.0],
                            material='Neutral'):
    """Generate tag for a static object"""
    world_transform_fm = lambda x: ' '.join([f'{i:.2f}' for i in x])
    return (
        f'\n'
        f'<static name="{name}" type="model">\n'
        f'\t<look name="{look}"/>\n'
        f'\t<material name="{material}"/>\n'
        f'\t<world_transform xyz="{world_transform_fm(world_xyz)}" rpy="{world_transform_fm(world_rpy)}"/>\n'
        f'\t<physical>\n'
        f'\t\t<mesh filename="{mesh}" scale="1.0"/>\n'
        f'\t\t<origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>\n'
        f'\t</physical>\n'
        f'</static>\n')


def generate_a0_buoy_group(world_xyz, world_rpy):
    """Generate A0 buoy group with a buoy and a 2m rope"""
    a0_buoy = generate_static_obj_def(name="A0_Buoy",
                                      look="yellow",
                                      mesh="$(param a0_buoy)",
                                      material="Neutral",
                                      world_xyz=world_xyz,
                                      world_rpy=world_rpy)
    a0_buoy_rope = generate_static_obj_def(name="A0Buoy_Rope",
                                           look="black",
                                           mesh="$(param a0_buoy_rope)",
                                           material="Neutral",
                                           world_xyz=world_xyz,
                                           world_rpy=world_rpy)
    return ''.join([a0_buoy, a0_buoy_rope])


def generate_anchoring_group(world_xyz, world_rpy):
    """Generate anchoring buoy and connected rope (used at the ending points of the ropes)"""
    anchoring_buoy = generate_static_obj_def(name="Anchoring_Buoy",
                                             look="orange",
                                             mesh="$(param anchoring_buoy)",
                                             material="Neutral",
                                             world_xyz=world_xyz,
                                             world_rpy=world_rpy)
    anchoring_buoy_rope = generate_static_obj_def(
        name="Anchoring_Buoy_Rope",
        look="black",
        mesh="$(param anchoring_buoy_rope)",
        material="Neutral",
        world_xyz=world_xyz,
        world_rpy=world_rpy)
    return ''.join([anchoring_buoy, anchoring_buoy_rope])


def get_algae_obj_and_textures(algae_obj_dir, algae_texture_dir):
    """ Given algae_obj_dir and algae_texture_dir, return a dictionary
    with two entries:
        - algae_obj: dictionary with (key, value) = (algea_id, algea xyz dim)
        - algae_texture: list with available texture file names
    """
    algae_obj = {}
    with open(os.path.join(algae_obj_dir, 'algae_info.csv')) as f:
        reader = csv.DictReader(f)
        for row in reader:
            algae_obj[int(row['name'])] = {
                'name': row['name'],
                'x': float(row['x']),
                'y': float(row['y']),
                'z': float(row['z'])
            }
    return {
        'algae_obj': algae_obj,
        'algae_texture': os.listdir(algae_texture_dir)
    }


def generate_algae(look, mesh, world_xyz, world_rpy):
    """Generate one algae static object"""
    return generate_static_obj_def(name="Algae",
                                   look=look,
                                   mesh=mesh,
                                   world_xyz=world_xyz,
                                   world_rpy=world_rpy,
                                   material='Neutral')


def generate_algae_row(rope_world_xyz, rope_world_rpy, algae_data):
    """Given the world transform of the rope and the algae_data dict,
    generate the algaes hanging on the rope"""
    algae_row = []
    current_y = rope_world_xyz[1] - 95
    limit_y = rope_world_xyz[1] + 95
    num_algae_obj = len(algae_data['algae_obj'])
    num_algae_texture = len(algae_data['algae_texture'])

    while current_y < limit_y:
        mesh_idx = random.randint(0, num_algae_obj - 1)
        mesh_dim = algae_data['algae_obj'][mesh_idx]
        texture_idx = random.randint(0, num_algae_texture - 1)
        sep = random.uniform(.05, .4)
        current_y += sep + mesh_dim['y']
        look = algae_data['algae_texture'][texture_idx]
        algae = generate_algae(look=f'{look}',
                               mesh=f'$(param algae_obj_dir)/{mesh_idx}.obj',
                               world_xyz=[
                                   rope_world_xyz[0], current_y,
                                   rope_world_xyz[2] + mesh_dim['z'] / 2
                               ],
                               world_rpy=rope_world_rpy)
        algae_row.append(algae)
    return ''.join(algae_row)


def generate_row(world_x,
                 world_rpy=[0.0, 0.0, 0.0],
                 algae_obj_dir=None,
                 algae_texture_dir=None):
    """Generate a row of algaes using the given world transform"""
    components = []
    buoy_z = 0
    rope_y = 0
    rope_z = buoy_z + 2
    anchoring_buoy_params = {
        'west': {
            'y_offset': -95,
            'rpy': [0, 0, 3.14]
        },
        'east': {
            'y_offset': 95,
            'rpy': [0, 0, 0]
        }
    }

    # Add rope
    rope_world_xyz = [world_x, rope_y, rope_z]
    rope = generate_static_obj_def(name="Rope",
                                   look="black",
                                   mesh="$(param rope)",
                                   world_xyz=rope_world_xyz,
                                   world_rpy=world_rpy)
    components.append(rope)

    # Add A0 buoys, 1 per 10 meters
    for y in range(rope_y - 85, rope_y + 90, 10):
        a0_buoy = generate_a0_buoy_group(world_xyz=[world_x, y, buoy_z],
                                         world_rpy=world_rpy)
        components.append(a0_buoy)

    # Add anchoring to the end of the rope
    for _, param in anchoring_buoy_params.items():
        anchoring_buoy = generate_anchoring_group(
            world_xyz=[world_x, rope_y + param['y_offset'], buoy_z],
            world_rpy=list(map(add, world_rpy, param['rpy'])))
        components.append(anchoring_buoy)

    # Attach algaes to the rope if both algae_obj_dir algae_texture_dir given
    if algae_obj_dir and algae_texture_dir:
        algae_data = get_algae_obj_and_textures(algae_obj_dir,
                                                algae_texture_dir)

        algae_row = generate_algae_row(rope_world_xyz=rope_world_xyz,
                                       rope_world_rpy=world_rpy,
                                       algae_data=algae_data)
        components.append(algae_row)

    return '\n'.join(components)
