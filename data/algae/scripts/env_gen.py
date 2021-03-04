import math
from operator import add


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


def generate_row(world_x, world_rpy=[0.0, 0.0, 0.0]):
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
    rope = generate_static_obj_def(name="Rope",
                                   look="black",
                                   mesh="$(param rope)",
                                   world_xyz=[world_x, rope_y, rope_z],
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

    return '\n'.join(components)
