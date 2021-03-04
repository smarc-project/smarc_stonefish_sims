import argparse


def parse_args(SIM_DIR):
    parser = argparse.ArgumentParser(
        description='Generate an algae farm simulation environment')

    parser.add_argument('--rope_x',
                        default=4.,
                        type=float,
                        help='world x position for the first rope')
    parser.add_argument('--world_rpy',
                        default=[0., 0., 0.],
                        type=list,
                        help='world rpy')
    parser.add_argument('--num_ropes',
                        default=2,
                        type=int,
                        help='number of ropes in the environment')
    parser.add_argument('--dist',
                        default=4.,
                        type=float,
                        help='distance between two ropes')
    parser.add_argument('--add_algae',
                        action='store_true',
                        help='whether there should be algae on the rope')
    parser.add_argument('--algae_obj_dir',
                        default=f'{SIM_DIR}/data/algae/algae_cube',
                        help='path to the folder with algae .obj files')
    parser.add_argument(
        '--algae_texture_dir',
        default=f'{SIM_DIR}/data/algae/textures',
        help='path to the folder with algae texture image files')

    parser.add_argument('-o',
                        '--output',
                        default=f'{SIM_DIR}/data/algae/algae_env.scn',
                        help='path for the output environment file')
    return parser.parse_args()
