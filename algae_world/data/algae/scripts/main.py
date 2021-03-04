"""Generate algae simulation environment files"""

import parser
import random
import subprocess

import env_gen


def generate_algae_env(args):
    environment = []

    environment.append(env_gen.generate_env_tag())
    environment.append(env_gen.generate_material_tag())
    environment.append(env_gen.generate_look_tag(args.algae_texture_dir))

    for i in range(args.num_ropes):
        environment.append(f'\n<!--rope number {i}-->')
        environment.append(
            env_gen.generate_row(world_x=args.rope_x + i * args.dist,
                                 world_rpy=args.world_rpy,
                                 algae_obj_dir=args.algae_obj_dir,
                                 algae_texture_dir=args.algae_texture_dir))

    print(f'Writing env file to {args.output}...')
    with open(args.output, 'w') as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<scenario>\n')
        f.writelines('\n'.join(environment))
        f.write('</scenario>\n')


if __name__ == '__main__':
    random.seed(0)
    SIM_DIR = subprocess.run(['rospack', 'find', 'algae_world'],
                             stdout=subprocess.PIPE,
                             check=True)
    SIM_DIR = SIM_DIR.stdout.decode('utf-8').strip()
    args = parser.parse_args(SIM_DIR)
    print(f'{args}\n')
    generate_algae_env(args)
