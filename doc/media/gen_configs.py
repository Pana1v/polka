#!/usr/bin/env python3
"""
Generate per-feature polka configs for the demo GIFs.

All configs share a base (3-LiDAR sources, os_sensor output frame); each feature
variant flips one knob. Run: `python3 gen_configs.py` -> writes media/configs/*.yaml
"""
import pathlib

import yaml

HERE = pathlib.Path(__file__).parent
OUT = HERE / 'configs'

SOURCES_3 = ['ouster', 'avia', 'mid360']
SRC_DEF = {
    'ouster': {'topic': '/ouster/points'},
    'avia': {'topic': '/avia/points'},
    'mid360': {'topic': '/mid360/points'},
}


def source_block(names):
    # reliable + deeper queue: the 2.3 MB Ouster cloud is dropped under
    # best_effort/depth-1 when the single-threaded executor is busy.
    return {
        n: {
            'topic': SRC_DEF[n]['topic'],
            'type': 'pointcloud2',
            'qos_reliability': 'reliable',
            'qos_history_depth': 10,
        }
        for n in names
    }


def base():
    return {'polka': {'ros__parameters': {
        'output_frame_id': 'os_sensor',
        'output_rate': 15.0,
        'enable_gpu': True,
        'source_timeout': 0.5,
        'timestamp_strategy': 'earliest',
        'outputs': {
            'cloud': {
                'enabled': True, 'topic': '~/merged_cloud',
                'filters': {
                    'range': {'enabled': False, 'min': 0.5, 'max': 6.0},
                    'angular': {'enabled': False, 'invert': False, 'ranges': [315.0, 45.0]},
                    'box': {'enabled': False, 'x_min': -3.0, 'x_max': 3.0,
                            'y_min': -3.0, 'y_max': 3.0, 'z_min': -2.0, 'z_max': 2.0},
                },
                'height_cap': {'enabled': False, 'z_min': -0.6, 'z_max': 1.0},
                'self_filter': {'enabled': False, 'box_names': ['chassis'],
                                'chassis': {'x_min': -0.6, 'x_max': 0.6,
                                            'y_min': -0.6, 'y_max': 0.6,
                                            'z_min': -0.5, 'z_max': 0.5}},
                # leaf_size must stay 0 here: polka enables voxel whenever any
                # leaf > 0, regardless of the `enabled` flag. Only voxel_on sets it.
                'voxel': {'enabled': False, 'leaf_size': 0.0},
            },
            'scan': {'enabled': False, 'topic': '~/merged_scan',
                     # wider vertical slice: captures more of each non-repetitive
                     # Livox swath per frame -> denser, less flickery 2D scans
                     'z_min': -1.0, 'z_max': 1.5,
                     'angle_min': -3.14159265, 'angle_max': 3.14159265,
                     'angle_increment': 0.00436332, 'range_min': 0.2, 'range_max': 20.0},
        },
        'source_names': list(SOURCES_3),
        'sources': source_block(SOURCES_3),
    }}}


def write(name, cfg):
    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / f'{name}.yaml', 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)


def cloud(cfg):
    return cfg['polka']['ros__parameters']['outputs']['cloud']


def main():
    configs = {}

    # merged (the common "before" for filter/voxel/self demos) and single (fusion)
    configs['merged'] = base()
    single = base()
    single['polka']['ros__parameters']['source_names'] = ['ouster']
    single['polka']['ros__parameters']['sources'] = source_block(['ouster'])
    configs['single'] = single

    # output filters, each alone
    c = base()
    cloud(c)['filters']['range']['enabled'] = True
    configs['filter_range'] = c
    c = base()
    cloud(c)['filters']['angular']['enabled'] = True
    configs['filter_angular'] = c
    c = base()
    cloud(c)['filters']['box']['enabled'] = True
    configs['filter_box'] = c
    c = base()
    cloud(c)['height_cap']['enabled'] = True
    configs['filter_height'] = c

    # angular invert flag
    c = base()
    cloud(c)['filters']['angular']['enabled'] = True
    cloud(c)['filters']['angular']['invert'] = False
    configs['invert_keep'] = c
    c = base()
    cloud(c)['filters']['angular']['enabled'] = True
    cloud(c)['filters']['angular']['invert'] = True
    configs['invert_exclude'] = c

    # self filter
    c = base()
    cloud(c)['self_filter']['enabled'] = True
    configs['self_on'] = c

    # voxel
    c = base()
    cloud(c)['voxel']['enabled'] = True
    cloud(c)['voxel']['leaf_size'] = 0.2
    configs['voxel_on'] = c

    # cpu vs cuda
    c = base()
    c['polka']['ros__parameters']['enable_gpu'] = False
    configs['cpu'] = c
    # (cuda == merged: enable_gpu True)

    # dual output (cloud + scan)
    c = base()
    c['polka']['ros__parameters']['outputs']['scan']['enabled'] = True
    configs['dual'] = c

    # per-source 2D scans (single source, scan output) for the scan-merge demo
    for src in SOURCES_3:
        c = base()
        c['polka']['ros__parameters']['source_names'] = [src]
        c['polka']['ros__parameters']['sources'] = source_block([src])
        c['polka']['ros__parameters']['outputs']['scan']['enabled'] = True
        configs[f'{src}_scan'] = c

    for name, cfg in configs.items():
        write(name, cfg)
    print(f'wrote {len(configs)} configs to {OUT}')
    for n in configs:
        print('  ', n)


if __name__ == '__main__':
    main()
