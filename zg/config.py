"""Config loading: mission.yaml + swarm.yaml + chaos.yaml merged into one dict."""
import copy
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(ROOT, "config")


def _deep_update(base, upd):
    for k, v in upd.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_update(base[k], v)
        else:
            base[k] = v
    return base


def load_config(overrides=None, config_dir=CONFIG_DIR):
    """Return {'mission':..., 'swarm':..., 'chaos':...}, optionally deep-updated."""
    cfg = {}
    for name in ("mission", "swarm", "chaos"):
        with open(os.path.join(config_dir, f"{name}.yaml")) as f:
            cfg[name] = yaml.safe_load(f)
    if overrides:
        _deep_update(cfg, copy.deepcopy(overrides))
    return cfg
