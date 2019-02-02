from gym import spaces
import numpy as np


def get_bounds_for_space(space, low_not_high):
    if isinstance(space, spaces.Discrete):
        return 0 if low_not_high else space.n
    elif isinstance(space, spaces.MultiDiscrete):
        return np.zeros(space.nvec) if low_not_high else np.ones(space.nvec)*space.nvec
    elif isinstance(space, spaces.MultiBinary):
        return np.zeros(space.n) if low_not_high else np.ones(space.n)
    elif isinstance(space, spaces.Box):
        return space.low if low_not_high else space.high
    elif isinstance(space, spaces.Dict):
        return {key: get_bounds_for_space(subspace, low_not_high) for key, subspace in space.spaces.items()}
    elif isinstance(space, spaces.Tuple):
        return (get_bounds_for_space(subspace, low_not_high) for subspace in space.spaces)
    else:
        try:
            return space.low if low_not_high else space.high
        except AttributeError:
            raise AttributeError("Could not find lower bound for space, did you define your own gym.Space? If so you'll need to define its upper and lower bound so that we can flatten it properly")

def get_desc_for_space(space, prepend=''):
    l = []
    for key, val in space.spaces.items():
        if isinstance(val, spaces.Dict):
            l.extend(get_desc_for_space(val, prepend+'/'+key))
        else:
            l.append(prepend+'/'+key)

    return l

def walk_dict(d, func=sum):
    return func(walk_dict(e) if isinstance(e, dict) else e for e in d.values())

def flatten(to_flatten):
    def _flatten(to_flatten):
        l = []
        if isinstance(to_flatten, dict):
            for val in to_flatten.values():
                l.extend(_flatten(val))
        elif isinstance(to_flatten, tuple):
            for val in to_flatten.values():
                l.extend(_flatten(val))
        else:
            l.append(np.reshape(to_flatten, -1))

        return l

    return np.concatenate(_flatten(to_flatten))

def unflatten(to_unflatten, action_space):
    def _unflatten(extractor, space):
        if isinstance(space, spaces.Dict):
            return {key: _unflatten(extractor, subspace) for key, subspace in space.spaces.items()}
        elif isinstance(space, spaces.Tuple):
            return (_unflatten(extractor, subspace) for subspace in space.spaces)
        elif isinstance(space, spaces.Discrete):
            return int(extractor.pop(1))
        elif isinstance(space, spaces.MultiDiscrete):
            return extractor.pop(space.nvec).astype(np.uint8)
        elif isinstance(space, spaces.MultiBinary):
            return extractor.pop(space.n).astype(np.uint8)
        elif isinstance(space, spaces.Box):
            return extractor.pop(space.low.size).astype(space.low.dtype).reshape(space.low.shape)
        else:
            raise AttributeError("Unrecognised datatype in action_space; spaces other than those defined by the gym aren't currently supported by flatten_actions" )

    return _unflatten(ArrayExtractor(to_unflatten), action_space)

class ArrayExtractor:
    def __init__(self, arr):
        self.arr = arr
        self.i = 0

    def pop(self, n):
        self.i += n
        return self.arr[self.i-n:self.i]
