import yaml
import os

# class and singleton to distinguish whether or not user has passed us a default value
class _Unspecified(object): pass
_unspecified = _Unspecified()


class Configuration:
    @classmethod
    def from_file(cls, file):
        with open(file, 'r') as stream:
            try:
                top = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        return cls(top['name'] if 'name' in top else os.path.splitext(os.path.basename(file))[0], top)

    def __init__(self, name, node):
        self.node = node
        self.name = name

    def get(self, key, default=_unspecified):
        if key in self.node:
            if isinstance(self.node[key], dict):
                return Configuration(key, self.node[key])
            else:
                return self.node[key]
        elif default is not _unspecified:
            return default
        else:
            raise KeyError("Couldn't find config and no default provided for config with key: " + key)

    def set(self, key, val):
        self.node[key] = val

    def __contains__(self, key):
        return key in self.node

    def find_all(self, key):
        ''' Finds the first instance of a config which contains the given key

        Returns: 
            iter(Configuration): An iterator for accessing every nested config which contains the specified key 
        '''
        for k, v in self.node.items():
            if isinstance(v, dict) and key in v:
                yield Configuration(k, v)

    def find(self, key):
        ''' Finds the first instance of a config which contains the given key

        Returns: 
            dict: The first nested config which contains the specified key
        '''
        return next(iter(self.find_all(key)))
