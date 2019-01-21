import xml.etree.ElementTree as ET


class Configuration:
    @classmethod
    def from_file(cls, file):
        tree = ET.parse(file)
        return cls(tree.getroot())

    def __init__(self, node):
        self.node = node

    def get(self, key, default=None):
        ret = self.node.find(key)
        if ret is not None:
            ret = [self.parse(elem) for elem in ret.text.split(' ')]
            return ret[0] if len(ret) == 1 else ret
        elif default is not None:
            return default
        else:
            raise KeyError("Couldn't find config and no default provided for config with key: " + key)

    def has_key(self, key):
        return self.node.find(key) is not None

    def find(self, key):
        return Configuration(self.node.find(key))

    def find_all(self, key):
        return [Configuration(child) for child in self.node.findall(key)]

    @property
    def attributes(self):
        return self.node.attrib

    def parse(self, string):
        if string.isdigit():
            return int(string)
        elif self.is_float(string):
            return float(string)
        elif string.lower() == 'true':
            return True
        elif string.lower() == 'false':
            return False
        else:
            return string

    def is_float(self, value):
        try:
            float(value)
            return True
        except ValueError:
            return False
