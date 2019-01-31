import json
import os
import shutil

import numpy as np


class NumpyEncoder(json.JSONEncoder):
    # Encodes numpy arrays as lists since JSON does not support np.ndarray
    def default(self, obj):
        return obj.tolist() if isinstance(obj, np.ndarray) else json.JSONEncoder.default(self, obj)


class Configuration(object):

    def __init__(self, configuration_directory):
        self.config = None
        self.file_path = os.path.join(configuration_directory, 'configuration.json')

        if not os.path.isfile(self.file_path):
            shutil.copy2(os.path.join(configuration_directory, 'default_configuration.json'), self.file_path)

    def load(self):
        # Load the configuration JSON file
        with open(self.file_path, 'r') as fp:
            self.config = json.load(fp)

        return self.config

    def save(self):
        # Save the configurations in a JSON file
        with open(self.file_path, 'w') as fp:
            json.dump(self.config, fp, sort_keys=True, indent=2, cls=NumpyEncoder)

    def update(self, field, value):
        # Update a value in the configuration
        self.config[field] = value
        self.save()
