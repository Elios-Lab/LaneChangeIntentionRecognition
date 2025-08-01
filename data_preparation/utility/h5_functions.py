import h5py
import numpy as np
from pathlib import Path

def read_h5_content(h5_file: Path) -> dict:
    """
    Reads the content of a given h5 file and returns it as a dict for further use.
    Args:
        h5_file (Path): The path of the h5 file

    Returns:
        dict: A dict containing all the signals in the file
    """
    h5_dict = {}
    with h5py.File(h5_file, mode='r') as h5:
        # Iterate through all groups and datasets
        for key in h5:
            h5_dict[key] = {}
            if isinstance(h5[key], h5py.Dataset):
                for f in h5[key].dtype.fields:
                    if len(h5[key][f].shape) == 1:
                        h5_dict[key][f] = h5[key][f]
                    elif len(h5[key][f].shape) == 2:
                        h5_dict[key][f] = {}
                        for sf in h5[key][f].dtype.fields:
                            h5_dict[key][f][sf] = h5[key][f][sf]
            elif isinstance(h5[key], h5py.Group):
                for g in h5[key]:
                    h5_dict[key][g] = {}
                    for f in h5[key][g].dtype.fields:
                        if len(h5[key][g][f].shape) == 1:
                            h5_dict[key][g][f] = h5[key][g][f]
                        elif len(h5[key][g][f].shape) == 2:
                            h5_dict[key][g][f] = {}
                            for sf in h5[key][g][f].dtype.fields:
                                h5_dict[key][g][f][sf] = h5[key][g][f][sf]
        # Iterate through the file meta data stored as attributes
        if 'metaData' in h5_dict:
            return h5_dict

        h5_dict['metaData'] = {}
        for a in h5.attrs:
            if isinstance(h5.attrs[a], np.ndarray):
                for n in h5.attrs[a].dtype.names:
                    if isinstance(h5.attrs[a][n], np.ndarray):
                        h5_dict['metaData'][n] = {}
                        for m in h5.attrs[a][n].dtype.names:
                            if isinstance(h5.attrs[a][n][m], np.ndarray) and h5.attrs[a][n][m].size == 1:
                                try:
                                    h5_dict['metaData'][n][m] = h5.attrs[a][n][m][0].decode()
                                except AttributeError:
                                    h5_dict['metaData'][n][m] = h5.attrs[a][n][m][0]
                                except UnicodeDecodeError:
                                    try:
                                        h5_dict['metaData'][n][m] = h5.attrs[a][n][m][0].decode( 'unicode_escape')
                                    except:
                                        h5_dict['metaData'][n][m] = h5.attrs[a][n][m][0]
                            else:
                                try:
                                    h5_dict['metaData'][n][m] = h5.attrs[a][n][m].decode()
                                except AttributeError:
                                    h5_dict['metaData'][n][m] = h5.attrs[a][n][m]
                    else:
                        h5_dict[a][n] = h5.attrs[a][n].decode()
            else:
                try:
                    h5_dict['metaData'][a] = h5.attrs[a].decode()
                except AttributeError:
                    h5_dict['metaData'][a] = h5.attrs[a]
    return h5_dict