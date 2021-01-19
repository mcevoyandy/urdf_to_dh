import xml.etree.ElementTree as ET
import numpy as np

# Helper functions for parsing the URDF
def get_urdf_root(urdf_file):
    """Parse a URDF for joints.

    Args:
        urdf_path (string): The absolute path to the URDF to be analyzed.

    Returns:
        root (xml object): root node of the URDF.
    """
    try:
        tree = ET.parse(urdf_file)
    except ET.ParseError:
        print('ERROR: Could not parse urdf file.')

    return tree.getroot()

def process_joint(joint):
    """Extracts the relevant joint info into a dictionary.
    Args:
    Returns:
    """
    axis = np.array([1, 0, 0])
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    parent_link = ''
    child_link = ''

    joint_name = joint.get('name')

    for child in joint:
        if child.tag == 'axis':
            axis = np.array(child.get('xyz').split(), dtype=float)
        elif child.tag == 'origin':
            xyz = np.array(child.get('xyz').split(), dtype=float)
            rpy = np.array(child.get('rpy').split(), dtype=float)
        elif child.tag == 'parent':
            parent_link = child.get('link')
        elif child.tag == 'child':
            child_link = child.get('link')
    return joint_name, {'axis': axis, 'xyz': xyz, 'rpy': rpy, 'parent': parent_link, 'child': child_link, 'dh': np.zeros(4)}
