import sys
import os

INUITIVE_SDK_PATH = os.getenv('INUITIVE_PATH')
if INUITIVE_SDK_PATH is None:
    if sys.platform == 'win32':
        INUITIVE_SDK_PATH = 'C:/Program Files/Inuitive/InuDev/'
    elif sys.platform == 'linux':
        INUITIVE_SDK_PATH = '/opt/Inuitive/InuDev/'
if not os.path.exists(INUITIVE_SDK_PATH):
    raise Exception("Sorry, Inuitive SDK path doesnt exist")
PYTHON_SDK_DIR = os.path.join(INUITIVE_SDK_PATH, 'pythonsdk')
sys.path.insert(1, PYTHON_SDK_DIR)

