import ctypes
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
BIN_DIR = os.path.join(INUITIVE_SDK_PATH, 'bin')
PYTHON_API_DIR = os.path.join(PYTHON_SDK_DIR, 'api')

if sys.platform == 'win32':
    myDll1 = ctypes.CDLL(os.path.join(BIN_DIR, "InuCommonUtilities.dll"))
    myDll2 = ctypes.CDLL(os.path.join(BIN_DIR, "InuStreams.dll"))
    myDll4 = ctypes.CDLL(os.path.join(PYTHON_API_DIR, "InuStreamsPyth.pyd"))
elif sys.platform == 'linux':
    myDll1 = ctypes.CDLL(os.path.join(BIN_DIR, "libInuCommonUtilities.so"))
    myDll2 = ctypes.CDLL(os.path.join(BIN_DIR, "libInuStreams.so"))
    import importlib.util

    def load_python_wrapper(module_name: str):
        # @brief    loadPythonWrapper
        #
        # The loadPythonWrapper function checks the Python version using sys.version_info and imports the appropriate
        #   module based on the version.
        # @param  iModuleName common module name
        # @return wrapper module
        python_version = sys.version_info
        print(f"Python version {python_version}")
        if python_version >= (3, 10):
            specific_file_name = "lib" + module_name + "_3_10_12.so"
        elif python_version >= (3, 6):
            specific_file_name = "lib" + module_name + "_3_6_9.so"
        else:
            raise RuntimeError("Unsupported Python version")
        # Modify sys.path to include the directory containing the specific .so file
        specific_file_path = os.path.join(PYTHON_API_DIR, specific_file_name)
        try:
            module_name = "api.InuStreamsPyth"
            spec = importlib.util.spec_from_file_location(module_name, specific_file_path)
            wrapper_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(wrapper_module)
        except ImportError:
            raise ImportError(f"Failed to import module {specific_file_name}")
        # Remove the directory from sys.path after importing the module
        sys.path.pop(0)
        return wrapper_module

    # Load the appropriate Python wrapper
    inu_streams = load_python_wrapper("InuStreamsPyth")
else:
    raise Exception("This platform doesn't  supported yet.")
