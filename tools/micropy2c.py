import argparse
import os
import sys
import glob
import requests
from requests.auth import HTTPBasicAuth

TOOLS_DIR = os.path.dirname(os.path.abspath(sys.argv[0]))

# Python program to translate MicroPython/CircuitPython program to Arduino C
# API for ESP32 boards from Heltec Automation. Requires a translation server
# host and port information.

def translate_py_to_c(args: list, py_file: str) -> (str, str):
    '''
      Translates a single Python program to Arduino C code for the given board.
      If translation is successful, generated C code will be returned,
      otherwise returns None.

      Return type: translated C code, compilation status of C code
    '''
    with open(py_file) as f:
        py_code = f.read()
    try:
        url = f"{args.host}:{args.port}/score"
        response = requests.post(url,
                                 json={"original_code": py_code,
                                       "original_lang": args.source_lang,
                                       "target_lang": "arduino-c",
                                       "target_hardware": args.board},
                                 auth=HTTPBasicAuth(args.user, args.passwd),
                                 verify=os.path.join(TOOLS_DIR, '../doc/hackathon-certfile.pem'))
        if args.verbose:
            print(response.text)
        response_json = response.json()

        if response.status_code == 200:
          # successful response
          return response_json['output_code'], "success" if response_json['metrics']['compilation_success'] else "failure"
        else:
          print("Received error during transpilation:" + response_json["error"]["message"])
          return None, None
    except requests.ConnectionError as e:
        print('ERROR: No one is listening to me! Did you provide a correct host/port?')
        if not args.verbose:
          print('Use verbose (-v) option to get more details if you want.')
        if args.verbose:
            print('Request received exception:' + str(e))
        return None, None
    except requests.RequestException as e:
        print('ERROR: Something is wrong with the request.')
        if not args.verbose:
          print('Use verbose (-v) option to get more details if you want.')
        if args.verbose:
            print('Request received exception:' + str(e))
        return None, None
    except Exception as e:
        print('Request received exception:' + str(e))
        return None, None


parser = argparse.ArgumentParser(
            prog='micropy2c',
            description='Translate MicroPython or CircuitPython program(s) '
                        'to Arduino C SDK for ESP32 boards from Heltec '
                        'Automation.')
parser.add_argument('-b', '--board',
                    help='Heltec board for which to generate Arduino C code',
                    choices=['heltec-wireless-tracker', 'heltec-wifi-lora-v3'],
                    required=True)
parser.add_argument('-s', '--source-file',
                    help='Path to Micro/Circuit Python program file',
                    default='',
                    required=False)
parser.add_argument('-d', '--source-dir',
                    help='Input is a directory containing Python source files',
                    default='',
                    required=False)
parser.add_argument('-o', '--output-dir',
                    help='Directory to store generated Arduino C files',
                    default='/tmp/out',
                    required=False)
parser.add_argument('-l', '--source-lang',
                    help='Language of Python program',
                    choices=['micropython', 'circuitpython'],
                    default='micropython')
parser.add_argument('-m', '--host', help='Translation API host',
                    default='https://localhost')
parser.add_argument('-n', '--port', help='Translation API port',
                    type=int, default=443)
parser.add_argument('-u', '--user', help='Username for translation API',
                    default='')
parser.add_argument('-p', '--passwd', help='Password for translation API',
                    default='')
parser.add_argument('-v', '--verbose', help='Prints response details.',
                    action='store_true')
args = parser.parse_args()

# Sanity checks
if args.source_file == '' and args.source_dir == '':
    print(f'ERROR: Must provide either source_file or source_dir')
    sys.exit(1)
if args.source_file != '' and args.source_dir != '':
    print(f'ERROR: Only supply either source_file or source_dir as input, not both.')
    sys.exit(1)
if args.output_dir == '':
    print(f'ERROR: Must provide output directory')
    sys.exit(1)
if args.source_file == '' and args.source_dir != '' and not os.path.isdir(args.source_dir):
    print(f'ERROR: {args.source_dir} is not a directory')
    sys.exit(1)
if args.source_dir == '' and args.source_file != '' and not os.path.isfile(args.source_file):
    print(f'ERROR: {args.source_file} is not a file')
    sys.exit(1)

# If output directory does not exist, create it.
if os.path.isdir(args.output_dir):
    print(f'{args.output_dir} exists. Please provide an empty directory.')
    sys.exit(1)
else:
    os.mkdir(args.output_dir)
print(f'Outputs will be stored in {args.output_dir}')

# Generate a list of Python programs to translate to Arduino C.
# Not a recursive search - if recursive is needed, set it to True.
if args.source_dir != '' and os.path.isdir(args.source_dir):
    py_prog_paths = [os.path.join(args.source_dir, py_prog)
                     for py_prog in glob.glob("*.py",
                                              root_dir=args.source_dir)]
else:
    py_prog_paths = [args.source_file]

for py_prog_path in py_prog_paths:
    print(f'Translating {py_prog_path}..')
    c_code, compilation_status = translate_py_to_c(args, py_prog_path)
    if c_code is not None:
        # from foo/hello.py, we generate hello and hello/hello.ino
        # get file name (hello.py) first.
        py_prog_name = os.path.split(py_prog_path)[1]
        c_file_dir = os.path.splitext(py_prog_name)[0]
        c_file_dir_path = os.path.join(args.output_dir, c_file_dir)
        # make hello
        os.mkdir(c_file_dir_path)
        c_file_path = os.path.join(c_file_dir_path, c_file_dir + ".ino")
        with open(c_file_path, 'w') as f:
            f.write(c_code)
        print(f'Generated Arduino sketch in {c_file_path}. Compilation:{compilation_status}')
