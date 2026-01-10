"""
Start NOMAD Edge Core server for testing.
Run this script, then run test_local_simple.py in another terminal.
"""

import os
import sys

# Set up environment
os.chdir(r'c:\Users\Youssef\Documents\Code\MAD\NOMAD')
os.environ['NOMAD_SIM_MODE'] = 'true'
os.environ['PYTHONPATH'] = os.getcwd()

# Add to path
sys.path.insert(0, os.getcwd())

# Override sys.argv for argparse
sys.argv = ['edge_core.main', '--sim', '--no-vision', '--port', '8000', '--host', '127.0.0.1']

if __name__ == "__main__":
    from edge_core.main import main
    main()
