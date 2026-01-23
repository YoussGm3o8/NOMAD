"""
Start NOMAD Edge Core server for testing.
Run this script, then run test_local_simple.py in another terminal.

Cross-platform development helper script.
"""

import os
import sys
from pathlib import Path

# Set up environment - use script's directory as project root
project_root = Path(__file__).parent.absolute()
os.chdir(project_root)
os.environ['NOMAD_SIM_MODE'] = 'true'
os.environ['PYTHONPATH'] = str(project_root)

# Add to path
sys.path.insert(0, str(project_root))

# Override sys.argv for argparse
sys.argv = ['edge_core.main', '--sim', '--no-vision', '--port', '8000', '--host', '127.0.0.1']

if __name__ == "__main__":
    from edge_core.main import main
    main()
