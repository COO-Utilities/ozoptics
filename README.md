# ozoptics_software

Low-level python modules for operating OZ Optics attenuators

## Currently Supported Models
- DD-100-MC (RS232)
- DD-600-MC (RS232)

## Features
- Connect to OZ Optics attenuators over serial through a terminal server
- Query attenuator state and parameters
- Command full range of attenuations


## Usage

```python
import dd100mc

controller = dd100mc.Controller()
controller.connect(host='192.168.29.222', port=10010)

# For a comprehensize list of classes and methods, use the help function
help(dd100mc)
```

## 🧪 Testing
Unit tests are located in `tests/` directory.

To run all tests from the project root:

```bash
pytest
```