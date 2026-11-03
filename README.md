# fpga
4th year project. Description will be updated.

First commit

Test commit

## Steps to run test_fpga_core

How to run `example/VCU108/fpga_10g/tb/fpga_core/test_fpga_core`

Open terminal in project working directory
First:
```
python3.12 -m venv .venv    # Only seems to work with python 3.12 specifically
```

Then for Linux/MacOS:
```
source .venv/bin/activate
```

Or Windows:
```
venv\Scripts\activate.bat
```

then:
```
pip install -r requirements.txt
cd example/VCU108/fpga_10g/tb/fpga_core/        # Or however windows navigates directories
make SIM=icarus WAVES=1     # Let output complete fully
gtkwave fpga_core.fst       # For waveform viewing
```