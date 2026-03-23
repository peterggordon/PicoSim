# PicoSim

A Python-based desktop operator interface for Picosun ALD tools, 
communicating directly with the Omron PLC over FINS/UDP. Developed 
as a community replacement for the original HMI software, which is 
no longer supported following Picosun's acquisition by ASM.

## Background

This software was developed by reverse-engineering the PLC communication 
protocol of a Picosun R200 ALD tool at the Carleton University Nanofab 
Facility. Picosun as an independent company no longer exists — their 
product line was absorbed by ASM. As a result, these tools receive no 
software support. PicoSim was created to fill that gap.

Tools used to create this software include: ChatGPT & Claude for coding,
Wireshark for capturing traffic between Picosun's HMI and the PLC, and
CX-One for reading the PLC code. Without those tools, this project would
never have happened. 

## Important Disclaimers

- This software has so far only been tested on a single Picosun R200 tool. 
  Behavior on other tools (R150, R200 Advanced, etc.) is unknown.
- This is not commercial software. It is not to be sold or used for 
  commercial gain.
- Use at your own risk. This software controls real hardware. Incorrect 
  configuration could damage equipment or create unsafe conditions.
- This project has no affiliation with Picosun or ASM.


## Intended Use

PicoSim is intended for research institutions, universities, and 
community labs that operate legacy Picosun tools and have lost access 
to vendor support. You are encouraged to:

- Use the software as-is on your own tool
- Modify it to suit your tool's specific configuration
- Use any part of the code as a starting point for your own PLC 
  interface projects

## Getting Started

### Requirements
- Python 3.11 or later
- Windows (developed and tested on Windows 10/11)
- Dependencies: see requirements.txt

### Installation
```
pip install -r requirements.txt
python main.py
```

### Running the pre-built executable
Download PicoSim.exe from the Releases page. No Python installation 
required. Place it in any folder and run it directly. You will 
need to know the IP address of your PC and PLC. 

## Adapting for Your Tool

PicoSim was built around the Picosun R200 PLC memory map. If you are 
adapting it for a different tool or configuration, the key files to 
modify are:

- `data_parser.py` — CIO snapshot field map and scaling
- `main.py` — SETPOINT_COMMANDS and VALVE_COMMANDS dictionaries
- `network.py` — IP addresses and FINS node numbers

All hardware-verified mappings are commented in the code.

## License

MIT License — see LICENSE file. You are free to use, modify, and 
distribute this code. Commercial use is permitted by the license but 
is contrary to the spirit of this project.

## Contributing

Pull requests are welcome. If you have successfully adapted PicoSim 
for a different tool, please consider contributing your configuration 
back to the repository. If you want to contact me directly, please
use peter.gordon@carleton.ca. 
