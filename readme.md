odri_control_interface
----------------------

### What it is

Common interface for controlling robots build with the odri master board.

### Installation

#### Download the package:

We use `treep` to download the required packages. Make sure your ssh key is unlocked. Then

```
mkdir -p ~/devel
pip install treep  # This installs treep
cd ~/devel
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
treep --clone master-board
treep --clone odri_control_interface 
```

#### Build the package

We use [colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon)
to build this package:
```
cd mkdir -p ~/devel/workspace
colcon build
```
### Usage:

#### Demos/Examples

You find examples in forms of demos under the `demos/` folder. The demos show how to use the python and C++ interface for talking to the library and setting up a robot.

##### Python examples

If you're using nix, you can get into a shell with all required packages available, including the C++-backed python packages

```
nix develop .#python
```

Since the SDK forges raw ethernet frames, you need to run these scripts as root (run it in a shell that has the dependencies installed and PYTHONPATH set):

```
sudo -HE env PATH=$PATH PYTHONPATH=$PYTHONPATH python demos/demo_testbech_joint_calibrator.py
```

Note that running within a network-mounted directory (e.g. using NFS) might result in a "Permission denied" error when trying to make python execute the script as root, even when all users have the `r`ead permission set.

_(LAAS members: this is the case for your home directory when using office computers)_

##### C++ examples

C++ demos also need to be run as root. Compilation produces binaries named `odri_control_interface_*`. (nix: you can do `nix run .#testbench` or `nix run .#solo12`)

Change the Ethernet interface the master board is plugged to by editing the relevant `config_*.yaml` file, then _re-build the binaries_ 

You can get the ethernet interface by runnning `ip a`

### License and Copyrights

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.
