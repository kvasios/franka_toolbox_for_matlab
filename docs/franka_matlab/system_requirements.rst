System Requirements
===================

Host PC Requirements
--------------------

MATLAB Toolbox Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following Mathworks products are required for the Host PC: 

* `MATLAB <https://www.mathworks.com/products/matlab.html>`_
* `Simulink <https://www.mathworks.com/products/simulink.html>`_
* `Simulink Coder <https://www.mathworks.com/products/simulink-coder.html>`_
* `Matlab Coder <https://www.mathworks.com/products/matlab-coder.html>`_

Some of demos provided with the franka_matlab need the following toolboxes:

* `Matlab Robotics Toolbox <https://www.mathworks.com/products/robotics.html>`_ (required for the MATLAB example pick_and_place_with_RRT.mlx)

MATLAB Coder Support Package for NVIDIA Jetson
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**For working with the Franka AI Companion & NVIDIA Jetson platforms** please download and install the `MATLAB Coder Support Package for NVIDIA Jetson and NVIDIA DRIVE Platforms <https://www.mathworks.com/matlabcentral/fileexchange/68644-matlab-coder-support-package-for-nvidia-jetson-and-nvidia-drive-platforms>`_.

MATLAB Support Package for Linux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In case of Ubuntu Target PC, it is highly recommended to install the `matlab-support package <https://packages.ubuntu.com/search?keywords=matlab-support>`_ 
in order for Matlab to reference the system dynamic libraries instead of the precompiled ones that it ships with:

.. code-block:: shell

    sudo apt install matlab-support

Target PC Requirements
----------------------

The Target PC is the Real-Time Kernel Linux PC responsible for maintaing the real-time 1kHz control loop and it can either be the same as the Host PC or a different one 
in the network like the AI Companion or NVIDIA Jetson.

All the same system requirements for FCI and libfranka apply for the Target PC.

The prebuild binaries for the Target PC are available for the amd64 and arm64 (NVIDIA Jetson) architectures.
