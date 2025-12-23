Installation
============

Toolbox Add-On Installation Methods
-----------------------------------

You can directly download and install the **latest pre-built** ``franka.mtlbx`` from the
`github release page <https://github.com/frankarobotics/franka_toolbox_for_matlab/releases/latest>`_,
which includes all the dependencies and should be able to execute out-of-the-box without additional
installations.

Option 1: Drag and drop the .mltbx file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Open your MATLAB workspace to the directory where you downloaded the ``franka.mtlbx`` file and drag and drop it into your MATLAB Command Window.

Option 2: Programmatically
^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: matlab

    uiopen('<path to your franka.mltbx file>', 1);

Install
---------

After adding the Toolbox Add-On to MATLAB, you can install it by executing the following command:

.. code-block:: matlab

    franka_toolbox_install();


Custom toolbox build 
--------------------

In case of any potential issues with the Toolbox prebuilt libfranka and the bundled 3d party dependencies, you can always manually build and install libfranka from source for your system.

For more details, please refer to the :ref:`custom_build` section.

Handling of system-wide libfranka installation
----------------------------------------------

In case a system-wide libfranka installation is preferred, the Toolbox must be forced to build against it by executing:

.. code-block:: matlab

    franka_libfranka_system_installation_set(true);

Switching back to the local libfranka installation can be achieved by executing:

.. code-block:: matlab

    franka_libfranka_system_installation_set(false);

This is a per-user setting and it will be stored in the MATLAB preferences.

Uninstall
---------

1. Clean-up local permanent installation artifacts:

.. code-block:: matlab

    franka_toolbox_uninstall();

2. Remove the toolbox using MATLAB Add-Ons Manager.

.. figure:: _static/franka_toolbox_uninstall.png
    :align: center
    :figclass: align-center
    :scale: 60%

    Uninstalling the Franka Toolbox.

.. _libfranka_handling_options: