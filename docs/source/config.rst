Export your own robot (writing config.json)
===========================================

To export your own robot, first create a directory:

.. code-block:: bash

    mkdir my-robot

Then edit ``my-robot/config.yaml``, here is the minimum example:

.. code-block:: yaml

    document_id: document-id

The ``document_id`` is the number (below XXXXXXXXX) you can find in Onshape URL:

.. code-block:: bash

    https://cad.onshape.com/documents/XXXXXXXXX/w/YYYYYYYY/e/ZZZZZZZZ

Once this is done, if you properly :doc:`installed and setup your API key <installation>`, just run:

.. code-block:: bash

    onshape-urdf-exporter my-robot

``config.yaml`` entries
-----------------------

Here is the full list of possible entries for this configuration.

``document_id``
~~~~~~~~~~~~~~

This is the Onshape ID of the document to be imported. It can be found in the Onshape URL,
just after ``document/``.


``assembly_name``
~~~~~~~~~~~~~~~~

*optional*

This can be used to specify the name of the assembly (in the Onshape document) to be used for robot export. If none
is used, the first assembly found will be used.

``workspace_id``
~~~~~~~~~~~~~~~

*optional, no default*

This argument can be used to use a specific workspace of the document. This can be used for specific branches
ofr your robot without making a version.
The workspace ID can be found in URL, after the ``/w/`` part when selecting a specific version in the tree.

``version_id``
~~~~~~~~~~~~~

*optional, no default*

This argument can be used to use a specific version of the document instead of the last one. The version ID
can be found in URL, after the ``/v/`` part when selecting a specific version in the tree.

If it is not specified, the very last version will be used for import.

``configuration``
~~~~~~~~~~~~~~~~~

*optional, default: "default"*

This is the robot configuration string that will be passed to Onshape. An example of format:

.. code-block:: js

    left_motor_angle=3+radian;enable_yaw=true

``draw_frames``
~~~~~~~~~~~~~~

*optional, default: false*

When :ref:`adding custom frames to your model <custom-frames>`, the part that is used for positioning the frame is
by default excluded from the output description (a dummy link is kept instead). Passing this option to ``true`` will
keep it instead.

``draw_collisions``
~~~~~~~~~~~~~~~~~~

*optional, default: false*

Controls if collision shapes are shown visually.

``joint_max_effort`` and ``joint_max_velocity``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

*optional, default: 1 and 20*

Those parameters can be used to specify the values that will be included in the ``joint`` entries.

Alternatively, they can be dictionaries associating named joints to the values.


``dynamics``
~~~~~~~~~~~~

*optional, default: {}*

This ``dict`` can be used to override the mass and inertia computed by Onshape for a specific part.
See :ref:`example <example-config>` below.


``no_dynamics``
~~~~~~~~~~~~~~

*optional, default: false*

This flag can be set if there is no dynamics. In that case all masses and inertia will be set to 0.

``ignore``
~~~~~~~~~~

*optional, default: []*

This can be a list of parts that you want to be ignored during the export.

Note: the dynamics of the part will not be ignored, but the visual and collision aspect will.

``whitelist``
~~~~~~~~~~~~~

*optional, default: None*

This can be used as the opposed of ``ignore``, to import only some items listed in the configuration
(all items not listed in ``whitelist`` will be ignored if it is not ``None``)

``color``
~~~~~~~~~

*optional, default: None*

Can override the color for parts (should be an array: ``[r, g, b]`` with numbers from 0 to 1)

``package_name``
~~~~~~~~~~~~~~~

*optional*

Prepends a string to the paths of STL files. This is helpful for ROS users as they often need to specify their
``robot_description`` package.

``add_dummy_base_link``
~~~~~~~~~~~~~~~~~~~~~~~

*optional*

Adds a ``base_link`` without inertia as root. This is often necessary for ROS users.

``robot_name``
~~~~~~~~~~~~~~

*optional*

Specifies the robot name.

``additional_urdf_file``
~~~~~~~~~~~~~~~~~~~~~~~~

*optional*

Specifies a file with XML content that is inserted into the URDF at the end of the file. Useful to add things that can't be modelled in Onshape, e.g. simulated sensors.

``use_fixed_links``
~~~~~~~~~~~~~~~~~~~

*optional, default: false*

With this option, visual parts will be added through fixed links to each part of the robot.

``simplify_stls``
~~~~~~~~~~~~~~~~~

*optional, default: "no"*

Can be "no", "visual", "collision" or "all".

If this is set, the complexity of the STL files will be reduced. This can be
good for file size and visualization performance.

``use_collisions_configurations``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

*optional, default: true*

With this option (enabled by default), the collisions=true configuration will be passed when exporting STL
meshes (and NOT dynamics), in order to retrieve simplified mesh parts from Onshape.

This is a way to approximate your robot with simpler meshes.

``post_import_commands``
~~~~~~~~~~~~~~~~~~~~~~~~

*optional, default: []*

This is an array of commands that will be executed after the import is done. It can be used to be sure that
some processing scripts are run everytime you run the tool.

.. _example-config:

Example ``config.yaml`` file
----------------------------

Here is an example of configuration:

.. code-block:: yaml

    # Can be found in the URL when editing the assembly
    document_id: 483c803918afc4d52e2647f0
    # If not specified, the first assembly will be used
    assembly_name: robot
    # The frames parts are kept in the final file
    draw_frames: false
    # Collisions (pure shapes) are also used in the visual section
    draw_collisions: false
    # Masses, com and inertias will be zero (can be used if you import a static
    # field for example)
    no_dynamics: false
    # Should we simplify STLs files?
    simplify_stls: false

    # Those can be used to configure the joint max efforts and velocity, and
    # overriden for specific joints
    joint_max_effort:
      default: 1.5
      head_pitch: 0.5
    joint_max_velocity: 22

    # This can be used to override the dynamics of some part (suppose it's a compound
    # which dynamics is well specified)
    dynamics:
      motorcase:
        mass: 0.5
        com: [0, 0.1, 0]
        inertia: [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]

      # "fixed" can be used to assign a null mass to the object, which makes it fixed (non-dynamics)
      base: fixed

    # Some parts can be totally ignored during import
    ignore:
      - small_screw
      - small_nut
