=====================
Using URDF with SPART
=====================

SPART can also use the popular Unified Robot Description Format (URDF) file. More information about how to create this type of descriptions can be found in the `ROS wiki`_. Particularly clarifying are the `XML field descriptions`_.

.. _ROS wiki: http://wiki.ros.org/urdf
.. _XML field descriptions: http://wiki.ros.org/urdf/XML

SPART includes several URDF models that can be found in the ``Utilities/URDF_Models`` folder. A list of this models is also found in :ref:`URDF-Models`.

Once you have your URDF description it has to be converted to the robot model. This can be easily done as follows.

.. code-block:: matlab

	%URDF filename
	filename='kuka_lwr/kuka.urdf';

	%Create robot model
	[robot,robot_keys] = urdf2robot(filename);


Virtual World in Simulink Using URDF
====================================

One advantage of using URDF is that it can be imported into a Simulink virtual world (since Matlab 2016b). To do so follow the `MathWorks documentation`_.

.. _Mathworks documentation: https://www.mathworks.com/help/sl3d/import-visual-representations-of-robot-models.html