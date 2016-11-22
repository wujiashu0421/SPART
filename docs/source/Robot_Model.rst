===========
Robot Model
===========

The robot model contains all the required kinematic and dynamic information of the robor that SPARTS requires to compute all the kinematic and dynamic quantities.

The `robot` model is build as a Matlab structure with the following fields:
	* robot.name -- Name of the robot.
	* robot.n_q -- Number of manipulator variables (exclude the base variables).
	* robot.n_links -- Number of manipulator links and joints (include fixed joints).
	* robot.links -- Structure containing the links information.
		* robot.links(i).id -- Link id.
		* robot.links(i).name -- Link name.
		* robot.links(i).parent_joint -- Parent joint id.
		* robot.links(i).T -- Transformation matrix from parent joint.
		* robot.links(i).mass -- Link's mass.
		* robot.links(i).inertia -- Link's inertia matrix.
	* robot.joints -- Structure containing the joints information.
		* robot.joints(i).id -- Joint id.
		* robot.joints(i).name -- Joint name.
		* robot.joints(i).type -- Joint type (0 -- fixed, 1 -- revolute, 2 -- prismatic)
		* robot.joints(i).q_id -- Manipulator variable id.
		* robot.joints(i).parent_link -- Parent link id.
		* robot.joints(i).child_link -- Child link id.
		* robot.joints(i).axis -- Joint axis.
		* robot.links(i).T -- Transformation matrix from parent Link.
	* robot.base_link -- Structure containing the base link (root) information.
		* robot.base_link.name -- Link name.
		* robot.base_links.T -- Transformation matrix from parent joint.
		* robot.base_links.mass -- Link's mass.
		* robot.base_links.inertia -- Link's inertia matrix.
	* robot.Con -- Structure containing extra connectivity information.
		* robot.Con.Branch -- Branch connectivity map.
		* robot.Con.Child -- Child connectivity of links.
		* robot.Con.Child_base -- Child connectivity of base links.
	* robot.origin -- Description of how the robot model was created (i.e. urdf, DH or )

	


