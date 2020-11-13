package org.roboticsapi.campus.kr5scara;

import org.roboticsapi.campus.dashboard.CartesianPosition;
import org.roboticsapi.campus.dashboard.Dashboard;
import org.roboticsapi.campus.dashboard.JointDescription;
import org.roboticsapi.campus.dashboard.MotionType;
import org.roboticsapi.campus.dashboard.RobotInterface;
import org.roboticsapi.campus.robots.KR5ScaraVisualization;

public class KR5ScaraMain implements RobotInterface {

	/**
	 * Static information about joints
	 * 
	 * JointDescription contains the following items for each axis:
	 * 
	 * 1. unit 2. minimum angle, 3. maximum angle, 4. maximum velocity, 5.
	 * maximum acceleration
	 * 
	 * Velocity is in unit/s, acceleration in unit/(s*s)
	 */
	static final JointDescription[] jointDescriptions = {
			new JointDescription("°", -155, 155, 52.5, 25),
			new JointDescription("°", -145, 145, 52.5, 25),
			new JointDescription("m", -0.2, 0, 0.2, 0.02),
			new JointDescription("°", -360, 360, 240.0, 25) };

	/**
	 * Number of joints of robot
	 */
	final int jointCount = jointDescriptions.length;

	public static void main(String[] args) {
		new Dashboard(jointDescriptions, new KR5ScaraMain(),
				new KR5ScaraVisualization()).setVisible(true);
	}

	/**
	 * Calculate the direct kinematics function
	 * 
	 * @param axis
	 *            Array with angles of all axes in degree
	 * @return {@link CartesianPosition} object with the Cartesian position for
	 *         the given axis configuration. Values X, Y and Z are in m, values
	 *         A, B and C in degrees
	 */
	@Override
	public CartesianPosition calculateDirectKinematics(double[] axis) {
		// TODO add correct implementation
		return new CartesianPosition(axis[0], axis[1], axis[2], axis[3],
				axis[0], axis[1]);
	}

	/**
	 * Calculate the inverse kinematics function
	 * 
	 * @param position
	 *            The given Cartesian position. Values X, Y and Z are in m,
	 *            values A, B and C in degrees
	 * @return Two-dimensional array with the left index specifying the
	 *         solution, and the right index the joint. e.g.
	 *         ret[solution][joint]
	 */
	@Override
	public double[][] calculateInverseKinematics(CartesianPosition position) {
		// TODO add correct implementation
		double[][] ret = new double[2][jointCount];

		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 4; j++)
				ret[i][j] = i + j;

		return ret;
	}

	/**
	 * Calculate all interpolation position for point-to-point motion
	 * 
	 * @param start
	 *            Joint angles for start position (in degrees)
	 * @param destination
	 *            Joint angles for destination position (in degrees)
	 * @param type
	 *            Type of the motion (asynchronous, synchronous or fully
	 *            synchronous)
	 * @return Two-dimensional array with all intermediate positions for all
	 *         joints (in degrees). The left index specifies the joint, the
	 *         right the step. e.g. traj[joint][step]
	 */
	@Override
	public double[][] calculatePTP(double[] start, double[] destination,
			MotionType type) {
		// TODO add correct implementation

		// Number of steps for demonstration - calculate proper value
		int size = 100;

		double[][] traj = new double[jointCount][size];
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < jointCount; j++) {
				traj[j][i] = start[j] + (destination[j] - start[j]) * i
						/ (size - 1);
			}
		}

		return traj;
	}

}
