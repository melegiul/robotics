package org.roboticsapi.campus.kr210;

import org.roboticsapi.campus.dashboard.CartesianPosition;
import org.roboticsapi.campus.dashboard.Dashboard;
import org.roboticsapi.campus.dashboard.JointDescription;
import org.roboticsapi.campus.dashboard.MotionType;
import org.roboticsapi.campus.dashboard.RobotInterface;
import org.roboticsapi.campus.robots.KR210Visualization;

public class KR210Main implements RobotInterface {

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
			new JointDescription("°", -185, 185, 10.5, 25),
			new JointDescription("°", -140, -5, 10.1, 25, -90),
			new JointDescription("°", -120, 155, 10.7, 25, 90),
			new JointDescription("°", -350, 350, 13.6, 25),
			new JointDescription("°", -122.5, 122.5, 12.9, 25),
			new JointDescription("°", -350, 350, 20.6, 25) };

	/**
	 * Number of joints of robot
	 */
	final int jointCount = jointDescriptions.length;

	public static void main(String[] args) {
		new Dashboard(jointDescriptions, new KR210Main(),
				new KR210Visualization()).setVisible(true);
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
				axis[4], axis[5]);
	}

	/**
	 * Calculate the inverse kinematics function
	 * 
	 * @deprecated Not necessary for assignment T3
	 * 
	 * @param position
	 *            The given Cartesian position. Values X, Y and Z are in m,
	 *            values A, B and C in degrees
	 * @return Two-dimensional array with the left index specifying the
	 *         solution, and the right index the joint. e.g.
	 *         ret[solution][joint]
	 */
	@Override
	@Deprecated
	public double[][] calculateInverseKinematics(CartesianPosition position) {
		// do NOT add correct implementation, unless you want to
		double[][] ret = new double[8][jointCount];

		for (int i = 0; i < 8; i++)
			for (int j = 0; j < jointCount; j++)
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

		int size = 1000;

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
