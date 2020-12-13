package org.roboticsapi.campus.kr210;

import org.roboticsapi.campus.dashboard.CartesianPosition;
import org.roboticsapi.campus.dashboard.Dashboard;
import org.roboticsapi.campus.dashboard.JointDescription;
import org.roboticsapi.campus.dashboard.MotionType;
import org.roboticsapi.campus.dashboard.RobotInterface;
import org.roboticsapi.campus.robots.KR210Visualization;

import org.apache.commons.math3.util.Precision;

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
	 * Sets transformation matrix according to parameters
	 * @param a arm length
	 * @param alpha rotation along x-axis
	 * @param d shift along z-axis
	 * @param theta rotation along z-axis
	 * @return
	 */
	private double[][] getTransformationMatrix(double a, double alpha, double d, double theta){
 		double cosTheta = Precision.round(Math.cos(Math.toRadians(theta)), 15);
		double sinTheta = Precision.round(Math.sin(Math.toRadians(theta)), 15);
		double cosAlpha = Precision.round(Math.cos(Math.toRadians(alpha)), 15);
		double sinAlpha = Precision.round(Math.sin(Math.toRadians(alpha)), 15);
		double[][] transformationMatrix = {
			new double[] {cosTheta, -1*sinTheta*cosAlpha, sinTheta*sinAlpha, a*cosTheta},
			new double[] {sinTheta, cosTheta*cosAlpha, -1*cosTheta*sinAlpha, a*sinTheta},
			new double[] {0, sinAlpha, cosAlpha, d},
			new double[] {0, 0, 0, 1}
		};
		return transformationMatrix;
	}
	
	/**
	 * Multiplies matrixA with matrixB and returns result as new matrix
	 * @param matrixA first factor of multiplication
	 * @param matrixB second factor of multiplication
	 * @return result as matrix
	 */
	private double[][] multiplyMatrix(double[][] matrixA, double[][] matrixB){
		double[][] result = new double[matrixA.length][matrixB[0].length];
		for(int row=0; row<matrixA.length; row++) {
			for(int col=0; col<matrixA[row].length; col++) {
				double cell = 0;
				for(int k=0; k<matrixB.length; k++) {
					cell += matrixA[row][k] * matrixB[k][col];
				}
				result[row][col] = cell;
			}
		}
		return result;
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
	 // determine DH parameter according to scara350 data sheet
	 		double[] a = new double[] {0.35, 1.35, 0.041, 0.0, 0.0, 0.0};
	 		double[] alpha = new double[] {90.0, 0.0, -90.0, 90.0, -90.0, 0.0};
	 		double[] theta = new double[] {axis[0], axis[1], axis[2]+90.0, axis[3], axis[4], axis[5]};
	 		double[] d = new double[] {-0.675, 0.0, 0.0, -1.4, 0.0, -0.24};

	 		// set transformation matrix with corresponding DH parameter
	 		double[][] firstTransformation = getTransformationMatrix(a[0], alpha[0], d[0], theta[0]);
	 		double[][] secondTransformation = getTransformationMatrix(a[1], alpha[1], d[1], theta[1]);
	 		double[][] thirdTransformation = getTransformationMatrix(a[2], alpha[2], d[2], theta[2]);
	 		double[][] fourthTransformation = getTransformationMatrix(a[3], alpha[3], d[3], theta[3]);
	 		double[][] fifthTransformation = getTransformationMatrix(a[4], alpha[4], d[4], theta[4]);
	 		double[][] sixthTransformation = getTransformationMatrix(a[5], alpha[5], d[5], theta[5]);
	 		// multiply all matrices to get transformation from basis to end effector
	 		double[][] result = multiplyMatrix(firstTransformation, secondTransformation);
	 		result = multiplyMatrix(result, thirdTransformation);
	 		result = multiplyMatrix(result, fourthTransformation);
	 		result = multiplyMatrix(result, fifthTransformation);
	 		result = multiplyMatrix(result, sixthTransformation);

	 		double angleA = Math.toDegrees(Math.atan2(result[1][0], result[0][0]));
	 		return new CartesianPosition(result[0][3],
	 				-result[1][3],
	 				-result[2][3],
	 				-angleA,
	 				-Math.toDegrees(Math.atan2(-result[2][0], Math.sqrt(Math.pow(result[2][1], 2) + Math.pow(result[2][2], 2)))),
	 				Math.toDegrees(Math.atan2(result[2][1], result[2][2])));
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
