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
	
	private double[][] getTransformationMatrix(double a, double alpha, double d, double theta){
		double cosTheta = Math.cos(theta);
		double sinTheta = Math.sin(theta);
		double cosAlpha = Math.cos(alpha);
		double sinAlpha = Math.sin(alpha);
		double[][] transformationMatrix = {
			new double[] {cosTheta, -1*sinTheta*cosAlpha, sinTheta*sinAlpha, a*cosTheta},
			new double[] {sinTheta, cosTheta*cosAlpha, -1*cosTheta*sinAlpha, a*sinTheta},
			new double[] {0, sinAlpha, cosAlpha, d},
			new double[] {0, 0, 0, 1}
		};
		return transformationMatrix;
	}
	
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
		double[] a = new double[] {0.125, 0.225, 0.177, 0.0};
		double[] alpha = new double[] {0.0, 0.0, 0.0, 0.0};
		double[] theta = new double[] {axis[0], axis[1], axis[3], 0.0};
		double[] d = new double[] {0.312, 0.065, -0.177, axis[2]};

		// set transformation matrix with corresponding DH parameter
		double[][] firstTransformation = getTransformationMatrix(a[0], alpha[0], d[0], theta[0]);
		double[][] secondTransformation = getTransformationMatrix(a[1], alpha[1], d[1], theta[1]);
		double[][] thirdTransformation = getTransformationMatrix(a[2], alpha[2], d[2], theta[2]);
		double[][] fourthTransformation = getTransformationMatrix(a[3], alpha[3], d[3], theta[3]);
		// multiply all matrices to get transformation from basis to end effector
		double[][] result = multiplyMatrix(firstTransformation, secondTransformation);
		result = multiplyMatrix(result, thirdTransformation);
		result = multiplyMatrix(result, fourthTransformation);

		return new CartesianPosition(result[0][3],
				result[1][3],
				result[2][3],
				Math.atan2(result[1][0], result[0][0]),
				0.0,
				0.0);
		// formula for angles B and C
		// in this case set to 0
				//Math.atan2(-result[2][0], Math.sqrt(Math.pow(result[2][1], 2) + Math.pow(result[2][2], 2))),
				//Math.atan2(result[2][1], result[2][2]))
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
		
		double firstArmlength = 0.125;
		double secondArmLength = 0.225;

		// vectorMagnitude = sqrt(square(x) + square(y))
		double squaredVector = Math.pow(position.getX(), 2) +
				Math.pow(position.getY(), 2);
		double vectorMagnitude = Math.sqrt(squaredVector);
		
		// compute thetaTwo
		// cosine law: sq(c) = sq(a) + sq(b) - 2*a*b*cos(gamma)
		double cosineThetaTwo = -1 *
				(vectorMagnitude - Math.pow(firstArmlength, 2) - Math.pow(secondArmLength, 2)) / 
				(2 * firstArmlength * secondArmLength);
		// since base position is the stretched arm
		cosineThetaTwo = -1 * (cosineThetaTwo);
		// elbow left
		double firstThetaTwo = Math.PI - Math.acos(cosineThetaTwo);
		// elbow right
		double secondThetaTwo = -1 * firstThetaTwo;
		
		// compute thetaOne
		// cosine law
		double cosineBeta = -1 * (Math.pow(secondArmLength, 2) -
				Math.pow(firstArmlength, 2) -
				squaredVector) /
				(2 * firstArmlength * Math.sqrt(squaredVector));

		double delta = Math.atan2(position.getY(), position.getX());
		double firstThetaOne = Math.acos(cosineBeta) + delta;
		double secondThetaOne = delta - Math.acos(cosineBeta);
		
		// compute thetaThree
		double firstThetaThree = position.getA() - firstThetaOne - firstThetaTwo;
		double secondThetaThree = position.getA() - secondThetaOne - secondThetaTwo;
		
		double dFour = position.getZ() - 0.2;
		
		double[][] ret = {
				new double[] {firstThetaOne, firstThetaTwo, dFour, firstThetaThree},
				new double[] {secondThetaOne, secondThetaTwo, dFour, secondThetaThree}
		};

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
