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
			new JointDescription("�", -185, 185, 10.5, 25),
			new JointDescription("�", -140, -5, 10.1, 25, -90),
			new JointDescription("�", -120, 155, 10.7, 25, 90),
			new JointDescription("�", -350, 350, 13.6, 25),
			new JointDescription("�", -122.5, 122.5, 12.9, 25),
			new JointDescription("�", -350, 350, 20.6, 25) };

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
        	// distance for each axis
        	double[] distance = new double[jointCount];
        	// time stamps when max velocity is reached after period of max acceleration
        	double[] timeToMaxVelocity = new double[jointCount];
        	for(int i=0; i<jointCount; i++) {
        		distance[i] = Math.abs(destination[i] - start[i]);
        		timeToMaxVelocity[i] = jointDescriptions[i].maximumVelocity /
        				jointDescriptions[i].maximumAcceleration;
        	}
        	double[] duration = new double[jointCount];
        	double[] maxVelocity = new double[jointCount];
        	double[] timeToDeceleration = new double[jointCount];
        	double maxDuration = 0;
        	int maxDurationAxis = -1;
        	for(int i=0; i<jointCount; i++) {
        		// according to formula t_end = s/v + v/a
        		// derived from: t_acc = v/a and s = v * (t_end - t_acc)
        		duration[i] = (distance[i] / jointDescriptions[i].maximumVelocity) +
        				(jointDescriptions[i].maximumVelocity / jointDescriptions[i].maximumAcceleration);
        		if (duration[i] < timeToMaxVelocity[i] * 2) {
        			// if the distance is to short to reach max velocity, update max velocity
        			// time to max velocity * 2 = time to accelerate + decelerate from/to max velocity
        			maxVelocity[i] = Math.sqrt(jointDescriptions[i].maximumAcceleration * distance[i]);
        			// set duration according to new max velocity
        			duration[i] = (distance[i] / maxVelocity[i]) +
        					(maxVelocity[i] / jointDescriptions[i].maximumAcceleration);
        			// also adjust time to reach new max velocity
        			timeToMaxVelocity[i] = maxVelocity[i] / jointDescriptions[i].maximumAcceleration;
        		} else {
        			// keep default max velocity
        			maxVelocity[i] = jointDescriptions[i].maximumVelocity;
        		}
        		timeToDeceleration[i] = duration[i] - timeToMaxVelocity[i];
        		if (maxDuration < duration[i]) {
        			// determine axis with longest operation time
        			maxDuration = duration[i];
        			maxDurationAxis = i;
        		}
        	}
        	if (type == MotionType.Synchronous) {
        		// synchronous mode: reduce max velocity of all non-main axis
        		// so that operation time ends for all axis at the same time
        		for(int i=0; i<jointCount; i++) {
        			if (i != maxDurationAxis) {
        				// if i is not main axis, max velocity must be reduced
        				duration[i] = maxDuration;
        				// formula for calculating new max velocity
        				// derived from: t = s/v + v/a
        				// => v = (a*t)/2 - sqrt( (sq(a)*sq(t))/4 - s*a)
        				maxVelocity[i] = (jointDescriptions[i].maximumAcceleration * duration[i] / 2) -
        						Math.sqrt((Math.pow(jointDescriptions[i].maximumAcceleration, 2) * Math.pow(duration[i], 2) / 4) -
        								(distance[i] * jointDescriptions[i].maximumAcceleration));
        				// adjust time to reach new max velocity
        				timeToMaxVelocity[i] = maxVelocity[i] / jointDescriptions[i].maximumAcceleration;
        				if (duration[i] < timeToMaxVelocity[i] * 2) {
        					// if distance is too short, cut velocity further
        					maxVelocity[i] = Math.sqrt(jointDescriptions[i].maximumAcceleration * distance[i]);
        					// set duration according to new max velocity
        					duration[i] = (distance[i] / maxVelocity[i]) +
        							(maxVelocity[i] / jointDescriptions[i].maximumAcceleration);
        					// also adjust time to reach new max velocity
        					timeToMaxVelocity[i] = maxVelocity[i] / jointDescriptions[i].maximumAcceleration;
        				}
        				timeToDeceleration[i] = duration[i] - timeToMaxVelocity[i];
        			}
        		}
        	}
        
        	// Number of step during operation
        	int size = (int) (maxDuration / 0.01);
        
        	double[][] traj = new double[jointCount][size];
        	for (int i = 0; i < size; i++) {
        		for (int j = 0; j < jointCount; j++) {
        			// set movement direction of current joint
        			int sign = -1;
        			if (destination[j] - start[j] > 0) {
        				sign = 1;
        			}
        			if (start[j] != destination[j] && (i+1) * 0.01 < duration[j]) {
        				// formulas are only valid if distance is not 0
        				// and if joint has not reached destination
        				if ((i+1) * 0.01 < timeToMaxVelocity[j]) {
        					// compute distance during acceleration
        					// according to: s(t) = 1/2 * a * t * t + s(0)
        					traj[j][i] = start[j] + sign * jointDescriptions[j].maximumAcceleration
        							* Math.pow((i+1) * 0.01, 2) / 2;
        				} else if ((i+1) * 0.01 < timeToDeceleration[j]) {
        					// compute distance during constant velocity
        					// according to: s(t) = v * t - (1/2 * v * v / a)
        					traj[j][i] = start[j] + sign * (maxVelocity[j] * (i+1) * 0.01
        							- (Math.pow(maxVelocity[j], 2) / jointDescriptions[j].maximumAcceleration / 2));
        				} else {
        					// compute distance during deceleration
        					// according to: s(t) = v * (t_end - t_acc) - 1/2 * a * pow(t_end - t)
        					double constant = maxVelocity[j] * (duration[j] - timeToMaxVelocity[j]);
        					traj[j][i] = start[j] + sign * (constant - jointDescriptions[j].maximumAcceleration
        							* Math.pow(duration[j] - (i+1) * 0.01, 2) / 2);
        				}
        			} else {
        				traj[j][i] = destination[j];
        			}
        		}
        	}
        	return traj;
	}

}
