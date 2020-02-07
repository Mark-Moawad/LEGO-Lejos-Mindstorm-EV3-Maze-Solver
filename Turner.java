package mazeSolve;

import mazeSolve.MazeSolver;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Turner implements Turn {
	protected static final EV3GyroSensor GYRO_SENSOR = new EV3GyroSensor(SensorPort.S3);
	protected static SampleProvider gyro = GYRO_SENSOR.getAngleMode();
	protected static int Angle = GYRO_SENSOR.sampleSize();
	protected static float[] Angle_array = new float[Angle];

	public void rotateLeft() { // rotate Counter clockwise with angle 90
		GYRO_SENSOR.reset();
		gyro.fetchSample(Angle_array, 0);
		double error = Math.abs(90 - Angle_array[0]);
		while (error >= 0) {
			MazeSolver.RIGHT_MOTOR.backward();
			MazeSolver.RIGHT_MOTOR.setSpeed(500);
			MazeSolver.LEFT_MOTOR.forward();
			MazeSolver.LEFT_MOTOR.setSpeed(500);
			gyro.fetchSample(Angle_array, 0);
			error = Math.abs(90 - Angle_array[0]);
			System.out.println("Angle = " + Angle_array[0]);
			System.out.println("error: " + error);
			Delay.msDelay(10);
		}
		MazeSolver.RIGHT_MOTOR.setSpeed(0);
		MazeSolver.LEFT_MOTOR.setSpeed(0);
	}

	public void rotateRight() { // rotate Clockwise with angle 90
		GYRO_SENSOR.reset();
		gyro.fetchSample(Angle_array, 0);
		double error = 90 - Math.abs(Angle_array[0]);
		while (error >= 0) {
			MazeSolver.RIGHT_MOTOR.forward();
			MazeSolver.RIGHT_MOTOR.setSpeed(500);
			MazeSolver.LEFT_MOTOR.backward();
			MazeSolver.LEFT_MOTOR.setSpeed(500);
			gyro.fetchSample(Angle_array, 0);
			error = 90 - Math.abs(Angle_array[0]);
			System.out.println("Angle = " + Angle_array[0]);
			System.out.println("error: " + error);
			Delay.msDelay(10);
		}
		MazeSolver.RIGHT_MOTOR.setSpeed(0);
		MazeSolver.LEFT_MOTOR.setSpeed(0);
	}

	public void flipDirection() {
		GYRO_SENSOR.reset();
		gyro.fetchSample(Angle_array, 0);
		double error = Math.abs(180 - Angle_array[0]);
		while (error >= 0) {
			MazeSolver.RIGHT_MOTOR.backward();
			MazeSolver.RIGHT_MOTOR.setSpeed(500);
			MazeSolver.LEFT_MOTOR.forward();
			MazeSolver.LEFT_MOTOR.setSpeed(500);
			gyro.fetchSample(Angle_array, 0);
			error = Math.abs(180 - Angle_array[0]);
			System.out.println("Angle = " + Angle_array[0]);
			System.out.println("error: " + error);
			Delay.msDelay(10);
		}
		MazeSolver.RIGHT_MOTOR.setSpeed(0);
		MazeSolver.LEFT_MOTOR.setSpeed(0);
	}

}
