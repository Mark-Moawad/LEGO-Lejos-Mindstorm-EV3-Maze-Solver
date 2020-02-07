package mazeSolve;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class MazeSolver {
	protected static final EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.C);
	protected static final EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
	private static final EV3UltrasonicSensor distanceSensor = new EV3UltrasonicSensor(SensorPort.S4);
	private static SampleProvider distance = distanceSensor.getDistanceMode();
	private static int Distance = distanceSensor.sampleSize();
	private static float[] Distance_array = new float[Distance];
	private static final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	private static SensorMode color = colorSensor.getColorIDMode();
	private static int colorSize = colorSensor.sampleSize();
	private static float[] color_Array = new float[colorSize];
	private static double leftDistance = 0;
	private static double frontDistance = 0;
	private static double rightDistance = 0;
	private static String currentColor = "Red";
	private static boolean menuShow = true;
	private static int colorIndex = -2;
	private final static int XLCD = 0;
	private final static int YLCD = 0;
	public static Turner turner = new Turner();

	public static void main(String[] args) {
		int distance_Count = 0;
		double distanceToMove = 0;
		LCD.drawString("Please press any", XLCD, YLCD);
		LCD.drawString("button to display", XLCD, YLCD + 1);
		LCD.drawString("color options", XLCD, YLCD + 2);
		Button.waitForAnyPress();
		menu();
		beep();
		LCD.clear();
		int x = colorSensing();
		int y = -1;
		int z = -1;
		while (x != colorIndex && y != colorIndex && z != colorIndex) {
			distance.fetchSample(Distance_array, 0);
			frontDistance = Distance_array[0];
			Delay.msDelay(10);
			turner.rotateRight();
			Delay.msDelay(10);
			distance.fetchSample(Distance_array, 0);
			rightDistance = Distance_array[0];
			Delay.msDelay(10);
			turner.flipDirection();
			Delay.msDelay(10);
			distance.fetchSample(Distance_array, 0);
			leftDistance = Distance_array[0];
			Delay.msDelay(10);
			while (leftDistance > 0.11 && leftDistance < 0.17) {
				RIGHT_MOTOR.backward();
				RIGHT_MOTOR.setSpeed(300);
				LEFT_MOTOR.backward();
				LEFT_MOTOR.setSpeed(300);
				distance.fetchSample(Distance_array, 0);
				leftDistance = Distance_array[0];
			}
			LCD.clear();
			LCD.drawString("Searching for:", XLCD, YLCD);
			LCD.drawString("" + currentColor, XLCD, YLCD + 1);
			turner.rotateRight();
			Delay.msDelay(10);
			Turner.GYRO_SENSOR.reset();
			LCD.clear();
			LCD.drawString("Front Distance" + frontDistance, XLCD, YLCD);
			LCD.drawString("Right Distance" + rightDistance, XLCD, YLCD + 1);
			LCD.drawString("Left Distance" + leftDistance, XLCD, YLCD + 2);
			Delay.msDelay(500);
			while (leftDistance < 0.2) {
				if (frontDistance > 0.2 && rightDistance < 0.2) {
					distanceToMove = frontDistance;
					LCD.clear();
					LCD.drawString("Case 1", XLCD, YLCD);
					Delay.msDelay(500);
					while (frontDistance > distanceToMove - 0.375) {
						LCD.clear();
						LCD.drawString("Searching for:", XLCD, YLCD);
						LCD.drawString("" + currentColor, XLCD, YLCD + 1);
						Delay.msDelay(10);
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(500);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(500);
						distance.fetchSample(Distance_array, 0);
						frontDistance = Distance_array[0];
						Delay.msDelay(10);
					}
					distanceToMove = 0;
				} else if (frontDistance < 0.2 && rightDistance > 0.2) {
					LCD.clear();
					LCD.drawString("Case 2", XLCD, YLCD);
					Delay.msDelay(500);
					turner.rotateRight();
					Delay.msDelay(10);
					Turner.GYRO_SENSOR.reset();
					distance.fetchSample(Distance_array, 0);
					frontDistance = Distance_array[0];
					distanceToMove = frontDistance;
					while (frontDistance > distanceToMove - 0.375) {
						LCD.clear();
						LCD.drawString("Searching for:", XLCD, YLCD);
						LCD.drawString("" + currentColor, XLCD, YLCD + 1);
						Delay.msDelay(10);
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(500);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(500);
						distance.fetchSample(Distance_array, 0);
						frontDistance = Distance_array[0];
						Delay.msDelay(10);
					}
					distanceToMove = 0;
				} else if (frontDistance > 0.2 && rightDistance > 0.2) {
					distanceToMove = frontDistance;
					LCD.clear();
					LCD.drawString("Case 3", XLCD, YLCD);
					Delay.msDelay(500);
					while (frontDistance > distanceToMove - 0.375) {
						LCD.clear();
						LCD.drawString("Searching for:", XLCD, YLCD);
						LCD.drawString("" + currentColor, XLCD, YLCD + 1);
						Delay.msDelay(10);
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(500);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(500);
						distance.fetchSample(Distance_array, 0);
						frontDistance = Distance_array[0];
						Delay.msDelay(10);
					}
					distanceToMove = 0;
				} else if (frontDistance < 0.2 && rightDistance < 0.2) {
					LCD.clear();
					LCD.drawString("Color Detection", XLCD, YLCD);
					LCD.drawString("Case", XLCD, YLCD + 1);
					Delay.msDelay(500);
					while (frontDistance > 0.05) {
						LCD.clear();
						LCD.drawString("Detecting now:", XLCD, YLCD);
						LCD.drawString("" + currentColor, XLCD, YLCD + 1);
						Delay.msDelay(10);
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(300);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(300);
						distance_Count++;
						distance.fetchSample(Distance_array, 0);
						frontDistance = Distance_array[0];
					}
					x = colorSensing();
					if (x == colorIndex) {
						break;
					}
					for (int i = distance_Count; i > 0; i--) {
						RIGHT_MOTOR.forward();
						RIGHT_MOTOR.setSpeed(300);
						LEFT_MOTOR.forward();
						LEFT_MOTOR.setSpeed(300);
						Delay.msDelay(10);
					}
					distance_Count = 0;
					turner.rotateLeft();
					Delay.msDelay(10);
					Turner.GYRO_SENSOR.reset();
					while (leftDistance > 0.05) {
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(300);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(300);
						distance_Count++;
						distance.fetchSample(Distance_array, 0);
						leftDistance = Distance_array[0];
					}
					y = colorSensing();
					if (y == colorIndex) {
						break;
					}
					for (int j = (int) (distance_Count * 1.5); j > 0; j--) {
						RIGHT_MOTOR.forward();
						RIGHT_MOTOR.setSpeed(300);
						LEFT_MOTOR.forward();
						LEFT_MOTOR.setSpeed(300);
					}
					distance_Count = 0;
					turner.flipDirection();
					Delay.msDelay(10);
					Turner.GYRO_SENSOR.reset();
					while (rightDistance > 0.05) {
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(300);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(300);
						distance_Count++;
						distance.fetchSample(Distance_array, 0);
						rightDistance = Distance_array[0];
					}
					z = colorSensing();
					if (z == colorIndex) {
						break;
					}
					for (int k = distance_Count / 2; k > 0; k--) {
						RIGHT_MOTOR.forward();
						RIGHT_MOTOR.setSpeed(300);
						LEFT_MOTOR.forward();
						LEFT_MOTOR.setSpeed(300);
					}
					distance_Count = 0;
					turner.rotateRight();
					Delay.msDelay(10);
					Turner.GYRO_SENSOR.reset();
				}
				break;
			}

			while (leftDistance > 0.2) {
				if (frontDistance < 0.2 && rightDistance < 0.2) {
					turner.rotateLeft();
					distance.fetchSample(Distance_array, 0);
					frontDistance = Distance_array[0];
					Turner.GYRO_SENSOR.reset();
					distanceToMove = frontDistance;
					LCD.clear();
					LCD.drawString("Case 4", XLCD, YLCD);
					Delay.msDelay(500);
					while (frontDistance > distanceToMove - 0.375) {
						LCD.clear();
						LCD.drawString("Searching for:", XLCD, YLCD);
						LCD.drawString("" + currentColor, XLCD, YLCD + 1);
						Delay.msDelay(10);
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(500);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(500);
						distance.fetchSample(Distance_array, 0);
						frontDistance = Distance_array[0];
					}
					distanceToMove = 0;
				} else if (frontDistance > 0.2 && rightDistance > 0.2) {
					turner.rotateLeft();
					distance.fetchSample(Distance_array, 0);
					frontDistance = Distance_array[0];
					Turner.GYRO_SENSOR.reset();
					distanceToMove = frontDistance;
					LCD.clear();
					LCD.drawString("Case 5", XLCD, YLCD);
					Delay.msDelay(500);
					while (frontDistance > distanceToMove - 0.375) {
						LCD.clear();
						LCD.drawString("Searching for:", XLCD, YLCD);
						LCD.drawString("" + currentColor, XLCD, YLCD + 1);
						Delay.msDelay(10);
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(500);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(500);
						distance.fetchSample(Distance_array, 0);
						frontDistance = Distance_array[0];
					}
					distanceToMove = 0;
				} else if (frontDistance < 0.2 && rightDistance > 0.2) {
					turner.rotateLeft();
					Delay.msDelay(10);
					Turner.GYRO_SENSOR.reset();
					distance.fetchSample(Distance_array, 0);
					frontDistance = Distance_array[0];
					distanceToMove = frontDistance;
					LCD.clear();
					LCD.drawString("Case 6", XLCD, YLCD);
					Delay.msDelay(500);
					while (frontDistance > distanceToMove - 0.375) {
						LCD.clear();
						LCD.drawString("Searching for:", XLCD, YLCD);
						LCD.drawString("" + currentColor, XLCD, YLCD + 1);
						Delay.msDelay(10);
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(500);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(500);
						distance.fetchSample(Distance_array, 0);
						frontDistance = Distance_array[0];
					}
					distanceToMove = 0;
				} else if (frontDistance > 0.2 && rightDistance < 0.2) {
					LCD.clear();
					LCD.drawString("Case 7", XLCD, YLCD);
					Delay.msDelay(500);
					turner.rotateLeft();
					Delay.msDelay(10);
					Turner.GYRO_SENSOR.reset();
					distance.fetchSample(Distance_array, 0);
					frontDistance = Distance_array[0];
					distanceToMove = frontDistance;
					while (frontDistance > distanceToMove - 0.375) {
						LCD.clear();
						LCD.drawString("Searching for:", XLCD, YLCD);
						LCD.drawString("" + currentColor, XLCD, YLCD + 1);
						Delay.msDelay(10);
						RIGHT_MOTOR.backward();
						RIGHT_MOTOR.setSpeed(500);
						LEFT_MOTOR.backward();
						LEFT_MOTOR.setSpeed(500);
						distance.fetchSample(Distance_array, 0);
						frontDistance = Distance_array[0];
					}
					distanceToMove = 0;
				}
				break;
			}
		}
		RIGHT_MOTOR.setSpeed(0);
		LEFT_MOTOR.setSpeed(0);
		RIGHT_MOTOR.close();
		LEFT_MOTOR.close();
		distanceSensor.close();
		colorSensor.close();
		Turner.GYRO_SENSOR.close();
		beep();
		LCD.clear();
		LCD.drawString("Done with", XLCD, YLCD);
		LCD.drawString("PRIDE", XLCD, YLCD + 1);
		Delay.msDelay(3000);
	}

	private static int colorSensing() {
		color.fetchSample(color_Array, 0);
		int col_Index = (int) color_Array[0];
		switch (col_Index) {
		case Color.BLUE:
			col_Index = 2;
			break;
		case Color.GREEN:
			col_Index = 1;
			break;
		case Color.YELLOW:
			col_Index = 3;
			break;
		case Color.RED:
			col_Index = 0;
			break;
		case Color.CYAN:
			col_Index = 12;
			break;
		case Color.GRAY:
			col_Index = 9;
			break;
		case Color.MAGENTA:
			col_Index = 4;
			break;
		case Color.PINK:
			col_Index = 8;
			break;
		case Color.ORANGE:
			col_Index = 5;
			break;
		default:
			col_Index = -1;
		}
		return col_Index;
	}

	private static int menu() {
		LCD.clear();
		while (menuShow == true) {
			while (currentColor == "Red") {
				LCD.clear();
				LCD.drawString(">Red", 0, 0);
				LCD.drawString("Blue", 0, 1);
				LCD.drawString("Green", 0, 2);
				LCD.drawString("Yellow", 0, 3);
				LCD.drawString("Orange", 0, 4);
				LCD.drawString("Gray", 0, 5);
				LCD.drawString("Magenta", 0, 6);
				LCD.drawString("Pink", 10, 0);
				LCD.drawString("Cyan", 10, 1);
				Button.waitForAnyPress();
				if (Button.UP.isDown()) {
					Button.discardEvents();
				} else if (Button.DOWN.isDown()) {
					currentColor = "Blue";
				} else if (Button.LEFT.isDown()) {
					currentColor = "Red";
					Button.discardEvents();
				} else if (Button.RIGHT.isDown()) {
					currentColor = "Pink";
				} else if (Button.ENTER.isDown()) {
					menuShow = false;
					colorIndex = 0;
					break;
				} else {
					Button.discardEvents();
				}
			}
			while (currentColor == "Blue") {
				LCD.clear();
				LCD.drawString("Red", 0, 0);
				LCD.drawString(">Blue", 0, 1);
				LCD.drawString("Green", 0, 2);
				LCD.drawString("Yellow", 0, 3);
				LCD.drawString("Orange", 0, 4);
				LCD.drawString("Gray", 0, 5);
				LCD.drawString("Magenta", 0, 6);
				LCD.drawString("Pink", 10, 0);
				LCD.drawString("Cyan", 10, 1);
				Button.waitForAnyPress();
				if (Button.UP.isDown()) {
					currentColor = "Red";
				} else if (Button.DOWN.isDown()) {
					currentColor = "Green";
				} else if (Button.LEFT.isDown()) {
					Button.discardEvents();
				} else if (Button.RIGHT.isDown()) {
					currentColor = "Cyan";
				} else if (Button.ENTER.isDown()) {
					menuShow = false;
					colorIndex = 2;
					break;
				} else {
					Button.discardEvents();
				}
			}
			while (currentColor == "Green") {
				LCD.clear();
				LCD.drawString("Red", 0, 0);
				LCD.drawString("Blue", 0, 1);
				LCD.drawString(">Green", 0, 2);
				LCD.drawString("Yellow", 0, 3);
				LCD.drawString("Orange", 0, 4);
				LCD.drawString("Gray", 0, 5);
				LCD.drawString("Magenta", 0, 6);
				LCD.drawString("Pink", 10, 0);
				LCD.drawString("Cyan", 10, 1);
				Button.waitForAnyPress();
				if (Button.UP.isDown()) {
					currentColor = "Blue";
				} else if (Button.DOWN.isDown()) {
					currentColor = "Yellow";
				} else if (Button.LEFT.isDown()) {
					Button.discardEvents();
				} else if (Button.RIGHT.isDown()) {
					currentColor = "Green";
					Button.discardEvents();
				} else if (Button.ENTER.isDown()) {
					menuShow = false;
					colorIndex = 1;
					break;
				} else {
					Button.discardEvents();
				}
			}
			while (currentColor == "Yellow") {
				LCD.clear();
				LCD.drawString("Red", 0, 0);
				LCD.drawString("Blue", 0, 1);
				LCD.drawString("Green", 0, 2);
				LCD.drawString(">Yellow", 0, 3);
				LCD.drawString("Orange", 0, 4);
				LCD.drawString("Gray", 0, 5);
				LCD.drawString("Magenta", 0, 6);
				LCD.drawString("Pink", 10, 0);
				LCD.drawString("Cyan", 10, 1);
				Button.waitForAnyPress();
				if (Button.UP.isDown()) {
					currentColor = "Green";
				} else if (Button.DOWN.isDown()) {
					currentColor = "Orange";
				} else if (Button.LEFT.isDown()) {
					Button.discardEvents();
				} else if (Button.RIGHT.isDown()) {
					Button.discardEvents();
				} else if (Button.ENTER.isDown()) {
					menuShow = false;
					colorIndex = 3;
					break;
				} else {
					Button.discardEvents();
				}
			}
			while (currentColor == "Orange") {
				LCD.clear();
				LCD.drawString("Red", 0, 0);
				LCD.drawString("Blue", 0, 1);
				LCD.drawString("Green", 0, 2);
				LCD.drawString("Yellow", 0, 3);
				LCD.drawString(">Orange", 0, 4);
				LCD.drawString("Gray", 0, 5);
				LCD.drawString("Magenta", 0, 6);
				LCD.drawString("Pink", 10, 0);
				LCD.drawString("Cyan", 10, 1);
				Button.waitForAnyPress();
				if (Button.UP.isDown()) {
					currentColor = "Yellow";
				} else if (Button.DOWN.isDown()) {
					currentColor = "Gray";
				} else if (Button.LEFT.isDown()) {
					Button.discardEvents();
				} else if (Button.RIGHT.isDown()) {
					Button.discardEvents();
				} else if (Button.ENTER.isDown()) {
					menuShow = false;
					colorIndex = 5;
					break;
				} else {
					Button.discardEvents();
				}
			}
			while (currentColor == "Gray") {
				LCD.clear();
				LCD.drawString("Red", 0, 0);
				LCD.drawString("Blue", 0, 1);
				LCD.drawString("Green", 0, 2);
				LCD.drawString("Yellow", 0, 3);
				LCD.drawString("Orange", 0, 4);
				LCD.drawString(">Gray", 0, 5);
				LCD.drawString("Magenta", 0, 6);
				LCD.drawString("Pink", 10, 0);
				LCD.drawString("Cyan", 10, 1);
				Button.waitForAnyPress();
				if (Button.UP.isDown()) {
					currentColor = "Orange";
				} else if (Button.DOWN.isDown()) {
					currentColor = "Magenta";
				} else if (Button.LEFT.isDown()) {
					Button.discardEvents();
				} else if (Button.RIGHT.isDown()) {
					Button.discardEvents();
				} else if (Button.ENTER.isDown()) {
					menuShow = false;
					colorIndex = 9;
					break;
				} else {
					Button.discardEvents();
				}
			}
			while (currentColor == "Magenta") {
				LCD.clear();
				LCD.drawString("Red", 0, 0);
				LCD.drawString("Blue", 0, 1);
				LCD.drawString("Green", 0, 2);
				LCD.drawString("Yellow", 0, 3);
				LCD.drawString("Orange", 0, 4);
				LCD.drawString("Gray", 0, 5);
				LCD.drawString(">Magenta", 0, 6);
				LCD.drawString("Pink", 10, 0);
				LCD.drawString("Cyan", 10, 1);
				Button.waitForAnyPress();
				if (Button.UP.isDown()) {
					currentColor = "Gray";
				} else if (Button.DOWN.isDown()) {
					Button.discardEvents();
				} else if (Button.LEFT.isDown()) {
					currentColor = "Magenta";
					Button.discardEvents();
				} else if (Button.RIGHT.isDown()) {
					Button.discardEvents();
				} else if (Button.ENTER.isDown()) {
					menuShow = false;
					colorIndex = 4;
					break;
				} else {
					Button.discardEvents();
				}
			}
			while (currentColor == "Pink") {
				LCD.clear();
				LCD.drawString("Red", 0, 0);
				LCD.drawString("Blue", 0, 1);
				LCD.drawString("Green", 0, 2);
				LCD.drawString("Yellow", 0, 3);
				LCD.drawString("Orange", 0, 4);
				LCD.drawString("Gray", 0, 5);
				LCD.drawString("Magenta", 0, 6);
				LCD.drawString(">Pink", 10, 0);
				LCD.drawString("Cyan", 10, 1);
				Button.waitForAnyPress();
				if (Button.UP.isDown()) {
					Button.discardEvents();
				} else if (Button.DOWN.isDown()) {
					currentColor = "Cyan";
				} else if (Button.LEFT.isDown()) {
					currentColor = "Red";
				} else if (Button.RIGHT.isDown()) {
					Button.discardEvents();
				} else if (Button.ENTER.isDown()) {
					menuShow = false;
					colorIndex = 8;
					break;
				} else {
					Button.discardEvents();
				}
			}
			while (currentColor == "Cyan") {
				LCD.clear();
				LCD.drawString("Red", 0, 0);
				LCD.drawString("Blue", 0, 1);
				LCD.drawString("Green", 0, 2);
				LCD.drawString("Yellow", 0, 3);
				LCD.drawString("Orange", 0, 4);
				LCD.drawString("Gray", 0, 5);
				LCD.drawString("Magenta", 0, 6);
				LCD.drawString("Pink", 10, 0);
				LCD.drawString(">Cyan", 10, 1);
				Button.waitForAnyPress();
				if (Button.UP.isDown()) {
					currentColor = "Pink";
				} else if (Button.DOWN.isDown()) {
					Button.discardEvents();
				} else if (Button.LEFT.isDown()) {
					currentColor = "Blue";
				} else if (Button.RIGHT.isDown()) {
					Button.discardEvents();
				} else if (Button.ENTER.isDown()) {
					menuShow = false;
					colorIndex = 12;
					break;
				} else {
					Button.discardEvents();
				}
			}
		}
		return colorIndex;
	}

	public static void beep() {
		Sound.setVolume(100);
		Sound.playTone(500, 1000);
	}
}