package com.arcrobotics.ftclib.drivebase;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.RobotLog;


/**
 * Kiwi 3 wheeled Holonomic drivebase
 */
public class KiwiDrive extends RobotDrive {
    Motor[] motors;

    public static final double kiwiDefaultLeftMotorAngle = Math.PI / 3;
    public static final double kiwiDefaultRightMotorAngle = 2 * Math.PI / 3;
    public static final double kiwiDefaultSlideMotorAngle = 3 * Math.PI / 2;

    private double rightMotorAngle = kiwiDefaultRightMotorAngle;
    private double leftMotorAngle = kiwiDefaultLeftMotorAngle;
    private double slideMotorAngle = kiwiDefaultSlideMotorAngle;

    /**
     * Constructor for the H-Drive class, which requires at least three motors.
     *
     * @param mLeft  one of the necessary primary drive motors
     * @param mRight one of the necessary primary drive motors
     * @param slide  the necessary slide motor for the use of h-drive
     */
    public KiwiDrive(Motor mLeft, Motor mRight, Motor slide) {
        motors = new Motor[3];
        motors[MotorType.kLeft.value] = mLeft;
        motors[MotorType.kRight.value] = mRight;
        motors[MotorType.kSlide.value] = slide;
    }

    /**
     * The constructor that includes the angles of the motors.
     *
     * <p>
     * The default angles are {@value #kDefaultRightMotorAngle},
     * {@value #kDefaultLeftMotorAngle}, {@value #kDefaultSlideMotorAngle}.
     * </p>
     *
     * @param mLeft           one of the necessary primary drive motors
     * @param mRight          one of the necessary primary drive motors
     * @param slide           the necessary slide motor for the use of h-drive
     * @param leftMotorAngle  the angle of the left motor in radians
     * @param rightMotorAngle the angle of the right motor in radians
     * @param slideMotorAngle the angle of the slide motor in radians
     */
    public KiwiDrive(Motor mLeft, Motor mRight, Motor slide, double leftMotorAngle,
                  double rightMotorAngle, double slideMotorAngle) {
        motors = new Motor[3];
        motors[0] = mLeft;
        motors[1] = mRight;
        motors[2] = slide;

        this.leftMotorAngle = leftMotorAngle;
        this.rightMotorAngle = rightMotorAngle;
        this.slideMotorAngle = slideMotorAngle;
    }

    /**
     * Sets up the constructor for the holonomic drive.
     *
     * @param myMotors The motors in order of:
     *                 frontLeft, frontRight, backLeft, backRight.
     *                 Do not input in any other order.
     */
    public KiwiDrive(Motor... myMotors) {
        motors = myMotors;
    }

    /**
     * Sets the range of the input, see {@link RobotDrive} for more info.
     *
     * @param min The minimum value of the range.
     * @param max The maximum value of the range.
     */
    public void setRange(double min, double max) {
        super.setRange(min, max);
    }

    /**
     * Sets the max speed of the drivebase, see {@link RobotDrive} for more info.
     *
     * @param value The maximum output speed.
     */
    public void setMaxSpeed(double value) {
        super.setMaxSpeed(value);
    }

    @Override
    public void stop() {
        for (Motor x : motors) {
            x.stopMotor();
        }
    }
    
        /**
     * Returns scalar projection of this vector onto another vector.
     *
     * @param other Vector onto which to project this vector.
     */
    public double scalarProject(Vector2d current, Vector2d other) {
        double magnitude = Math.sqrt(other.getX() * other.getX() + other.getY() * other.getY());
        double dot = current.getX() * other.getX() + current.getY() * other.getY();
        return dot / magnitude;
    }
    
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turn, double heading) {
        RobotLog.d(String.format("KiwiDrive:driveFieldCentric - strafeSpeed=%.03f forwardSpeed=%03f turn=%3f heading=%03f",strafeSpeed,forwardSpeed,turn,heading));
        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);

        Vector2d vector = new Vector2d(forwardSpeed,strafeSpeed );
        
        RobotLog.d(String.format("KiwiDrive:driveFieldCentric before-rotate - vector.getX=%.03f vector.getY=%03f",vector.getX(),vector.getY()));

        vector = vector.rotateBy(heading);

        RobotLog.d(String.format("KiwiDrive:driveFieldCentric after-rotate - vector.getX=%.03f vector.getY=%03f",vector.getX(),vector.getY()));

        double theta = vector.angle();
        
        double[] speeds = new double[motors.length];

        if (speeds.length == 3) {
            Vector2d leftVec = new Vector2d(Math.cos(leftMotorAngle), Math.sin(leftMotorAngle));
            Vector2d rightVec = new Vector2d(Math.cos(rightMotorAngle), Math.sin(rightMotorAngle));
            Vector2d slideVec = new Vector2d(Math.cos(slideMotorAngle), Math.sin(slideMotorAngle));

            speeds[MotorType.kLeft.value] = vector.scalarProject(leftVec) + turn;
            speeds[MotorType.kRight.value] = vector.scalarProject(rightVec) + turn;
            speeds[MotorType.kSlide.value] = vector.scalarProject(slideVec) + turn;

            normalize(speeds);

            motors[MotorType.kLeft.value].set(speeds[MotorType.kLeft.value] * maxOutput);
            motors[MotorType.kRight.value].set(speeds[MotorType.kRight.value] * maxOutput);
            motors[MotorType.kSlide.value].set(speeds[MotorType.kSlide.value] * maxOutput);
        }

    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turn) {
        driveFieldCentric(strafeSpeed, forwardSpeed, turn, 0.0);
    }
}
