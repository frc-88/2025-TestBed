package frc.robot;

public class Constants {

    public static final String RIO_CANBUS = "rio";

    // CLIMBER
    public static final int CLIMBER_PIVOT_MOTOR = 17;
    public static final int CLIMBER_GRIPPER_MOTOR = 4;
    public static final int CLIMBER_GAS_MOTOR = 0;
    public static final int CLIMBER_ENCODER = 2;

    public static final double PIVOT_MOTOR_ROTATIONS_TO_CLIMBER_POSITION = (360.0 / 196.0);
    public static final double GRIPPER_MOTOR_ROTATIONS_TO_POSITION = (360 / 49.0);
    public static final double GAS_MOTOR_ROTATIONS_TO_LENGTH = (1.39 / 28.0);
    public static final double CLIMBER_ENCODER_ROTATIONS_TO_ANGLE = 360;

    public static final double ARM_L4_ANGLE = 20.0;

    public static final double ELEVATOR_L4_HEIGHT = 25.2;
    public static final double ELEVATOR_L3_HEIGHT = 13.25;
    public static final double ELEVATOR_L2_HEIGHT = 5.3;

    // ARMEVATOR
    public static final int ELEVATOR_MAIN_MOTOR = 3;
    public static final int ELEVATOR_FOLLOWER_MOTOR = 1;
    public static final int ELEVATOR_ARM_MOTOR = 7;
    public static final int ELEVATOR_MANIPULATOR_MOTOR = 5;
    public static final int ARM_LEFT_CANRANGE = 0;
    public static final int ARM_MIDDLE_CANRANGE = 10;
    public static final int ARM_RIGHT_CANRANGE = 3;


    public static final double ELEVATOR_ROTATIONS_TO_INCHES = ((2.256*Math.PI)/8.0);
    public static final double ARM_ROTATIONS_TO_DEGREES = (360.0 / 112.0);

    // DOGHOUSE
    public static final int DOGHOUSE_FUNNEL_MOTOR = 6;

}
