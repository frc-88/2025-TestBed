package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {

    public static final String RIO_CANBUS = "rio";

    // CLIMBER
    public static final int CLIMBER_PIVOT_MOTOR = 17;
    public static final int CLIMBER_GRIPPER_MOTOR = 4;
    public static final double PIVOT_MOTOR_ROTATIONS_TO_CLIMBER_POSITION = (360.0 / 196.0);
    public static final double GRIPPER_MOTOR_ROTATIONS_TO_POSITION = (360 / 49.0);
}
