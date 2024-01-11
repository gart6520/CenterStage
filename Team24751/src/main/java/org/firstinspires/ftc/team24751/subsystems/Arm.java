package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

public class Arm {
    // Hardware map
    private HardwareMap hardwareMap = null;

    // Motors
    private DcMotor leftArmMotor = null;
    private DcMotor rightArmMotor = null;
    private DcMotor elevatorMotor = null;

    // Servos
    private ServoImplEx wristServo = null;
    private ServoImplEx leftClawServo = null;
    private ServoImplEx rightClawServo = null;

    /**
     * Subsystem implementation for Arm
     * This should include:
     * - Basic arm movement: up / down
     * - Monitor current arm's angle
     * - Auto parallel of wrist and backdrop
     * - Auto limit arm's velocity based on gravity direction
     * - Move arm to a specific angle and stay there
     * - Software-stalling (if needed)
     */
    public Arm() {}

    public void init(LinearOpMode opMode) {
        // Get hardware map
        hardwareMap = opMode.hardwareMap;

        // Get arm motors
        leftArmMotor = hardwareMap.get(DcMotor.class, LEFT_ARM_MOTOR);
        rightArmMotor = hardwareMap.get(DcMotor.class, RIGHT_ARM_MOTOR);
        elevatorMotor = hardwareMap.get(DcMotor.class, ELEVATOR_MOTOR);

        // Get claw servos
        wristServo = (ServoImplEx)hardwareMap.get(Servo.class, WRIST_SERVO);
        leftClawServo = (ServoImplEx)hardwareMap.get(Servo.class, LEFT_CLAW);
        rightClawServo = (ServoImplEx)hardwareMap.get(Servo.class, RIGHT_CLAW);
    }

    public double getArmAngle() {
        return 0; // TODO
    }
}
