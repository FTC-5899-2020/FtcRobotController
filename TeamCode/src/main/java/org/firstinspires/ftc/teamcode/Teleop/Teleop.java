package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "LiamTempTeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {
    //All hardware
    protected DcMotor motorFwdLeft = null;
    protected DcMotor motorFwdRight = null;
    protected DcMotor motorBackLeft = null;
    protected DcMotor motorBackRight = null;
    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;
    public DcMotor intakeFwd = null;
    public DcMotor intakeBack = null;
    public Servo basketServo = null;
    public Servo unloadServo = null;

    //Encoder Values
    // Neverest 40 motor spec: quadrature encoder, 280 pulses per revolution, count = 280 *4
    private static final double COUNTS_PER_MOTOR_REV = 1120; // Neverest 40 motor encoder All drive motor gearings
    private static final double DRIVE_GEAR_REDUCTION1 = .5; // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;

    //Variables
    private double max = 1.0;
    double maxPower;
    double powerLim = 1;
    double moveDir = 1;

    /*This function determines the number of ticks a motor
    would need to move in order to achieve a certain degree*/
    private int getCountsPerDegree(double degrees, int motorNumber){
        int ans = 0;
        if(motorNumber == 1){
            ans = (int)(degrees * COUNTS_PER_DEGREE1);
        }
        else{
            return 1;
        }
        return ans;
    }

    @Override
    public void runOpMode() {
        //Prepares all the hardware
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        intakeFwd = hardwareMap.get(DcMotor.class, "intakeFront");
        intakeBack = hardwareMap.get(DcMotor.class, "intakeBack");

        basketServo = hardwareMap.get(Servo.class,"basketServo");
        unloadServo = hardwareMap.get(Servo.class,"unloadServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //variables
        boolean changed1 = false;
        boolean changed2 = false;
        boolean changed3 = false;
        boolean changed4 = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //movement controls
            //collects input from the joysticks
            double fwdBackPower = this.gamepad1.left_stick_y * moveDir;
            double strafePower = this.gamepad1.left_stick_x * moveDir;
            double turnPower = this.gamepad1.right_stick_x;

            //does math to figure the power that should be applied to every motor
            double leftFrontPower = (fwdBackPower - turnPower - strafePower)*powerLim;
            double rightFrontPower = (fwdBackPower + turnPower - strafePower)*powerLim;
            double leftBackPower = (fwdBackPower - turnPower + strafePower)*powerLim;
            double rightBackPower = (fwdBackPower + turnPower + strafePower)*powerLim;


            maxPower = Math.abs(leftFrontPower);
            if (Math.abs(rightFrontPower) > maxPower) {
                maxPower = Math.abs(rightFrontPower);
            }
            if (Math.abs(leftBackPower) > maxPower) {
                maxPower = Math.abs(leftBackPower);
            }
            if (Math.abs(rightBackPower) > maxPower) {
                maxPower = Math.abs(rightBackPower);
            }
            if (maxPower > 1) {
                leftFrontPower = leftFrontPower / maxPower;
                rightFrontPower = rightFrontPower / maxPower;
                leftBackPower = leftBackPower / maxPower;
                rightBackPower = rightBackPower / maxPower;

            }
            //sets the power of the motors
            motorFwdLeft.setPower(leftFrontPower*max);
            motorFwdRight.setPower(rightFrontPower*max);
            motorBackLeft.setPower(leftBackPower*max);
            motorBackRight.setPower(rightBackPower*max);

            //collects input for shooter, and intake motors
            double intake = this.gamepad1.right_trigger - this.gamepad1.left_trigger;
            double shoot = this.gamepad2.right_trigger - this.gamepad2.left_trigger;


            //sets power for lift, plate, intake
            intakeFwd.setPower(intake);
            intakeBack.setPower(intake);


            //toggles for the two side arms
            /*
            if(gamepad2.x && !changed1) {//toggles the claw near to slides
                if(claw1Pos == .186){
                    claw1Pos = .789;
                    claw1.setPosition(claw1Pos);
                }
                else{
                    claw1Pos = .186;
                    claw1.setPosition(claw1Pos);
                }
                changed1 = true;
            } else if(!gamepad2.x){changed1 = false;}
            */

            //toggle for speed and direction of the bot for easier control
            if(gamepad1.b && !changed3) {//speed limiter toggle
                if(powerLim == .5){
                    powerLim = 1;
                }
                else{
                    powerLim = .5;
                }
                changed3 = true;
            } else if(!gamepad1.b){changed3 = false;}

            if(gamepad1.a && !changed4) {//direction change toggle
                if(moveDir == 1){
                    moveDir = -1;
                }
                else{
                    moveDir = 1;
                }
                changed4 = true;
            } else if(!gamepad1.a){changed4 = false;}
            telemetry.addData("Wheel Position", motorFwdLeft.getCurrentPosition()); //to be used when the encoders are ready
            telemetry.addData("Max Speed",powerLim);
            telemetry.addData("Direction",moveDir);
            telemetry.addData("basketServo",basketServo.getPosition());
            telemetry.addData("unloadServo",unloadServo.getPosition());
            telemetry.update();
        }
    }
}
