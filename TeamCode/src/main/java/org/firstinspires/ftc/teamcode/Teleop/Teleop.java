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
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "CompetitionTeleOp", group = "CompetitionTeleOp")

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
    public Servo wobbleArmServo = null;
    public Servo wobbleGrabberServo = null;
    public Servo ringPullPivotServo = null;
    public Servo ringPullArmServo = null;
    public RevBlinkinLedDriver lights;

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

    //Op Mode Members
    protected ElapsedTime runtime = new ElapsedTime();
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
        intakeFwd = hardwareMap.get(DcMotor.class, "intakeFwd");
        intakeBack = hardwareMap.get(DcMotor.class, "intakeBack");

        basketServo = hardwareMap.get(Servo.class,"basketServo");
        unloadServo = hardwareMap.get(Servo.class,"unloadServo");
        wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");
        wobbleGrabberServo = hardwareMap.get(Servo.class, "wobbleGrabberServo");
        ringPullPivotServo = hardwareMap.get(Servo.class, "ringPullPivotServo");
        ringPullArmServo = hardwareMap.get(Servo.class, "ringPullArmServo");

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //variables
        boolean changed1 = false;
        boolean changed2 = false;
        boolean changed3 = false;
        boolean changed4 = false;
        boolean changed5 = false;
        boolean changed6 = false;
        boolean pullPressed = false;
        boolean firstPull = true;
        double pullTime1 = 0;
        double basket = .64;
        double unload = .71;
        double wobbleArm = 0.238;
        double wobbleGrabber = 0.00;
        double ringPullArm = .4419;
        double ringPullPivot = 0.0699;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //movement controls
            //collects input from the joysticks
            double fwdBackPower = -this.gamepad1.left_stick_y * moveDir;
            double strafePower = this.gamepad1.left_stick_x * moveDir;
            double turnPower = -this.gamepad1.right_stick_x;

            //does math to figure the power that should be applied to every motor
            double leftFrontPower = (fwdBackPower - turnPower + strafePower)*powerLim;
            double rightFrontPower = (fwdBackPower + turnPower - strafePower)*powerLim;
            double leftBackPower = (fwdBackPower - turnPower - strafePower)*powerLim;
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
            double intake;
            if(this.gamepad2.right_bumper){
                if(this.gamepad2.left_bumper){
                    intake = 0;
                }
                else{intake = 1;}
            }
            else if(this.gamepad2.left_bumper){
                intake = -1;
            }
            else{intake = 0;}
            double shoot = this.gamepad2.right_trigger - this.gamepad2.left_trigger;


            //sets power for intake and shooter
            intakeFwd.setPower(-intake);
            intakeBack.setPower(intake);
            shooterLeft.setPower(shoot);
            shooterRight.setPower(-shoot);

            //toggle for various actions for the bot for easier control
            if(gamepad2.x && !changed5) {//wobble arm posistion toggle
                if(wobbleArm == .00999){
                    wobbleArm = .2839;
                }
                else{
                    wobbleArm = .00999;
                }
                changed5 = true;
            } else if(!gamepad2.x){changed5 = false;}

            if(gamepad2.y && !changed6) {//wobble grabber posistion toggle - NOT CORRECT POSISTIONS
                if(wobbleGrabber == 0){
                    wobbleGrabber = .319;
                }
                else{
                    wobbleGrabber = 0;
                }
                changed6 = true;
            } else if(!gamepad2.y){changed6 = false;}

            if(gamepad2.a && !changed1) {//basket posistion toggle
                if(basket == .64){
                    basket = .4069;
                }
                else{
                    basket = .64;
                }
                changed1 = true;
            } else if(!gamepad2.a){changed1 = false;}

            if(gamepad2.b && !changed2) {//unloading arm posistion toggle
                if(unload == .32399){
                    unload = .710;
                }
                else{
                    unload = .32399;
                }
                changed2 = true;
            } else if(!gamepad2.b){changed2 = false;}

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
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
                else{
                    moveDir = 1;
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                }
                changed4 = true;
            } else if(!gamepad1.a){changed4 = false;}

            //toggle for servo pull arm movement
            if(gamepad2.dpad_up && !pullPressed){
                pullPressed = true;
            }
            else if(pullPressed && gamepad2.dpad_up){
                if(firstPull){
                    pullTime1 = runtime.milliseconds();
                    ringPullPivot = 0.4429; //in
                    basket = .561;
                    firstPull = false;
                }
                else{
                    double timeDif = runtime.milliseconds() - pullTime1;
                    telemetry.addData("timeDif", timeDif);
                    if(timeDif > 0 && timeDif <= 300){
                        //exists to ensure that the else is not called incorrectly
                    }
                    else if(timeDif > 300 && timeDif <= 800){
                        ringPullArm = 0.5939; //down
                    }
                    //else if(timeDif >800 && timeDif <=1300){
                    //    ringPullPivot = 0.260;
                    //}
                    else if(timeDif > 800 && timeDif <= 1300){
                        ringPullArm = 0.4119; //up
                        basket = .64;
                    }
                    else if(timeDif > 1300){
                        ringPullPivot = 0.0649; //out to starting pos
                    }
                    else{
                        pullPressed = false;
                        firstPull = true;
                    }
                }
            }
            else{
                pullPressed = false;
                firstPull = true;
                ringPullPivot = .0649;
                ringPullArm = .4119;
                if(basket != .4069){
                    basket = .64;
                }
            }

            //assign servo position values
            basketServo.setPosition(basket);
            unloadServo.setPosition(unload);
            wobbleArmServo.setPosition(wobbleArm);
            wobbleGrabberServo.setPosition(wobbleGrabber);
            ringPullPivotServo.setPosition(ringPullPivot);
            ringPullArmServo.setPosition(ringPullArm);


            telemetry.addData("firstPull", firstPull);
            telemetry.addData("Wheel Position", motorFwdLeft.getCurrentPosition()); //to be used when the encoders are ready
            telemetry.addData("Max Speed",powerLim);
            telemetry.addData("Direction",moveDir);
            telemetry.addData("basketServo",basketServo.getPosition());
            telemetry.addData("unloadServo",unloadServo.getPosition());
            telemetry.update();
        }
    }
}
