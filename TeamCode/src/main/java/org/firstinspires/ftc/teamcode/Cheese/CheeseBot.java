package org.firstinspires.ftc.teamcode.Cheese;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Cheesy Boi", group="Cheese")
//@Disabled
public class CheeseBot extends LinearOpMode {
        //All hardware
        protected DcMotor motorFwdLeft = null;
        protected DcMotor motorFwdRight = null;
        protected DcMotor motorBackLeft = null;
        protected DcMotor motorBackRight = null;
        //text- I love you bro <3
        //Variables
        private double max = 1.0;
        double maxPower;
        double powerLim = 1;
        double moveDir = -1;

        @Override
        public void runOpMode() {
            //Prepares all the hardware
            motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
            motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
            motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
            motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
            motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
            motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            //variables
            boolean changed3 = false;
            boolean changed4 = false;

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                //collects input from the joysticks
                double fwdBackPower = this.gamepad1.left_stick_y * moveDir;
                double strafePower = this.gamepad1.left_stick_x * moveDir;
                double turnPower = this.gamepad1.right_stick_x;

                //does math to figure the power that should be applied to every motor
                double leftFrontPower = (fwdBackPower - turnPower - strafePower)*powerLim;
                double rightFrontPower = (fwdBackPower + turnPower + strafePower)*powerLim;
                double leftBackPower = (fwdBackPower - turnPower + strafePower)*powerLim;
                double rightBackPower = (fwdBackPower + turnPower - strafePower)*powerLim;


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
                    telemetry.addData("Works",true);
                    telemetry.update();
                }
                //sets the power of the motors
                motorFwdLeft.setPower(leftFrontPower*max);
                motorFwdRight.setPower(rightFrontPower*max);
                motorBackLeft.setPower(leftBackPower*max);
                motorBackRight.setPower(rightBackPower*max);


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
                telemetry.update();
            }
        }
    }