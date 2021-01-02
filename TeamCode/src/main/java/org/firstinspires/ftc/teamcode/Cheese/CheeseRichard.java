package org.firstinspires.ftc.teamcode.Cheese;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Cheesy Richard", group="Cheese")
//@Disabled
public class CheeseRichard extends AutoSuppliesCheese {
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();

        //  Wait until start
        waitForStart();

        //straight
        move(5000,0,1);
        pause(500);
        move(3000, 1,1);
        pause(500);
        turnToS(90,0.5, -2);
        pause(500);
        move(5000,0, 1);
        pause(500);
        turnToS(-180,.5,-2);
        pause(500);
        move(5000,0, 1);
        pause(500);

        move(3000,-1,1);
        pause(500);
        turnToS(-90, 0.5,-2);
        pause(500);
        move(5000,0, 1);
        pause(500);

        //  Turn all motors off and sleep
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(1000);
    }
}
