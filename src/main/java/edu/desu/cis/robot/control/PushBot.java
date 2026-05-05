package edu.desu.cis.robot.control;

public class PushBot extends RobotController {
    private static final double STOP_THRESHOLD_CM = 15.0;
    private static final long POLL_DELAY_MS = 50;

    public PushBot(String robotName) {
        super(robotName);
    }

    @Override
    public void run() {
        mbot.pushObject();
        //mbot.pushObject();
        /*
        mbot.avoidCrashing(STOP_THRESHOLD_CM);
        mbot.forward(20);
        while (true) {
            double distance = mbot.readUltrasonic();
            if (distance > 0 && distance <= STOP_THRESHOLD_CM) {

                mbot.turnLeft(180);
                mbot.straight(-distance * 1.5);
                mbot.moveAndTurnRight(-40, 2,20);

                mbot.moveAndTurnRight(40, 2,20);
                mbot.straight(distance * 1.3);
                mbot.turnLeft(180);

                //Make find black line


                //find black line again

                mbot.stopBehavior("AVOID_CRASHING");
                mbot.pushObject();
                mbot.avoidCrashing(STOP_THRESHOLD_CM);

                mbot.moveAndTurnLeft(40, 5,40);


                //when detects object and comes into threshold

                //move forward and find black line and then continue
            }

            try {
                Thread.sleep(POLL_DELAY_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        */
    }

    public static void main(String[] args) {
        try (PushBot robot = new PushBot("Stingbot")) {
            robot.run();
        }
    }
}
