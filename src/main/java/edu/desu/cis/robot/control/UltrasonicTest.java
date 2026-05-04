package edu.desu.cis.robot.control;

public class UltrasonicTest extends RobotController {
    private static final long POLL_DELAY_MS = 500;

    public UltrasonicTest(String robotName) {
        super(robotName);
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            double distance = mbot.readUltrasonic();
            System.out.println("Distance: " + distance + " cm");

            try {
                Thread.sleep(POLL_DELAY_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    public static void main(String[] args) {
        try (UltrasonicTest robot = new UltrasonicTest("Vulcans")) {
            robot.run();
        }
    }
}
