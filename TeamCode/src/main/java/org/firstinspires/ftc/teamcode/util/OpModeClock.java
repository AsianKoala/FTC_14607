package org.firstinspires.ftc.teamcode.util;

// only used if it HAS to be global
public class OpModeClock {
    private static long INIT_TIME;
    private static long START_TIME;
    private static double lastMessage = System.currentTimeMillis();

    public static void markInit() {
        INIT_TIME = System.nanoTime();
    }

    public static void markStart() {
        START_TIME = System.nanoTime();
    }

    public static int getElapsedInitTime() {
        return (int) (System.currentTimeMillis() - INIT_TIME);
    }

    public static int getElapsedStartTime() {
        return (int) (System.currentTimeMillis() - START_TIME);
    }

    public static boolean isOk() {
        if(System.currentTimeMillis() - lastMessage > 250) {
            lastMessage = System.currentTimeMillis();
            return true;
        }
        return false;
    }
}
