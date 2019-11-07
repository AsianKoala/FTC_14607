package org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Debugging;


import org.firstinspires.ftc.teamcode.PepeLaugh.HelperClasses.Robot;
import org.firstinspires.ftc.teamcode.lastMinutescuffedPP.HelperClasses.FloatPoint;

public class ComputerDebuggingTcp {
    private static TcpServer tcpServer;


    /**
     * Initializes udp server and starts it's thread
     */
    public ComputerDebuggingTcp(){
        TcpServer.kill = false;
        tcpServer = new TcpServer(11115);
        Thread runner = new Thread(tcpServer);
        runner.start();//go go go
    }



    /**
     * Sends the robot location to the debug computer
     */

    public void sendRobotLocation(Robot robot){
        tcpServer.addMessage("f");
    }

    /**
     * Sends the location of any other point you would like to send
     * @param floatPoint
     * @throws InterruptedException
     */
    public static void sendKeyPoint(FloatPoint floatPoint) {
        tcpServer.addMessage("POINT," + floatPoint.x + "," + floatPoint.y);
    }

    /**
     * Used for debugging lines
     * @param point1
     * @param point2
     */
    public static void sendLine(FloatPoint point1, FloatPoint point2){
        tcpServer.addMessage("LINE," + point1.x + "," + point1.y + "," + point2.x + "," + point2.y);
    }


    /**
     * This kills the tcpServer background thread
     */
    public static void stopAll() {
        tcpServer.kill = true;
    }


    /**
     * Call this at the end of every update so that we don't buffer too much
     */
    public static void markEndOfUpdate(){
        tcpServer.markEndUpdate();
    }
}
