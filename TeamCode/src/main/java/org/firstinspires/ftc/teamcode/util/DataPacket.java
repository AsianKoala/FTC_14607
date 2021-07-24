package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class DataPacket extends TelemetryPacket {

    public void addData(String key, Object val) {
        addLine(key + ": " + val.toString());
    }

    public void addSpace() {
        addLine(" ");
    }
}
