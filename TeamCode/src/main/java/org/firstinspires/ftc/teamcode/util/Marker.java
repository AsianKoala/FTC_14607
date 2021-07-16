package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedHashMap;
import java.util.Map;

public class Marker {
    // [freq,sum]
    public static LinkedHashMap<String, Object> telemetryMap = new LinkedHashMap<>();
    public static LinkedHashMap<String, long[]> freqMap = new LinkedHashMap<>();
    private static long startTime;

    public static void markStart() {
        startTime = System.currentTimeMillis();
    }

    public static void markEnd(String name) {
        long endTime = System.currentTimeMillis();
        long elapsedTime = endTime - startTime;
        long[] freqData = freqMap.get(name);
        if(freqData != null) {
            freqData[0]++;
            freqData[1]+=elapsedTime;
            freqMap.put(name, freqData);
        } else {
            freqMap.put(name, new long[]{1, elapsedTime});
        }
        freqData = freqMap.get(name);
        assert freqData != null;
        telemetryMap.put(name, (1.0 * freqData[1]) / freqData[0]);
        double smallest = 1000000000;
        for(Map.Entry<String, Object> e : telemetryMap.entrySet()) {
            smallest = Math.min(smallest, (double) e.getValue());
        }
        final double finalSmallest = smallest;
        telemetryMap.replaceAll((k,v) -> (double) v / finalSmallest);
    }
}
