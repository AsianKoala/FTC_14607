package org.firstinspires.ftc.teamcode.util

import java.util.LinkedHashMap

object Marker {
    // [freq,sum]
    // todo fix
    var telemetryMap = LinkedHashMap<String, Any>()
    var freqMap = LinkedHashMap<String, LongArray>()
    private var startTime: Long = 0
    fun markStart() {
        startTime = System.currentTimeMillis()
    }

    fun markEnd(name: String) {
        val endTime = System.currentTimeMillis()
        val elapsedTime = endTime - startTime
        var freqData = freqMap[name]
        if (freqData != null) {
            freqData[0]++
            freqData[1] += elapsedTime
            freqMap[name] = freqData
        } else {
            freqMap[name] = longArrayOf(1, elapsedTime)
        }
        freqData = freqMap[name]
        assert(freqData != null)
        telemetryMap[name] = 1.0 * freqData!![1] / freqData[0]
        var smallest = 1000000000.0
        for ((_, value) in telemetryMap) {
            smallest = Math.min(smallest, value as Double)
        }
        val finalSmallest = smallest
        telemetryMap.replaceAll { _: String?, v: Any -> v as Double / finalSmallest }
    }
}