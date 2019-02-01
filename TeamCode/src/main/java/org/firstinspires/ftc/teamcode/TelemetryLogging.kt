package org.firstinspires.ftc.teamcode

import android.util.Log
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * Enhanced logger that also logs to logcat for ease of debugging
 */
fun Telemetry.logEx(message: String) {
    Log.d("FTCOPMODE", message)
    this.log().add(message)
    this.update()
}