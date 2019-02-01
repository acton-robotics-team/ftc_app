package org.firstinspires.ftc.teamcode

import android.util.Log
import org.firstinspires.ftc.robotcore.external.Telemetry

fun Telemetry.logEx(message: String) {
    this.log().add(message)
    this.update()
    Log.wtf("FTCOpMode", message)
}