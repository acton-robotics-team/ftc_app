package org.firstinspires.ftc.teamcode

/**
 * Calculates the heading that the robot needs to turn to to reach
 * another point from a current point.
 *
 * @return the heading, in degrees, to turn to
 */
fun calculateHeading(x1: Float, y1: Float, x2: Float, y2: Float): Float {
    var brng = Math.atan2((x1 - x2).toDouble(), (y2 - y1).toDouble())
    brng *= (180 / Math.PI) // convert radians to degrees
    brng = (brng + 360) % 360
    brng = 360 - brng
    return brng.toFloat()
}

fun distanceTo(x1: Float, y1: Float, x2: Float, y2: Float): Float {
    return Math.sqrt(Math.pow((x1 - x2).toDouble(), 2.0) +
            Math.pow((y1 - y2).toDouble(), 2.0)).toFloat()
}

