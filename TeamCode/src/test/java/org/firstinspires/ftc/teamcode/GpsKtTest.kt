package org.firstinspires.ftc.teamcode

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class GpsKtTest {
    @Test
    fun calculatesNavigation() {
        assertEquals(45f, calculateHeading(0.0f, 0.0f, 1.0f, 1.0f))
        assertEquals(360f, calculateHeading(0.0f, 0.0f, 0.0f, 1.0f))
        assertEquals(225f, calculateHeading(0.0f, 0.0f, -1.0f, -1.0f))
        assertEquals(225f, calculateHeading(1.0f, 1.0f, -1.0f, -1.0f))
    }
}