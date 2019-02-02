package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Facing crater")
class AutonomousFacingCrater : BaseAutonomousWithArm() {
    override val startLocation: AutonomousStartLocation = AutonomousStartLocation.FACING_CRATER
}