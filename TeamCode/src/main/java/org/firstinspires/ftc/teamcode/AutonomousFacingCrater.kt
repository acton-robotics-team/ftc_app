package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous: Facing crater")
class AutonomousFacingCrater : BaseAutonomous() {
    override val startLocation: AutonomousStartLocation = AutonomousStartLocation.CRATER
}