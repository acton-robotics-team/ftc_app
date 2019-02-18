import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Hardware


@Autonomous(name = "Turn Test Auto")
class TurnTestsAuto : LinearOpMode() {

    override fun runOpMode() {
        val hw = Hardware(hardwareMap, this)
        hw.turnFromStart(180.0f)
        hw.turnFromStart(-90f)
    }
}