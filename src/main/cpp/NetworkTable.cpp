#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include "networktables/DoubleTopic.h"
#include <frc/TimedRobot.h>

class NetworkTable : public frc::TimedRobot
{
public:
    nt::DoublePublisher tvPub;
    nt::DoublePublisher txPub;
    nt::DoublePublisher tyPub;
    nt::DoublePublisher clPub;

    void RobotInit()
    {
        auto limelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

        tvPub = limelightTable->GetDoubleTopic("tv").Publish();
        txPub = limelightTable->GetDoubleTopic("tx").Publish();
        tyPub = limelightTable->GetDoubleTopic("ty").Publish();
        clPub = limelightTable->GetDoubleTopic("cl").Publish();
    }
};