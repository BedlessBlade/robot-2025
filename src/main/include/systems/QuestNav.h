#pragma once

#include <frc/geometry/Pose2d.h>
class QuestNav {
public:
    QuestNav();
    frc::Pose2d GetQuestPose();
    void Calibrate();
private:
    static std::shared_ptr<nt::NetworkTable> table;
};
