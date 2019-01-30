#pragma once

namespace garage {
    struct Command {
    public:
        double driveForward, driveTurn, flipper;
        bool button;
    };
}
