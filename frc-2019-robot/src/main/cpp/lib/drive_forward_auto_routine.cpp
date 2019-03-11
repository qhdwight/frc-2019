#include <lib/drive_forward_auto_routine.hpp>

namespace garage {
    namespace lib {
        void DriveForwardAutoRoutine::GetWaypoints() {
            m_Waypoints = {
                    {4.0, 0.0, d2r(0.0)}
            };
        }
    }
}
