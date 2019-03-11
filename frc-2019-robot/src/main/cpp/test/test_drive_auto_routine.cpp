#include <test/test_drive_auto_routine.hpp>

namespace garage {
    namespace test {
        void TestDriveAutoRoutine::GetWaypoints() {
            m_Waypoints = {
                    {4.0, 0.0, d2r(0.0)},
                    {6.0, 1.0, d2r(90.0)},
                    {6.0, 6.0, d2r(90.0)}
            };
        }
    }
}
