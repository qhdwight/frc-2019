#include <test/test_drive_auto_routine.hpp>

namespace garage {
    namespace test {
        void TestDriveAutoRoutine::GetWaypoints() {
            m_Waypoints = {
                    {4.0, 0.0, d2r(0.0)},
                    {6.0, 2.0, d2r(90.0)},
                    {6.0, 6.0, d2r(90.0)},
                    {8.0, 8.0, d2r(0.0)},
                    {12.0, 8.0, d2r(0.0)},
            };
        }
    }
}
