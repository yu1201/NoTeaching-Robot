#define POINTCLOUDEXTRATION_IMPORTS
#include "PointCloudExtration.h"

int main()
{
    TrackPointsPosition* points = nullptr;
    ReleaseTrackPoints(&points);
    return points == nullptr ? 0 : 1;
}
