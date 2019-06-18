#include "SoftRobotsROSConnector.h"
#include "SoftRobotROSConnectorServers.h"

#include <sofa/core/ObjectFactory.h>

using namespace Zyklio::ROSConnector;

SOFA_DECL_CLASS(SoftRobotsROSConnector)

int SoftRobotsROSConnectorClass = sofa::core::RegisterObject("Soft robots ROS connector.")
.add< SoftRobotsROSConnector >()
;

