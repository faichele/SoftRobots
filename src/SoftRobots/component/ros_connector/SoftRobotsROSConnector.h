#ifndef SOFT_ROBOTS_ROS_CONNECTOR_H
#define SOFT_ROBOTS_ROS_CONNECTOR_H

#include "../initSoftRobots.h"

#include <boost/thread/mutex.hpp>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Data.h>

#include <ZyROSConnector.h>
#include <ZyROSConnectionManager/ZyROSConnectionManager.h>
#include <ZyROSConnectorServiceServer.h>

#include "SoftRobots/component/constraint/CableConstraint.h"
#include "SoftRobots/component/constraint/SurfacePressureConstraint.h"

#include <sofa_softrobots_msgs/SoftRobotCableActuators.h>
#include <sofa_softrobots_msgs/SoftRobotSurfacePressureActuators.h>

using namespace Zyklio::ROSConnectionManager;

namespace Zyklio
{
    namespace ROSConnector
    {
        class SoftRobotsROSConnectorPrivate;
        class SOFA_SOFTROBOTS_API SoftRobotsROSConnector: public sofa::core::objectmodel::BaseObject
        {
            public:
                SOFA_CLASS(SoftRobotsROSConnector, sofa::core::objectmodel::BaseObject);
                SoftRobotsROSConnector();
                virtual ~SoftRobotsROSConnector();

                void init();
                void bwdInit();
                void reset();
                void cleanup();

                bool cableActuatorExists(const std::string&);
                bool surfacePressureActuatorExists(const std::string&);

                const std::vector<std::string> getCableActuatorNames() const;
                const std::vector<std::string> getSurfacePressureActuatorNames() const;

                sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>* getCableActuator(const std::string&);
                sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>* getSurfacePressureActuator(const std::string&);

                bool connectToROSMaster();
                bool disconnectFromROSMaster();

            protected:
                void searchCableActuators();
                void searchPneumaticActuators();

                void instantiateServiceServers();
                void shutdownServiceServers();

                SoftRobotsROSConnectorPrivate* m_d;

                Data<std::string> m_rosMasterURI;
                sofa::core::objectmodel::BaseContext* m_context;
                boost::mutex m_mutex;
        };

        /*class SofaSoftRobotsCableActuatorsServiceServer;
        class SofaSoftRobotsSurfacePressureActuatorsServiceServer;*/
    }
}

#endif // SOFT_ROBOTS_ROS_CONNECTOR_H
