#ifndef SOFTROBOTROSCONNECTORSERVERS_H
#define SOFTROBOTROSCONNECTORSERVERS_H

#include <ZyROSConnectorServiceServer.h>

#include "SoftRobots/component/constraint/CableConstraint.h"
#include "SoftRobots/component/constraint/SurfacePressureConstraint.h"

#include <sofa_softrobots_msgs/SoftRobotCableActuators.h>
#include <sofa_softrobots_msgs/SoftRobotSurfacePressureActuators.h>

namespace Zyklio
{
    namespace ROSConnector
    {
        class SoftRobotsROSConnector;

        template <class Request, class Response>
        struct SoftRobotsCableActuatorsRequestHandler: public ZyROSConnectorServerRequestHandler<Request, Response>
        {
            public:
                SoftRobotsCableActuatorsRequestHandler(void* param = nullptr):
                    m_softRobotsROSConnector(NULL),
                    ZyROSConnectorServerRequestHandler<Request, Response>(param)
                {
                    msg_info("SoftRobotsCableActuatorsRequestHandler") << "Called Request, Response, void* constructor.";
                    m_softRobotsROSConnector = (SoftRobotsROSConnector*)(param);
                }

                bool handleRequest(const Request&, Response&);

                void setSoftRobotsROSConnector(SoftRobotsROSConnector* connector)
                {
                    m_softRobotsROSConnector = connector;
                }

            private:
                SoftRobotsROSConnector* m_softRobotsROSConnector;
        };

        template <class Request, class Response>
        struct SoftRobotsSurfacePressureActuatorsRequestHandler: public ZyROSConnectorServerRequestHandler<Request, Response>
        {
            public:
                SoftRobotsSurfacePressureActuatorsRequestHandler(void* param = nullptr):
                    m_softRobotsROSConnector(NULL),
                    ZyROSConnectorServerRequestHandler<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse>(param)
                {
                    msg_info("SoftRobotsCableActuatorsRequestHandler") << "Called Request, Response, void* constructor.";
                    m_softRobotsROSConnector = (SoftRobotsROSConnector*)(param);
                }

                bool handleRequest(const Request&, Response&);

                void setSoftRobotsROSConnector(SoftRobotsROSConnector* connector)
                {
                    m_softRobotsROSConnector = connector;
                }

            private:
                SoftRobotsROSConnector* m_softRobotsROSConnector;
        };
    }
}

#endif // SOFTROBOTROSCONNECTORSERVERS_H
