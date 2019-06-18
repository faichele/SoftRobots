#include "SoftRobotROSConnectorServers.h"
#include <ZyROSConnectorServiceServer.inl>

#include "SoftRobotsROSConnector.h"

using namespace Zyklio::ROSConnector;

template class ZyROSConnectorServiceServerImpl<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse, SoftRobotsCableActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse>>;
template class ZyROSConnectorServiceServerImpl<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse, SoftRobotsSurfacePressureActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse>>;

template <class Request, class Response>
bool SoftRobotsCableActuatorsRequestHandler<Request, Response>::handleRequest(const Request& req, Response& resp)
{
    msg_info("SoftRobotsSurfacePressureActuatorsRequestHandler") << "Called handleRequest for CableActuator: " << req.cableActuator.data;
    if (this->m_softRobotsROSConnector && this->m_softRobotsROSConnector->cableActuatorExists(req.cableActuator.data))
    {
        msg_info("SoftRobotsCableActuatorsRequestHandler") << "CableActuator found, sending response data.";
        sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>* cableActuator = this->m_softRobotsROSConnector->getCableActuator(req.cableActuator.data);
        const std::vector<sofa::defaulttype::Vector3> actuatorPoints = cableActuator->getCableActuatorPoints();
        resp.cableActuatorPoints.resize(actuatorPoints.size());
        for (size_t k = 0; k < actuatorPoints.size(); k++)
        {
            resp.cableActuatorPoints[k].x = actuatorPoints[k].x();
            resp.cableActuatorPoints[k].y = actuatorPoints[k].y();
            resp.cableActuatorPoints[k].z = actuatorPoints[k].z();
        }

        resp.cableLength.data = cableActuator->getCurrentCableLength();
        resp.force.data = cableActuator->getForce();
        resp.displacement.data = cableActuator->getDisplacement();

        return true;
    }
    else
    {
        if (this->m_softRobotsROSConnector != NULL)
        {
            msg_warning("SoftRobotsCableActuatorsRequestHandler") << "CableActuator NOT found, not sending response data. Known CableActuators:";
            std::vector<std::string> actuatorNames = this->m_softRobotsROSConnector->getCableActuatorNames();
            for (size_t k = 0; k < actuatorNames.size(); k++)
                msg_warning("SoftRobotsCableActuatorsRequestHandler") << " - " << actuatorNames[k];
        }
        else
        {
            msg_error("SoftRobotsCableActuatorsRequestHandler") << "SoftRobotsROSConnector instance is NULL - this should never happen!";
        }

        return false;
    }
}

template <class Request, class Response>
bool SoftRobotsSurfacePressureActuatorsRequestHandler<Request, Response>::handleRequest(const Request& req, Response& resp)
{
    msg_info("SoftRobotsSurfacePressureActuatorsRequestHandler") << "Called handleRequest for SurfacePressureActuator: " << req.surfacePressureActuator.data;
    if (this->m_softRobotsROSConnector && this->m_softRobotsROSConnector->surfacePressureActuatorExists(req.surfacePressureActuator.data))
    {
        msg_info("SoftRobotsSurfacePressureActuatorsRequestHandler") << "SurfacePressureActuator found, sending response data.";
        sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>* surfacePressureActuator = this->m_softRobotsROSConnector->getSurfacePressureActuator(req.surfacePressureActuator.data);
        resp.cavityVolume.data = surfacePressureActuator->getCurrentCavityVolume();
        resp.pressure.data = surfacePressureActuator->getCurrentPressure();
        resp.volumeGrowth.data = surfacePressureActuator->getCurrentVolumeGrowth();
        return true;
    }
    else
    {
        msg_warning("SoftRobotsSurfacePressureActuatorsRequestHandler") << "SurfacePressureActuator NOT found, not sending response data. Known SurfacePressureActuators:";
        std::vector<std::string> actuatorNames = this->m_softRobotsROSConnector->getSurfacePressureActuatorNames();
        for (size_t k = 0; k < actuatorNames.size(); k++)
            msg_warning("SoftRobotsSurfacePressureActuatorsRequestHandler") << " - " << actuatorNames[k];

        return false;
    }
}

// Why is this needed: Linker complains about unresolved symbol for some unknown reason otherwise, even if this method specialization is not explicitly instantiated anywhere!
/*template <>
bool SoftRobotsSurfacePressureActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse>::handleRequest()
{
    msg_error("SoftRobotsSurfacePressureActuatorsRequestHandler") << "Called incorrect template specialization of handleRequest!";
    return false;
}

template <class RequestType, class ResponseType, class RequestHandler>
class SofaSoftRobotsCableActuatorsServiceServer: public ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>
{
    public:
        SofaSoftRobotsCableActuatorsServiceServer(const std::string& service_uri, SoftRobotsROSConnector* src):
            m_softRobotsConnector(src),
            ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>(service_uri)
        {
            this->m_requestHandler.setSoftRobotsROSConnector(this->m_softRobotsConnector);
        }

        ~SofaSoftRobotsCableActuatorsServiceServer()
        {}

        bool handleRequest(const sofa_softrobots_msgs::SoftRobotCableActuatorsRequest& req, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse& resp)
        {
            msg_info("SofaSoftRobotsCableActuatorsServiceServer") << "handleRequest called.";

            bool handler_status = this->m_requestHandler.handleRequest(req, resp);

            msg_info("SofaSoftRobotsCableActuatorsServiceServer") << "Response from handler: " << resp;

            return handler_status;
        }

    protected:
        SoftRobotsROSConnector* m_softRobotsConnector;
};*/

template <class RequestType, class ResponseType, class RequestHandler>
class SofaSoftRobotsActuatorsServiceServer: public ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>
{
    public:
        SofaSoftRobotsActuatorsServiceServer(const std::string& service_uri, SoftRobotsROSConnector* src):
            m_softRobotsConnector(src),
            ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>(service_uri)
        {
            this->m_requestHandler.setSoftRobotsROSConnector(this->m_softRobotsConnector);
        }

        ~SofaSoftRobotsActuatorsServiceServer()
        {}

        bool handleRequest(const RequestType& req, ResponseType& resp)
        {
            msg_info("SofaSoftRobotsActuatorsServiceServer") << "handleRequest called.";

            bool handler_status = this->m_requestHandler.handleRequest(req, resp);

            msg_info("SofaSoftRobotsActuatorsServiceServer") << "Response from handler: " << resp;

            return handler_status;
        }
    protected:
        SoftRobotsROSConnector* m_softRobotsConnector;
};

template class SofaSoftRobotsActuatorsServiceServer<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse, SoftRobotsCableActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse>>;
template class SofaSoftRobotsActuatorsServiceServer<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse, SoftRobotsSurfacePressureActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse>>;

namespace Zyklio
{
    namespace ROSConnector
    {
        class SoftRobotsROSConnectorPrivate
        {
            public:
                sofa::core::objectmodel::BaseContext* m_context;

                std::vector<sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>*> m_cableConstraints;
                std::vector<sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>*> m_surfacePressureConstraints;

                ZyROSConnectorServiceServerWorkerThread<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse, SoftRobotsCableActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse>>* m_serviceWorker_CableConstraints;
                ZyROSConnectorServiceServerWorkerThread<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse, SoftRobotsSurfacePressureActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse>> * m_serviceWorker_SurfacePressureConstraints;

                boost::shared_ptr<ZyROSConnectionManager> m_rosConnectionManager;

                boost::shared_ptr<ZyROSConnectorServiceServerImpl<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse, SoftRobotsCableActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse>>> m_cableActuatorsServiceServer;
                boost::shared_ptr<ZyROSConnectorServiceServerImpl<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse, SoftRobotsSurfacePressureActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse>>> m_surfacePressureActuatorsServiceServer;
        };
    }
}

SoftRobotsROSConnector::SoftRobotsROSConnector(): sofa::core::objectmodel::BaseObject(),
    m_rosMasterURI(initData(&m_rosMasterURI, std::string("http://localhost:11311"), "rosMasterURI", "ROS master URI to connect to", true, false))
{
    m_d = new SoftRobotsROSConnectorPrivate();
}

SoftRobotsROSConnector::~SoftRobotsROSConnector()
{
    delete m_d;
}

bool SoftRobotsROSConnector::connectToROSMaster()
{
    m_d->m_rosConnectionManager->init();
    m_d->m_rosConnectionManager->bwdInit();
    msg_info("SoftRobotsROSConnector") << "connectToROSMaster: " << m_rosMasterURI.getValue();
    m_d->m_rosConnectionManager->getROSConnector()->setRosMasterURI(m_rosMasterURI.getValue());

    boost::mutex::scoped_lock lock(m_mutex);
    while (!m_d->m_rosConnectionManager->getROSConnector()->isThreadRunning())
    {
        msg_info("SoftRobotsROSConnector") << "Waiting for connector thread to start...";
        m_d->m_rosConnectionManager->getROSConnector()->connectorCondition().wait(lock);
    }
    msg_info("SoftRobotsROSConnector") << "Connector thread started.";
    if (m_d->m_rosConnectionManager->getROSConnector()->isConnected())
    {
        msg_info("SoftRobotsROSConnector") << "Connection to roscore established.";
        return true;
    }

    msg_info("SoftRobotsROSConnector") << "Failed to connect to roscore.";
    return false;
}

bool SoftRobotsROSConnector::disconnectFromROSMaster()
{
    boost::mutex::scoped_lock lock(m_mutex);
    msg_info("SoftRobotsROSConnector") << "Cleaning up ROS connector.";
    m_d->m_rosConnectionManager->cleanup();

    return true;
}

void SoftRobotsROSConnector::init()
{
    msg_info("SoftRobotsROSConnector") << "init() SoftRobotsROSConnector.";
    m_context = this->getContext();
    m_d->m_rosConnectionManager.reset(new ZyROSConnectionManager());
}

void SoftRobotsROSConnector::bwdInit()
{
    msg_info("SoftRobotsROSConnector") << "bwdInit() SoftRobotsROSConnector.";
    searchCableActuators();
    searchPneumaticActuators();

    if (m_d && m_d->m_rosConnectionManager)
    {
        msg_info("SoftRobotsROSConnector") << "Starting ROS connector main loop.";
        m_d->m_rosConnectionManager->getROSConnector()->setRosMasterURI(this->m_rosMasterURI.getValue());

        msg_info("SoftRobotsROSConnector") << "Starting ROS connector main thread.";
        m_d->m_rosConnectionManager->getROSConnector()->startComponent();
        m_d->m_rosConnectionManager->getROSConnector()->resumeComponent();

        // Instantiate ROS service servers
        msg_info("SoftRobotsROSConnector") << "Instantiating ROS service servers.";
        instantiateServiceServers();

        msg_info("SoftRobotsROSConnector") << "Instantiating ROS service worker threads.";
        m_d->m_serviceWorker_CableConstraints = new ZyROSConnectorServiceServerWorkerThread<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse, SoftRobotsCableActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse>>(m_d->m_cableActuatorsServiceServer);
        m_d->m_serviceWorker_SurfacePressureConstraints = new ZyROSConnectorServiceServerWorkerThread<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse, SoftRobotsSurfacePressureActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse>>(m_d->m_surfacePressureActuatorsServiceServer);

        msg_info("SoftRobotsROSConnector") << "Starting ROS service server worker threads.";
        m_d->m_serviceWorker_CableConstraints->start();
        m_d->m_serviceWorker_SurfacePressureConstraints->start();
    }
    else
    {
        msg_warning("SoftRobotsROSConnector") << "Can not start ROS connector main loop, ROS connection manager is not instantiated!";
    }
}

void SoftRobotsROSConnector::reset()
{
    msg_info("SoftRobotsROSConnector") << "Resetting SoftRobotsROSConnector.";
}

void SoftRobotsROSConnector::cleanup()
{
    msg_info("SoftRobotsROSConnector") << "Cleaning up SoftRobotsROSConnector.";
    m_d->m_cableConstraints.clear();
    m_d->m_surfacePressureConstraints.clear();

    // Stop ROS service servers
    shutdownServiceServers();

    m_d->m_serviceWorker_CableConstraints->stop();
    m_d->m_serviceWorker_SurfacePressureConstraints->stop();

    delete m_d->m_serviceWorker_CableConstraints;
    m_d->m_serviceWorker_CableConstraints = NULL;

    delete m_d->m_serviceWorker_SurfacePressureConstraints;
    m_d->m_serviceWorker_SurfacePressureConstraints = NULL;

    // Shutdown ROS connector main loop
    m_d->m_rosConnectionManager->getROSConnector()->pauseComponent();
    m_d->m_rosConnectionManager->getROSConnector()->stopComponent();
}

void SoftRobotsROSConnector::searchCableActuators()
{
    sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>, std::vector<sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>* > > cb(&(m_d->m_cableConstraints));
    m_context->getObjects(sofa::core::objectmodel::TClassInfo<sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>>::get(), cb, sofa::core::objectmodel::TagSet(), sofa::core::objectmodel::BaseContext::SearchRoot);

    msg_info("SoftRobotsROSConnector") << "CableConstraint instances found: " << m_d->m_cableConstraints.size();
}

void SoftRobotsROSConnector::searchPneumaticActuators()
{
    sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>, std::vector<sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>* > > cb(&(m_d->m_surfacePressureConstraints));
    m_context->getObjects(sofa::core::objectmodel::TClassInfo<sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>>::get(), cb, sofa::core::objectmodel::TagSet(), sofa::core::objectmodel::BaseContext::SearchRoot);

    msg_info("SoftRobotsROSConnector") << "SurfacePressureConstraint instances found: " << m_d->m_surfacePressureConstraints.size();
}

bool SoftRobotsROSConnector::cableActuatorExists(const std::string& actuatorName)
{
    for (std::vector<sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>*>::const_iterator it = m_d->m_cableConstraints.begin(); it != m_d->m_cableConstraints.end(); ++it)
    {
        if ((*it)->getName() == actuatorName)
            return true;
    }
    return false;
}

bool SoftRobotsROSConnector::surfacePressureActuatorExists(const std::string& actuatorName)
{
    for (std::vector<sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>*>::const_iterator it = m_d->m_surfacePressureConstraints.begin(); it != m_d->m_surfacePressureConstraints.end(); ++it)
    {
        if ((*it)->getName() == actuatorName)
            return true;
    }
    return false;
}

const std::vector<std::string> SoftRobotsROSConnector::getCableActuatorNames() const
{
    std::vector<std::string> actuatorNames;
    for (std::vector<sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>*>::const_iterator it = m_d->m_cableConstraints.begin(); it != m_d->m_cableConstraints.end(); ++it)
    {
        actuatorNames.push_back((*it)->getName());
    }
    return actuatorNames;
}

const std::vector<std::string> SoftRobotsROSConnector::getSurfacePressureActuatorNames() const
{
    std::vector<std::string> actuatorNames;
    for (std::vector<sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>*>::const_iterator it = m_d->m_surfacePressureConstraints.begin(); it != m_d->m_surfacePressureConstraints.end(); ++it)
    {
        actuatorNames.push_back((*it)->getName());
    }
    return actuatorNames;
}

sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>* SoftRobotsROSConnector::getCableActuator(const std::string& actuatorName)
{
    for (std::vector<sofa::component::constraintset::CableConstraint<sofa::defaulttype::Vec3Types>*>::const_iterator it = m_d->m_cableConstraints.begin(); it != m_d->m_cableConstraints.end(); ++it)
    {
        if ((*it)->getName() == actuatorName)
            return (*it);
    }
    return nullptr;
}

sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>* SoftRobotsROSConnector::getSurfacePressureActuator(const std::string& actuatorName)
{
    for (std::vector<sofa::component::constraintset::SurfacePressureConstraint<sofa::defaulttype::Vec3Types>*>::const_iterator it = m_d->m_surfacePressureConstraints.begin(); it != m_d->m_surfacePressureConstraints.end(); ++it)
    {
        if ((*it)->getName() == actuatorName)
            return (*it);
    }
    return nullptr;
}

void SoftRobotsROSConnector::instantiateServiceServers()
{
    /*if (m_d->m_cableActuatorsServiceServer)
    {
        delete m_d->m_cableActuatorsServiceServer;
        m_d->m_cableActuatorsServiceServer = NULL;
    }

    if (m_d->m_surfacePressureActuatorsServiceServer)
    {
        delete m_d->m_surfacePressureActuatorsServiceServer;
        m_d->m_surfacePressureActuatorsServiceServer = NULL;
    }*/

    m_d->m_cableActuatorsServiceServer.reset(new SofaSoftRobotsActuatorsServiceServer<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse, SoftRobotsCableActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse>>(/*m_d->m_rosConnectionManager->getRosNodeHandle(),*/ "/SoftRobots/CableActuators", this));
    m_d->m_surfacePressureActuatorsServiceServer.reset(new SofaSoftRobotsActuatorsServiceServer<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse, SoftRobotsSurfacePressureActuatorsRequestHandler<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse>>(/*m_d->m_rosConnectionManager->getRosNodeHandle(),*/ "/SoftRobots/SurfacePressureActuators", this));
}

void SoftRobotsROSConnector::shutdownServiceServers()
{
    msg_info("SoftRobotsROSConnector") << "Shutting down service servers.";
    m_d->m_cableActuatorsServiceServer->stopAdvertisingService();
    m_d->m_surfacePressureActuatorsServiceServer->stopAdvertisingService();

    m_d->m_cableActuatorsServiceServer->shutdownServer();
    m_d->m_surfacePressureActuatorsServiceServer->shutdownServer();
}
