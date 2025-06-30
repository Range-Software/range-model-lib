#include "rml_monitoring_point.h"
#include "rml_monitoring_point_manager.h"

void RMonitoringPointManager::_init(const RMonitoringPointManager *pMonitoringPointManager)
{
    if (pMonitoringPointManager)
    {

    }
}

RMonitoringPointManager::RMonitoringPointManager()
{
    this->_init();
}

RMonitoringPointManager::RMonitoringPointManager(const RMonitoringPointManager &monitoringPointManager)
    : std::vector<RMonitoringPoint>(monitoringPointManager)
{
    this->_init(&monitoringPointManager);
}

RMonitoringPointManager::~RMonitoringPointManager()
{

}

RMonitoringPointManager &RMonitoringPointManager::operator =(const RMonitoringPointManager &monitoringPointManager)
{
    this->std::vector<RMonitoringPoint>::operator = (monitoringPointManager);
    this->_init(&monitoringPointManager);
    return (*this);
}

void RMonitoringPointManager::setOutputFileName(const QString &fileName)
{
    for (unsigned int i=0;i<this->size();i++)
    {
        this->at(i).setOutputFileName(fileName);
    }
}
