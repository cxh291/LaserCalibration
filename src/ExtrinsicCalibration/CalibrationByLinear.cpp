#include "CalibrationByLinear.h"
#include <QBoxLayout>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <qfile.h>
using namespace std;
LaserCalib::LaserCalib()
{
}


LaserCalib::~LaserCalib()
{
}

bool LaserCalib::Init(const std::vector<std::string> &left, const std::vector<std::string> &right)
{
	m_laser1_list = left;
	m_laser2_list = right;
	int count = m_laser2_list.size();
	m_linear_points_list_1.resize(count);
	m_linear_points_list_2.resize(count);
	return true;
}

std::string LaserCalib::GetLaserPath(int index, bool is_laser1/*=true*/) const
{
	return is_laser1 ? m_laser1_list[index] : m_laser2_list[index];
}

std::vector<int> LaserCalib::GetLinearPoints(int index, bool is_laser1/*=true*/) const
{
	return is_laser1 ? m_linear_points_list_1[index] : m_linear_points_list_2[index];
}

void LaserCalib::SetLinearPointsIndex(int index, const std::vector<int> &list, bool is_laser1)
{
	if (is_laser1)
	{
		m_linear_points_list_1[index] = list;
	}
	else
		m_linear_points_list_2[index] = list;
}