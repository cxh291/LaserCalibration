#include "LaserLaserCalibMainwindow.h"

//system
#include <iostream>

//Qt4
#include <QHBoxLayout>
#include <QAction>
#include <QMenu>
#include <QHBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QStatusBar>
#include <QKeyEvent>
#include <QFileInfoList>

//pcl

#include "Common_GQ.h"
using namespace std;
using namespace Common_GQ;

LaserLaserCalibMainwindow::LaserLaserCalibMainwindow(QWidget *parent/*=0*/) :QMainWindow(parent)
{
	InitUI();
}

LaserLaserCalibMainwindow::~LaserLaserCalibMainwindow()
{
}

void LaserLaserCalibMainwindow::InitUI() {

	QWidget *widget = new QWidget(this);

	right_window = new PointCloudWindow(this);
	left_window = new PointCloudWindow(this);

	QHBoxLayout *h_layout = new QHBoxLayout;
	h_layout->addWidget(left_window);
	h_layout->addWidget(right_window);
	h_layout->setStretch(0, 1);
	h_layout->setStretch(1, 1);
	widget->setLayout(h_layout);
	setCentralWidget(widget);

	QMenu *menu = menuBar()->addMenu("Project");
	connect(menu->addAction("Open"), SIGNAL(triggered()), this, SLOT(OpenProject()));
	connect(menu->addAction("Save"), SIGNAL(triggered()), this, SLOT(SaveProject()));


	menu = menuBar()->addMenu("View");
	QAction *act = menu->addAction("Last");
	act->setShortcut(Qt::Key_Up);
	connect(act, SIGNAL(triggered()), SLOT(LastFrame()));

	act = menu->addAction("Next");
	act->setShortcut(Qt::Key_Down);
	connect(act, SIGNAL(triggered()), SLOT(NextFrame()));

	menu = menuBar()->addMenu("Laser Calibration");
	connect(menu->addAction("LoadLaserlinearPoints"), SIGNAL(triggered()), SLOT(LoadLaserLinear()));
	connect(menu->addAction("SaveLaserlinearPoints"), SIGNAL(triggered()), SLOT(SaveLaserLinear()));
	connect(menu->addAction("Calibrate"), SIGNAL(triggered()), SLOT(LaserLaserCalibration()));


	m_message_label = new QLabel(statusBar());
	m_message_label->setText("Current Index: N/A");
	m_message_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
	statusBar()->addPermanentWidget(m_message_label);
	statusBar()->setStyleSheet(QString("QStatusBar::item{border: 0px}"));
	statusBar()->setSizeGripEnabled(true);
	statusBar()->showMessage("Please Open a project", 5);

	connect(this, SIGNAL(DisplayPointCloudRight(const POINTTYPE &)), right_window, SLOT(LoadPCLPointCloud(const POINTTYPE &)));
	connect(right_window, SIGNAL(SelectPoints(const std::vector<int> &)), this, SLOT(SelectPointsSlotRight(const std::vector<int> &)));

	connect(this, SIGNAL(DisplayPointCloudLeft(const POINTTYPE &)), left_window, SLOT(LoadPCLPointCloud(const POINTTYPE &)));
	connect(left_window, SIGNAL(SelectPoints(const std::vector<int> &)), this, SLOT(SelectPointsSlotLeft(const std::vector<int> &)));

}

void LaserLaserCalibMainwindow::OpenProject() {

	QString path = QFileDialog::getExistingDirectory(nullptr, "Project dir selection");
	if (!path.isEmpty())
	{
		if (CheckDirPath(path) > 0)
			InitProject();
	}
}

void LaserLaserCalibMainwindow::SaveProject() {

}

void LaserLaserCalibMainwindow::NextFrame() {
	if (m_calib_max_num == 0)
		return;
	DisplayCalibData(m_calib_index + 1);
}

void LaserLaserCalibMainwindow::LastFrame() {
	if (m_calib_max_num == 0)
		return;
	DisplayCalibData(m_calib_index - 1);
}

void LaserLaserCalibMainwindow::LoadLaserLinear() {

}

void LaserLaserCalibMainwindow::SaveLaserLinear() {

}

void LaserLaserCalibMainwindow::LaserLaserCalibration() {

}

void LaserLaserCalibMainwindow::SelectPointsSlotRight(const std::vector<int> &indexs) {
	m_calib.SetLinearPointsIndex(m_calib_index, indexs, false);
	if (indexs.size() == 0)
	{
		RefreshDisplay();
	}
}

void LaserLaserCalibMainwindow::SelectPointsSlotLeft(const std::vector<int> &indexs) {
	m_calib.SetLinearPointsIndex(m_calib_index, indexs, true);
	if (indexs.size() == 0)
	{
		RefreshDisplay();
	}
}

void LaserLaserCalibMainwindow::RefreshDisplay()
{
	DisplayCalibData(m_calib_index);
}

void LaserLaserCalibMainwindow::getFiles(const std::string path, const std::string exd, std::vector<std::string>& files) {
	//文件句柄  
	long long hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	std::string pathName, exdName;
	files.clear();
	if (0 != strcmp(exd.c_str(), ""))
	{
		exdName = "\\*." + exd;
	}
	else
	{
		exdName = "\\*";
	}

	if ((hFile = _findfirst(pathName.assign(path).append(exdName).c_str(), &fileinfo)) != -1)
	{
		do
		{
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					files.push_back(pathName.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

void LaserLaserCalibMainwindow::InitProject() {

	m_calib.Init(m_left_pcd_list, m_right_pcd_list);
	m_calib_max_num = m_right_pcd_list.size();
	DisplayCalibData(0);
	right_window->ZoomGlobal();
	left_window->ZoomGlobal();
}

void LaserLaserCalibMainwindow::DisplayCalibData(int index) {

	if (index >= m_calib_max_num)
		index = 0;
	if (index < 0)
		index = m_calib_max_num - 1;
	m_calib_index = index;
	QString disp;
	disp = QString(("Left: " + m_left_pcd_list[index] + "	 Right: " + m_right_pcd_list[index]).c_str());

	statusBar()->showMessage(disp);
	m_message_label->setText(" Current Index: " + QString::number(m_calib_index + 1) + '/' + QString::number(m_calib_max_num));

	POINTTYPE pc;
	pcl::io::loadPCDFile(m_left_pcd_list[m_calib_index], pc);
	emit DisplayPointCloudLeft(pc);

	pcl::io::loadPCDFile(m_right_pcd_list[m_calib_index], pc);
	emit DisplayPointCloudRight(pc);



}

int LaserLaserCalibMainwindow::CheckDirPath(QString path) {

	vector<string> list = Common_GQ::GetListFolders(path.toStdString());
	if (list.size() != 2)
	{
		return -1;
	}
	m_right_pcd_list = Common_GQ::GetListFiles(list[1], "pcd");
	m_left_pcd_list = Common_GQ::GetListFiles(list[0], "pcd");
	if (m_right_pcd_list.size() != m_left_pcd_list.size())
	{
		return -1;
	}
	return m_right_pcd_list.size();
}
