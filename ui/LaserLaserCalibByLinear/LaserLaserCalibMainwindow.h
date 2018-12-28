#ifndef LASERLASERCALIBMAINWINDOW_H
#define LASERLASERCALIBMAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QActionGroup>
#include "../CCPointCloudWindow/PointCloudWindow.h"
#include <opencv2/opencv.hpp>
#include "ExtrinsicCalibration/CalibrationByLinear.h"

using namespace cv;

class LaserLaserCalibMainwindow:public QMainWindow
{
	Q_OBJECT
public:
	LaserLaserCalibMainwindow(QWidget *parent = 0);
	~LaserLaserCalibMainwindow();

	void InitUI();

signals:
	void DisplayPointCloudRight(const POINTTYPE &pc);
	void DisplayPointCloudLeft(const POINTTYPE &pc);

private:
	PointCloudWindow* right_window;
	PointCloudWindow* left_window;

	QLabel *m_message_label;

	std::vector<std::string> m_right_pcd_list;
	std::vector<std::string> m_left_pcd_list;

	int m_calib_index;         //current pcd index
	int m_calib_max_num;       //pcd max num

	std::vector<cv::Vec3d> m_rotation_vectors;
	std::vector<cv::Vec3d> m_translation_vectors;

	LaserCalib m_calib;

	void getFiles(const std::string path, const std::string exd, std::vector<std::string>& files);
	int CheckDirPath(QString path);
	void InitProject();
	void DisplayCalibData(int index);
	void RefreshDisplay();

private slots:
	void OpenProject();
	void SaveProject();
	void NextFrame();
	void LastFrame();
	void LoadLaserLinear();
	void SaveLaserLinear();
	void LaserLaserCalibration();

	void SelectPointsSlotRight(const std::vector<int> &indexs);
	void SelectPointsSlotLeft(const std::vector<int> &indexs);



};




#endif // !LASERLASERCALIBMAINWINDOW_H