#pragma once
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QtWidgets/QMainWindow>
#include "ui_QtWidgetsApplication3.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class QtWidgetsApplication3 : public QMainWindow
{
    Q_OBJECT

public:
    QtWidgetsApplication3(QWidget *parent = nullptr);
    ~QtWidgetsApplication3();

private:
    Ui::QtWidgetsApplication3Class ui;
    //点云数据存储
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    //初始化vtk部件
    void initialVtkWidget();

    private slots:
        void pushButton_OpenClicked();
        void pushButton_ColorPickerClicked();
        void pushButton_PointCloudColorPickerClicked();


};
