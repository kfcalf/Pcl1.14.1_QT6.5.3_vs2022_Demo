#include "QtWidgetsApplication3.h"

#include <QFileDialog>
#include <iostream>
#include <vtkRenderWindow.h>
#include <QColorDialog>
#include "vtkGenericOpenGLRenderWindow.h"
#include "QVTKOpenGLNativeWidget.h"
#include <QFuture>
#include <QtConcurrent/QtConcurrent>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#pragma execution_character_set("utf-8")


QtWidgetsApplication3::QtWidgetsApplication3(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    initialVtkWidget();
    ui.progressBar->setValue(0); // 初始化进度条
}

QtWidgetsApplication3::~QtWidgetsApplication3()
{}

void QtWidgetsApplication3::initialVtkWidget()
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    cloud->resize(1);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
    viewer->addPointCloud(cloud, "cloud");

    // 添加坐标轴线条
    double lineWidth = 0.8; // 坐标轴粗细
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(1, 0, 0), 1.0, 0.0, 0.0, "x_axis", lineWidth);
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 1, 0), 0.0, 1.0, 0.0, "y_axis", lineWidth);
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 1), 0.0, 0.0, 1.0, "z_axis", lineWidth);


     //添加X, Y, Z标注
    viewer->addText3D("X", pcl::PointXYZ(1.1, 0, 0), 0.1, 1.0, 0.0, 0.0, "x_text");
    viewer->addText3D("Y", pcl::PointXYZ(0, 1.1, 0), 0.1, 0.0, 1.0, 0.0, "y_text");
    viewer->addText3D("Z", pcl::PointXYZ(0, 0, 1.1), 0.1, 0.0, 0.0, 1.0, "z_text");

    ui.openGLWidget->setRenderWindow(viewer->getRenderWindow());

    viewer->setupInteractor(ui.openGLWidget->interactor(), ui.openGLWidget->renderWindow());
    ui.openGLWidget->update();

    viewer->setBackgroundColor(0, 0, 0);
}

void QtWidgetsApplication3::pushButton_OpenClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open PointCloud"), ".",
        tr("Point Cloud Files (*.pcd *.ply *.stl)"));

    if (!fileName.isEmpty())
    {
        ui.progressBar->setValue(0); // 重置进度条

        QtConcurrent::run([this, fileName]() {
            std::string file_name = fileName.toStdString();
            std::string file_extension = file_name.substr(file_name.find_last_of('.') + 1);

            if (file_extension == "pcd")
            {
                pcl::PCDReader reader;
                reader.read(file_name, *cloud);

                // 模拟解析过程中的进度更新
                for (int i = 0; i < 100; i++)
                {
                    QThread::msleep(10); // 模拟处理延时
                    QMetaObject::invokeMethod(this, [this, i]() {
                        ui.progressBar->setValue(i + 1);
                        }, Qt::QueuedConnection);
                }
            }
            else if (file_extension == "ply")
            {
                pcl::io::loadPLYFile(file_name, *cloud);

                // 模拟解析过程中的进度更新
                for (int i = 0; i < 100; i++)
                {
                    QThread::msleep(10); // 模拟处理延时
                    QMetaObject::invokeMethod(this, [this, i]() {
                        ui.progressBar->setValue(i + 1);
                        }, Qt::QueuedConnection);
                }
            }
            else if (file_extension == "stl")
            {
                pcl::PolygonMesh mesh;
                pcl::io::loadPolygonFileSTL(file_name, mesh);
                pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

                // 模拟解析过程中的进度更新
                for (int i = 0; i < 100; i++)
                {
                    QThread::msleep(10); // 模拟处理延时
                    QMetaObject::invokeMethod(this, [this, i]() {
                        ui.progressBar->setValue(i + 1);
                        }, Qt::QueuedConnection);
                }
            }

            QMetaObject::invokeMethod(this, [this]() {
                viewer->updatePointCloud(cloud, "cloud");
                viewer->resetCamera();
                viewer->getRenderWindow()->Render(); // 强制渲染
                viewer->setCameraPosition(
                    0, 0, 1,   // 相机位置 (Z轴正方向朝屏幕)
                    0, 0, 0,   // 目标点 (原点)
                    0, 1, 0    // 垂直方向 (Y轴垂直向上)
                );
                viewer->resetCamera();
                ui.openGLWidget->update();
                }, Qt::QueuedConnection);
            });
    }
}

//设置背景颜色
void QtWidgetsApplication3::pushButton_ColorPickerClicked()
{
    QColor color = QColorDialog::getColor(Qt::white, this, tr("Choose Background Color"));

    if (color.isValid())
    {
        double r = color.redF();
        double g = color.greenF();
        double b = color.blueF();

        viewer->setBackgroundColor(r, g, b);
        viewer->getRenderWindow()->Render(); //  更新渲染窗口
    }
}


//设置点云颜色
void QtWidgetsApplication3::pushButton_PointCloudColorPickerClicked()
{
    QColor color = QColorDialog::getColor(Qt::white, this, tr("Choose Point Cloud Color"));

    if (color.isValid())
    {
        double r = color.redF();
        double g = color.greenF();
        double b = color.blueF();

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, r * 255, g * 255, b * 255);
        viewer->updatePointCloud(cloud, color_handler, "cloud");
        viewer->getRenderWindow()->Render(); // 更新渲染窗口
    }
}
