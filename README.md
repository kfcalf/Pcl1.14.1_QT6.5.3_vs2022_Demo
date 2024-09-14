# QtWidgetsApplication3

## 需要安装qt6.5.3+vs2022+Pcl1.14.1_vtk9.3编译 

![图片](https://github.com/user-attachments/assets/4ec4917e-04f6-42c2-a748-c105d1315f1d)

Pcl1.14.1对应的是VTK9.3，编译vtk的时候要选择With_QT

关于编译有几个需要注意的地方：

vtk9.3编译之前必须要修改的文件,不然编译FiltersReduction debug会出错
在 Common/Core/vtkConstantImplicitBackend.h 文件中，替换
struct VTKCOMMONCORE_EXPORT vtkConstantImplicitBackend final
为
struct vtkConstantImplicitBackend final。
（去掉 VTKCOMMONCORE_EXPORT）

install目录设置CMAKE_INSTALL_PREFIX（感觉不设置，默认也行,会在项目下生成一个install目录)

