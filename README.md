# QtWidgetsApplication3

## 需要安装qt6.5.3+vs2022+Pcl1.14.1_vtk9.3编译 

![图片](https://github.com/user-attachments/assets/4ec4917e-04f6-42c2-a748-c105d1315f1d)

Pcl1.14.1对应的是VTK9.3，编译vtk的时候要选择With_QT

关于编译有几个需要注意的地方：

1.vtk9.3编译之前必须要修改的文件,不然编译FiltersReduction debug会出错
在 Common/Core/vtkConstantImplicitBackend.h 文件中，替换
struct VTKCOMMONCORE_EXPORT vtkConstantImplicitBackend final
为
struct vtkConstantImplicitBackend final。
（去掉 VTKCOMMONCORE_EXPORT）![图片](https://github.com/user-attachments/assets/2ceab3db-aef4-42bf-90f5-0c9b7829e853)


2.install目录设置CMAKE_INSTALL_PREFIX（感觉不设置，默认也行,会在项目下生成一个install目录)
![图片](https://github.com/user-attachments/assets/38ff47b8-a830-4cd3-aecd-5ced83a0267d)

3.Testing和example不要选择，不然编译时间会很长，除非可以等
![图片](https://github.com/user-attachments/assets/3d1d0b5f-18c7-4c46-b9e1-730899000d83)

4.添加一个变量
勾选Cmake的Grouped和Advanced,再点击【Add Entry】按钮添加缓存变量 CMAKE_DEBUG_POSTFIX，类型为 STRING，值设置为 -gd。注意-gd前后不要有空格
设置CMAKE_INSTALL_PREFIX，用于指定我们的输出目录，这个地方就设置为我们创建的install目录
![图片](https://github.com/user-attachments/assets/260de206-7858-4a89-87ac-3130bde4b23e)

5.QT相关设置，看自己的qt位置，我用的是qt6.5.3,编译好后是否适用其它版本的qt例如qt6.7这个没有验证过
![图片](https://github.com/user-attachments/assets/6c9d808b-deed-4c6a-aa02-e7f066c01115)



