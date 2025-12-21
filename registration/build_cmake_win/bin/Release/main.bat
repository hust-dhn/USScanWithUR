::cuda  
::这个要放在tensorrt前面，否则会报如下错误
::[error][simple_yolo.cu:1114]:NVInfer: C:\source\rtSafe\safeRuntime.cpp (32) - Cuda Error in nvinfer1::internal::DefaultAllocator::freE: 1 (invalid argument)
::set PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.1\bin\;%PATH%
::set PATH=E:\Robot\Project\cuda_cudnn_TensorRT\cuda11.1_cudnn\cudnn-11.2-windows-x64-v8.1.0.77\cuda\bin\;%PATH%


::qt
::set PATH=E:\Robot\Project\3rdparty\qt\Qt5.12.12\5.12.12\msvc2017_64\bin\;%PATH%
::set QT_QPA_PLATFORM_PLUGIN_PATH=E:\Robot\Project\3rdparty\qt\Qt5.12.12\5.12.12\msvc2017_64\plugins_5.12.12\


::openNI
::set PATH=E:\Robot\Project\3rdparty\pcl\OpenNI2\Samples\Bin\;%PATH%


::vtk pcl
::set PATH=E:\Robot\Project\3rdparty\pcl\PCL-1.9.0-AllInOne-msvc2017-win64\3rdParty\VTK\bin\;%PATH%
::set PATH=E:\Robot\Project\3rdparty\pcl\PCL-1.9.0-AllInOne-msvc2017-win64\bin\;%PATH%

::set PATH=E:\Robot\Project\3rdparty\pcl\PCL-1.10.0-AllInOne-msvc2019-win64\3rdParty\VTK\bin\;%PATH%
::set PATH=E:\Robot\Project\3rdparty\pcl\PCL-1.10.0-AllInOne-msvc2019-win64\bin\;%PATH%

::set PATH=E:\Robot\Project\3rdparty\vtk\VTK-8.1.2_Debug_qt5.12.12\build_cmake\install\bin\;%PATH%
::set PATH=E:\Robot\Project\3rdparty\pcl\pcl-pcl-1.9.0_Debug_vtk9.4.2\build_cmake\install\bin\;%PATH%

::set PATH=E:\Robot\Project\3rdparty\vtk\VTK-8.2.0_Debug_qt5.12.12\build_cmake\install\bin\;%PATH%
::set PATH=E:\Robot\Project\3rdparty\pcl\pcl-pcl-1.10.0_Debug_vtk9.4.2\build_cmake\install\bin\;%PATH%

::set PATH=E:\Robot\Project\3rdparty\vtk\VTK-9.4.2_Debug_qt5.12.12\build_cmake\install\bin\;%PATH%
::set PATH=E:\Robot\Project\3rdparty\pcl\pcl-pcl-1.15.0_Debug_vtk9.4.2\build_cmake\install\bin\;%PATH%

set PATH=E:\Robot\Project\3rdparty\vtk\VTK-9.4.1_Release_qt5.13.2\build_cmake\install\bin\;%PATH%
set PATH=E:\Robot\Project\3rdparty\pcl\pcl-pcl-1.15.0_Release_vtk9.4.1\build_cmake\install\bin\;%PATH%


::opencv
set PATH=E:\Robot\Project\3rdparty\opencv\opencv-3.4.7_Release\build_cmake\install\x64\vc16\bin\;%PATH%


echo %PATH%


.\main.exe


Pause

