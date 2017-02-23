# Panoramix
Code of the paper: 
<cite>
Hao Yang and Hui Zhang. "Efficient 3D Room Shape Recovery From a Single Panorama." Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition. 2016.
</cite>

## Build

### Requirements
* Only Visual Studio 2015 is supported currently.
* OpenCV is installed;
* Qt5 is installed;
* MATLAB is installed;
* CVX is installed;
* The [MATLABTools](https://github.com/YANG-H/MATLABTools) repo is downloaded, the CVX path is set in file `startup.m`;
* CMake is installed.

### Build with CMake
* Set `OpenCV_DIR` to the path of OpenCV;
* Set `Qt_DIR` to the path of Qt;
* Set `MATLAB_CODE_DIR` to the path of MATLABTools;
* Build.

### Set the Environment Variables
* Add Qt/OpenCV/MATLAB bin paths to `PATH`;
* Set `QT_QPA_PLATFORM_PLUGIN_PATH=$(Qt_DIR)/plugins/platforms`.

### Citation
```
@inproceedings{YangHao_CVPR2016_3DRoom,
  title={Efficient 3D Room Shape Recovery from a Single Panorama},
  author={Yang, Hao and Zhang, Hui},
  booktitle={Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition},
  pages={5422--5430},
  year={2016}
}
```
