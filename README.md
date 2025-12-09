## KF-GINS-Matlab

[[中]](./README_CN.md) &ensp; [[EN]](./README.md)

## An EKF-based GNSS/INS Integrated Navigation Systems in Matlab (Matlab Version of KF-GINS)

### Introduction

As a classic loosely coupled GNSS/INS integrated navigation algorithm, [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS) is suitable for beginners to learn and understand integrated navigation. However, the C++ implementation of KF-GINS still makes debugging, modification, and extension challenging for newcomers. Therefore, we open-source a Matlab-based integrated navigation software, KF-GINS-Matlab, which serves as the Matlab version of KF-GINS. This version is more suited for teaching, experiments, and beginners. In addition to replicating KF-GINS in Matlab, KF-GINS-Matlab incorporates a GNSS velocity update algorithm and a framework for ODO/NHC updates (Note: the ODO/NHC measurement update is not fully implemented, and ODO scale factor error is not augmented. You are encouraged to complete the algorithms based on the documentation as a learning exercise).

**Origanization:** [Integrated and Intelligent Navigation (i2Nav) Group](http://www.i2nav.com/), GNSS Research Center, Wuhan University.

**Related Reference:**

- X. Niu, L. Wang, Q. Chen, H. Tang, Q. Zhang, and T. Zhang, “KF-GINS: an open-sourced software for GNSS/INS integrated navigation,” GPS Solut, vol. 29, no. 4, p. 202, Oct. 2025, doi: 10.1007/s10291-025-01967-w.
- 牛小骥, 陈起金, 张全, "惯性导航原理与算法设计", 高等学校导航工程专业规划教材. 武汉: 武汉大学出版社, 2025.
- 牛小骥, 陈起金, "[惯性导航原理与GNSS/INS组合导航课程讲义](http://www.i2nav.com/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=40f3c65b158742c099ba3b600c983aa1)", 武汉大学多源智能导航实验室, 2022.
- 牛小骥, 陈起金, "[惯性导航原理与GNSS/INS组合导航课程视频](https://www.bilibili.com/video/BV1na411Z7rQ?spm_id_from=333.999.0.0&vd_source=a417ebe0768fc96919fe8e34c55ed591)", 武汉大学多源智能导航实验室, 2022.
- X. Niu, Q. Zhang, L. Gong, C. Liu, H. Zhang, C. Shi, J. Wang and M. Coleman (2014). "Development and evaluation of GNSS/INS data processing software for position and orientation systems." Survey Review 2014; 47(341), 87-98.
- 严恭敏, 翁浚, 捷联惯导算法与组合导航原理. 西北工业大学出版社, 2019.
- E.-H. Shin, "Estimation techniques for low-cost inertial navigation," Ph.D. dissertation, Dept. Geomatics Eng., Univ. of Calary, AB, Cabada, 2005
- [Savage, P.G.](http://www.strapdownassociates.com/), "Strapdown analytics, Part I", Maple Plain, MN: Strapdown Associates, 2000.
- Titterton David, JohnL. Weston, and John Weston, "Strapdown inertial navigation technology", The Institution of Electrical Engineers, 2004.
- P. D. Groves, "Principles of GNSS, Inertial, and Multi-sensor Integrated Navigation Systems", 2nd ed., vol. 39. Artech House, 2013.

**If you use this software for your academic research, please give acknowledgment as follows and cite our [related document and papers](./ref.bib)**

```
English version: “The authors would like to acknowledge the team of Prof. Xiaoji Niu of the Integrated and Intelligent Navigation (i2Nav) group from GNSS Research Center of Wuhan University for providing the open-source KF-GINS-Matlab software that was used in the paper.”
中文模板：“本文作者感谢武汉大学卫星导航定位技术研究中心多源智能导航实验室(i2Nav)牛小骥教授团队开源的KF-GINS-Matlab软件平台。”
```

**Contacts:**

- For any technique problem, you can send an email to Liqiang Wang (wlq@whu.edu.cn).
- For Chinese users, we also provide a QQ group (481173293) for discussion. You are requested to provide your organization and name to join the QQ group.

## 1.KF-GINS-Maltab Files Overview

| Filename                      | Description                                                        |
| ----------------------------- | ------------------------------------------------------------------ |
| kf_gins.m                     | Entry point; main loop of the algorithm                            |
| Initialize.m                  | Program initialization                                             |
| InsMech.m                     | Strapdown inertial navigation mechanization algorithm              |
| InsPropagate.m                | State vector prediction and covariance propagation                 |
| GNSSUpdate.m                  | GNSS position/velocity observation update                          |
| GetOdoVel.m                   | Processes ODO raw data to get velocity results                     |
| ODONHCUpdate.m                | ODO/NHC velocity update function (not fully implemented)           |
| ErrorFeedback.m               | Error feedback                                                     |
| ProcessConfig.m               | Configuration of program parameters                                |
| ProcessConfig1.m              | Configuration for GNSS position/INS test data                      |
| ProcessConfig2.m              | Configuration for GNSS velocity/INS test data                      |
| ProcessConfig3.m              | Configuration for ODO/NHC test data                                |
| function/Param.m              | Geographical parameter handling                                    |
| function/*.m                  | Libraries for rotation parameters and geographical transformations |
| plot-function/plot_result.m   | Plots navigation results                                           |
| plot-function/plot_std.m      | Plots estimated navigation state STD                               |
| plot-function/plot_imuerror.m | Plots estimated IMU errors                                         |
| plot-function/calc_error.m    | Calculates and plots positioning errors                            |
| dataset1                      | GNSS position/INS test data                                        |
| dataset2                      | GNSS velocity/INS test data                                        |
| dataset3                      | ODO/NHC test data                                                  |

## 2. Run KF-GINS-Matlab

### 2.1 Environment and Download KF-GINS-Matlab

KF-GINS-Matlab runs in Matlab without additional dependencies, requiring no specific system environment or Matlab version.

To download, use the following git command or download the zip package from GitHub and extract it.

```
git clone https://github.com/i2Nav-WHU/KF-GINS-Matlab.git ~/
```

### 2.2  Run KF-GINS-Matlab

1）Open the KF-GINS-Matlab folder in Matlab.[[中]](./README_CN.md) &ensp; [[EN]](./README.md)

2）Double-click the `kf-gins.m` file to open it in the editor. Click "Run" in the editor or type `kf-gins` in the command window (default: `ProcessConfig1.m`).

3）Wait for the program to finish running.

### 2.3 Display Results

1）Add the `plot-function` folder to the workspace: right-click the folder and select "Add to Path" -> "Selected Folders and Subfolders."

2）Expand the `plot-function` folder, and execute `plot_result.m` to plot navigation results or `calc_error.m` to plot positioning errors.

### 2.4 Notes

For the ODO/NHC update framework, set the ODO update time according to the preset update frequency. At each update, use the `GetOdoVel` function to obtain ODO velocities from raw data.

In `GetOdoVel.m`, average ODO data from 10 epochs before and after the update time to reduce quantization noise.

### 2.5 Run Other Test Data

1）Modify the `cfg` value in `kf-gins.m` to run other test datasets. Update file paths in plotting scripts as needed.

2）For custom test data, define a new `ProcessConfig.m` file and modify file paths and configurations accordingly.

## 3 Test Data

### 3.1 Provided Data

KF-GINS-Matlab includes three test datasets with corresponding parameter configuration files in the main folder. dataset1 is the GNSS position/INS test data (i.e. the test data in KF-GINS); dataset2 is the GNSS velocity/INS test data; dataset3 is the ODO/NHC test data. The datasets are as follows:

| Dataset  | Filename        | Description                 |
| -------- | --------------- | --------------------------- |
| dataset1 | Leador-A15.txt  | IMU text file               |
|          | GNSS-RTK.txt    | GNSS position file          |
|          | truth.nav       | Reference truth file        |
| dataset2 | ADIS16465.txt   | IMU text file               |
|          | GNSS-POSVEL.txt | GNSS position/velocity file |
|          | truth.nav       | Reference truth file        |
| dataset3 | ADIS16465.txt   | IMU text file               |
|          | GNSS-POS.txt    | GNSS position file          |
|          | ODO.txt         | ODO velocity file           |
|          | truth.nav       | Reference truth file        |

### 3.2 Data Format

For the formats of IMU files, GNSS position files, reference truth files, generated results, IMU error, and STD files, refer to `KF-GINS/README.md`. The formats for GNSS position/velocity files and ODO files are as follows:

- GNSS position/velocity file format:

| Column | Description                    | Unit    |
| ------ | ------------------------------ | ------- |
| 1      | GNSS time of week              | $s$   |
| 2      | Latitude                       | $rad$ |
| 3      | Longitude                      | $rad$ |
| 4      | Ellipsoidal height             | $m$   |
| 5~7    | Position STD (North-East-Down) | $m$   |
| 8~10   | Velocity (North-East-Down)     | $m/s$ |
| 11~13  | Velocity STD (North-East-Down) | $m/s$ |

- ODO file format:

| Column | Description       | Unit    |
| ------ | ----------------- | ------- |
| 1      | GNSS time of week | $s$   |
| 2      | ODO velocity      | $m/s$ |

## 4 License

KF-GINS-Matlab is released under the GPLv3 license.

For commercial usage, please contact Prof. Xiaoji Niu (xjniu@whu.edu.cn).
