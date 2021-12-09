# RBE501_final_project

### How to Run

0. This system is built using ROS melodic and Ubuntu 18.04.

1. Please install AMBF in advance and make sure it is sourced correctly. Follow the instruction in this link/repo:

https://github.com/WPI-AIM/ambf

2. Clone the repository from:

https://github.com/JackHaoyingZhou/RBE501_final_project

First time cloning:
```bash
git clone --recursive https://github.com/JackHaoyingZhou/RBE501_final_project.git
```

3. Please double check your python packages. You may add corresponding if necessary. 

4. 

4. Go to `<your_repository_path>`

6. Run the python script for downloading data
```bash
python3 download_data.py
```

7. Run the python shell script for training
```bash
python3 RBE_595_train.py
```
The corresponding results will be stored in `<your_repository_path>\results\train`


8. Run the python shell script for testing. This script will detect the needles from the images in  `<downloaded_data_folder>/test/images`
```bash
python3 RBE_595_detect.py
```
The corresponding results will be stored in `<your_repository_path>\results\detect`

9. Run the python shell script for testing. This script will detect the needles from the videos in  `<your_repository_path>/video_for_test`
```bash
python3 RBE_595_detect_video.py
```
The corresponding results will be stored in `<your_repository_path>\results\detect_video`

** When runing the script, it may have some errors which can be ignored. Just ignore those issues (mainly because python does not quit regularly). Otherwise, check above steps and python packages.



### How to find results

Just in case that you cannot or do not want to run the codes. Some results have been pre-stored in the repository.

Please check `<your_repository_path>\results`

For data processing, here are some samples in `<your_repository_path>\data_processing`



### Bonus: How to run the surgical platform for data collection

1. Regrading the README file of `<your_repository_path>/surgical_platform`

2. To run the script, you need to install ROS:
http://wiki.ros.org/melodic/Installation/Ubuntu

3. Please install `AMBF (Asynchronous Multi-Body Framework)` based on the instruction in the README file of `<your_repository_path>/surgical_platform`

4. The platform enables us to move forward for more complicated tasks.

5. Now run AMBF with the launch file and ADFs from this repo as:
```bash
cd <AMBF_repository_path>/bin/<your_OS>
./ambf_simulator --launch_file <your_repository_path>/surgical_platform/launch.yaml -l 14,15
```
This is an example of what the scene should look like (only have the needle and the phantom, more instructments can be added later):

<p align="center">
<img src=surgical_platform/Media/scene_sample.png width="480"/>
</p>

6. Run the python script in `<your_repository_path>\data_collection` to collect data
```bash
cd <your_repository_path>\data_collection
python2 camera_test.py
```
Press Enter on your keyboard, corresponding images will be stored to `<your_repository_path>\data_collection\task1_data`