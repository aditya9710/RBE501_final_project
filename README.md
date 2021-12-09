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

4. Visualize the model in AMBF.

For simplified model:
```bash
cd <ambf_path>/bin/<your OS>
./ambf_simulator -a <this_repository_path>/models/psm1_modify/default.yaml
```

For full model:
```bash
cd <ambf_path>/bin/<your OS>
./ambf_simulator -a <this_repository_path>/models/psm_full/default.yaml
```

4. Go to `<this_repository_path>/RBE501_project/scripts`

6. Run the python script for test connection
```bash
python3 ambf_connect_test.py
```


### How to see the specific model

1. You need to download `Blender 2.8x`

2. You need to load the add on from `ambf_addon` folder into Blender.