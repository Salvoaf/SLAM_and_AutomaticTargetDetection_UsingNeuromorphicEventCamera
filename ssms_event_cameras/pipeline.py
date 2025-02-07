

import os
import subprocess

commands = [
    #   "python3 RVT/utils/bat_to_datH5.py",
    "sudo rm -rf RVT/gen1_frequenciesgen1_200hz/",
    "python preprocess_dataset.py /home/salvatore/ssms_event_cameras/dataset /home/salvatore/ssms_event_cameras/RVT/gen1_frequenciesgen1_200hz/ conf_preprocess/representation/stacked_hist.yaml conf_preprocess/extraction/const_duration.yaml conf_preprocess/filter_gen1.yaml -ds gen1 -np 20",
    "sudo mv RVT/gen1_frequenciesgen1_200hz/val/* RVT/gen1_frequenciesgen1_200hz/test",
    "sudo mv RVT/gen1_frequenciesgen1_200hz/test/dynamic_translation/event_representations_v2/* RVT/gen1_frequenciesgen1_200hz/test/dynamic_translation/event_representations_v2/stacked_histogram_dt=50_nbins=10",
    "sudo rm -rf poses/*",
    "python RVT/validation.py dataset=gen1 dataset.path=\"/home/salvatore/ssms_event_cameras/RVT/gen1_frequenciesgen1_200hz\" checkpoint=\"/home/salvatore/ssms_event_cameras/checkpoint/gen1_small.ckpt\" use_test_set=1 hardware.gpus=0 +experiment/gen1=\"small.yaml\" batch_size.eval=1 model.postprocess.confidence_threshold=0.01 hardware.num_workers.eval=1",
    #"python3 RVT/utils/csv_to_npy.py",
    #"sudo rm -rf RVT/gen1_frequenciesgen1_200hz/",
    #"python preprocess_dataset.py /home/salvatore/ssms_event_cameras/dataset /home/salvatore/ssms_event_cameras/RVT/gen1_frequenciesgen1_200hz/ conf_preprocess/representation/stacked_hist.yaml conf_preprocess/extraction/const_duration.yaml conf_preprocess/filter_gen1.yaml -ds gen1 -np 20",
    #"sudo rm -rf result",
    #"sudo python3 viz_gt.py"
    "bash -c 'source /home/salvatore/uslam_ws/devel/setup.bash'",
    "roslaunch ze_vio_ceres live_DAVIS240C_events_only.launch camera_name:=DAVIS-IJRR17",
    "rosbag play /home/salvatore/uslam_ws/src/rpg_ultimate_slam_open/data/dynamic_translation.bag"
]
cmd = ["sudo rm -rf RVT/gen1_frequenciesgen1_200hz/", 
    "python preprocess_dataset.py /home/salvatore/ssms_event_cameras/dataset /home/salvatore/ssms_event_cameras/RVT/gen1_frequenciesgen1_200hz/ conf_preprocess/representation/stacked_hist.yaml conf_preprocess/extraction/const_duration.yaml conf_preprocess/filter_gen1.yaml -ds gen1 -np 20",
    "python RVT/validation.py dataset=gen1 dataset.path=\"/home/salvatore/ssms_event_cameras/RVT/gen1_frequenciesgen1_200hz\" checkpoint=\"/home/salvatore/ssms_event_cameras/checkpoint/gen1_small.ckpt\" use_test_set=1 hardware.gpus=0 +experiment/gen1=\"small.yaml\" batch_size.eval=1 model.postprocess.confidence_threshold=0.01 hardware.num_workers.eval=1"
    ]
process = subprocess.run(cmd[0], shell=True)
base_dir = "/home/salvatore/ssms_event_cameras/RVT/scripts/genx/"
os.chdir(base_dir)
process = subprocess.run(cmd[1], shell=True)
process = subprocess.run(cmd[2], shell=True)

process = subprocess.run(cmd, shell=True)
l = 1
for i, cmd in enumerate(commands):
    if i == 2-l or i == 9-l:
        base_dir = "/home/salvatore/ssms_event_cameras/RVT/scripts/genx/"
    elif i == 11-l:
        base_dir = "/home/salvatore/ssms_event_cameras/RVT/scripts/viz/"    
    else:
        base_dir = "/home/salvatore/ssms_event_cameras"
    os.chdir(base_dir)  
    print("Directory corrente:", os.getcwd())
    process = subprocess.run(cmd, shell=True)
    if process.returncode != 0:
        print(f"Command failed: {cmd}")
        break

