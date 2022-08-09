import argparse
import os

import threading, time

from requests import options
from LidarProcess import rpScanReceiver
from LidarMQTT import LidarMQTT
import multiprocessing as mp
import roslaunch
import subprocess
import rospy

DIR = os.path.dirname(os.path.realpath(__file__))
LAUNCHDIR = '/home/ves/ws_livox/src/livox_ros_driver/launch/'
# ROOT = FILE.parents[0]  # YOLOv5 root directory
# if str(ROOT) not in sys.path:
#     sys.path.append(str(ROOT))  # add ROOT to PATH
# ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

def LidarProcess():
    option = parse_opt()
    rpScanReceiver(option)
    
def MQTTProcess():
    LidarMQTT()
    
def LidarLaunch():
    os.chdir("/home/ves/ws_livox")
    #os.system("ls")
    option = parse_opt()
    publish_freq = 'publish_freq:=' + str(option.freq)
    com = 'source ./devel/setup.sh' + '&&' + 'roslaunch livox_ros_driver livox_lidar.launch bd_list:="3WEDK3V0015B191" ' + publish_freq
    print(com)
    print(" ----------------------------- " + publish_freq + "Hz ----------------------------- ")
    subProc = subprocess.Popen(["/bin/bash", "-i", "-c", com])
    subProc.communicate()

    
    #os.system("source ./setup.sh")
    #os.system("roslaunch livox_ros_driver livox_lidar.launch bd_list:=“3WEDK3V0015B191”")
    # option = parse_opt()
    # rospy.init_node('test', anonymous=True)
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # publish_freq = 'publish_freq:=' + str(option.freq)
    # print(" ----------------------------- " + publish_freq + "Hz ----------------------------- ")
    # cli_args1 = ['livox_ros_driver', 'livox_lidar.launch', publish_freq, 'bd_list:="3WEDK3V0015B191"']
    # roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
    # roslaunch_args1 = cli_args1[2:]


    #launch_files = [(roslaunch_file1, roslaunch_args1)]

    #parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

    #parent.start()
 
 
def run(freq, mqtt, process, launch, downsampling, launch_dir, test_dir, mqtt_dir,):
    """라이다 처리 및 통신 과정 실행 프로세스
    Args:
        launch        (str): 센서 데이터 수집 및 ros topic broadcast 수행 launch 파일 실행
        process (str): 데이터 처리 process 수행 python 모듈 실행
        mqtt   (str): 처리 데이터를 API 서버에 업로드하기 위한 MQTT publish 모듈 실행
        downsampling, ...,    : argument parser에 의해 얻은 실행 변수 및 설정값
    Examples:
        >>> run(1, 'config.yaml', 'occupancy', opt)
    Raises:
        RecursionError, AssertionError: 카메라 접속이 불가하거나 원활하지 않은 경우 발생
    """
    
    # MQTT Client
    # logins = login_mqtt()
    # topic_class = 'log'
    # service_mqtt_client = mqtt_data_client(logins, topic_class)
    # parkingLot = "skv1"
    # parkingFloor = "b3"
    # service_topic = "wlogs/device/service/" + logins.clientID + "/" + topic_class + "/video/" + parkingLot + "/" + parkingFloor
    # start = time.time()

    options = parse_opt()
    
    if launch:
        p1 = mp.Process(name="LidarLaunch", target=LidarLaunch)
        p1.start()
    if process:
        #process = threading.Thread(target=LidarThread, args=(), daemon=True)
        #process.start()
        p2 = mp.Process(name="LidarProcess", target=LidarProcess)
        p2.start()
    if mqtt:
        p3 = mp.Process(name="MQTTProcess", target=MQTTProcess)
        p3.start()

def parse_opt():
    """프로세스 실행 매개변수 목록
        Arguments:
            launch        (str): 센서 데이터 수집 및 ros topic broadcast 수행 launch 파일 실행
            process (str): 데이터 처리 process 수행 python 모듈 실행
            mqtt   (str): 처리 데이터를 API 서버에 업로드하기 위한 MQTT publish 모듈 실행
            downsampling, ...,    : argument parser에 의해 얻은 실행 변수 및 설정값
        Returns:
            namespace: 프로세스 실행 매개변수 및 설정값
        Examples:
            >>> python3 demon.py --downsampling 0.3 --mqtt-log Ture
    """
    parser = argparse.ArgumentParser()
    #parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path(s)')
    #parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob, 0 for webcam')
    #parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    #parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    #parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    #parser.add_argument('--view-img', action='store_true', help='show results')
    #parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    #parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    # ---------------------------------------------- watchmile ---------------------------------------------------------#

    parser.add_argument('--freq', type=int, default=10, help='Set the frequency of point cloud publish, recommended 5.0, 10.0, 20.0, 50.0, etc.')
    parser.add_argument('--mqtt', default=False, action='store_true', help='MQTT ON/OFF')
    parser.add_argument('--process', default=False, action='store_true', help='Lidar processing ON/OFF')
    parser.add_argument('--launch', default=False, action='store_true', help='launch ON/OFF')
    parser.add_argument('--downsampling', type=float, default=0.2, help='downsampling pointCloud data')
    parser.add_argument('--launch-dir', type=str, default=LAUNCHDIR, help='/home/ves/ws_livox/src/livox_ros_driver/launch/')
    parser.add_argument('--test-dir', type=str, default=DIR, help='/home/ves/ws_livox/src/livox_ros_driver/scripts/')
    parser.add_argument('--mqtt-dir', type=str, default=DIR, help='/home/ves/ws_livox/src/livox_ros_driver/scripts/')
    # parser.add_argument('--xmin', type=float, default=0, help='Set the min value of ROI')
    # parser.add_argument('--ymin', type=float, default=0, help='Set the min value of ROI')
    # parser.add_argument('--zmin', type=float, default=0, help='Set the min value of ROI')
    # parser.add_argument('--xmax', type=float, default=0, help='Set the min value of ROI')
    # parser.add_argument('--ymax', type=float, default=0, help='Set the min value of ROI')
    # parser.add_argument('--zmax', type=float, default=0, help='Set the min value of ROI')

    opt = parser.parse_args()
    #opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    return opt

if __name__ == "__main__":
    opt = parse_opt()
    
    run(**vars(opt))

