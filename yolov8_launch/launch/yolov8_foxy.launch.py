from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description() :

    weight = LaunchConfiguration('weight')
    device = LaunchConfiguration('device')
    conf_threshold = LaunchConfiguration('conf_threshold')
    iou_threshold = LaunchConfiguration('iou_threshold')
    image_reliability = LaunchConfiguration('image_reliability')
    img_topic = LaunchConfiguration('img_topic')
    namespace = LaunchConfiguration('namespace')    

    return LaunchDescription([
        DeclareLaunchArgument(
            'weight',
            default_value='yolov8n.pt',
            description='weight model path or name'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='Device type(GPU|CPU)'
        ),
        DeclareLaunchArgument(
            'conf_threshold',
            default_value='0.5',
            description='NMS confidence threshold '    
        ),
        DeclareLaunchArgument(
            'iou_threshold',
            default_value='0.7',
            description='NMS IoU threshold'
        ),
        DeclareLaunchArgument(
            'image_reliability',
            default_value='2',
            choices=['0', '1', '2'],
            description='image reliability QOS [systemdefault|reliable|best effort]'
        ),
        DeclareLaunchArgument(
            'img_topic',
            default_value='/camera/rgb/image_raw',
            description='image topic name'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='yolov8',
            description='Namespace of the node'
        ),
        Node(
            package='yolov8_main',  
            executable='yolov8_node',  
            name='yolov8_node',
            parameters=[
                {'weight': weight},
                {'device': device},
                {'conf_threshold': conf_threshold},  
                {'iou_threshold': iou_threshold},  
                {'image_reliability': image_reliability}
            ],
            remappings=[('/image_raw', img_topic)] 
        )
    ])