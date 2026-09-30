from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    checkTerrainConn_arg = DeclareLaunchArgument(
        'checkTerrainConn',
        default_value='false',
        description='Check terrain connectivity'
    )

    terrain_analysis_ext_node = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        name='terrainAnalysisExt',
        output='screen',
        parameters=[{
            'scanVoxelSize': 0.1,
            'decayTime': 0.5,
            'noDecayDis': 0.0,
            'clearingDis': 30.0,
            'useSorting': True,
            'quantileZ': 0.1,
            'vehicleHeight': 1.5,
            'voxelPointUpdateThre': 60,
            'voxelTimeUpdateThre': 0.5,
            'lowerBoundZ': -50.0,
            'upperBoundZ': 50.0,
            'disRatioZ': 0.1,
            'checkTerrainConn': LaunchConfiguration('checkTerrainConn'),
            'terrainConnThre': 0.5,
            'terrainUnderVehicle': -0.75,
            'ceilingFilteringThre': 2.0,
            'localTerrainMapRadius': 4.0
        }]
    )

    return LaunchDescription([
        checkTerrainConn_arg,
        terrain_analysis_ext_node
    ])