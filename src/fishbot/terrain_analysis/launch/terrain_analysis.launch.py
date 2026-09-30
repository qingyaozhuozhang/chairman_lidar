from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[{
                'scanVoxelSize': 0.05,
                'decayTime': 0.2,
                'noDecayDis': 0.0,
                'clearingDis': 30.0,
                'useSorting': True,
                'quantileZ': 0.25,
                'considerDrop': False,
                'limitGroundLift': False,
                'maxGroundLift': 0.15,
                'clearDyObs': True,
                'minDyObsDis': 0.3,
                'minDyObsAngle': 0.0,
                'minDyObsRelZ': -0.3,
                'absDyObsRelZThre': 0.2,
                'minDyObsVFOV': -50.0,
                'maxDyObsVFOV': 50.0,
                'minDyObsPointNum': 1,
                'noDataObstacle': False,
                'noDataBlockSkipNum': 0,
                'minBlockPointNum': 10,
                'vehicleHeight': 100.0,
                'voxelPointUpdateThre': 60,
                'voxelTimeUpdateThre': 0.3,
                'minRelZ': -100.0,
                'maxRelZ': 100.0,
                'disRatioZ': 0.2
            }]
        )
    ])