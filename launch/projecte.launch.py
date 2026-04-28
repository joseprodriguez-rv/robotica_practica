from launch import LaunchDescription 
from launch_ros.actions import Node 
def generate_launch_description(): 
  return LaunchDescription([ 
    Node( 
      package='projecte_final_pkg', 
      executable='deteccio_exe', # El nom que posarem al setup.py 
      name='deteccio' 
    ), 
    Node( 
      package='projecte_final_pkg', 
      executable='moviment_exe', # El nom que posarem al setup.py 
      name='moviment' 
    ), 
    Node( 
      package='projecte_final_pkg', 
      executable='cartograf_exe', # El nom que posarem al setup.py 
      name='cartograf' 
    ), 
    Node( 
      package='projecte_final_pkg', 
      executable='safety_node_exe', # El nom que posarem al setup.py 
      name='safety' 
    ), 
  ]) 
