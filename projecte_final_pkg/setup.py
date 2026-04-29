import os 
from glob import glob 
from setuptools import setup 
package_name = 'projecte_final_pkg' 
setup( 
  name=package_name, 
  version='0.0.1', 
  packages=[package_name], 
  data_files=[ 
    ('share/ament_index/resource_index/packages', 
    ['resource/' + package_name]), 
    ('share/' + package_name, ['package.xml']), 
    # Aquesta línia instal·la els Launch files (es nova) 
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), 
  ], 
  install_requires=['setuptools'], 
  zip_safe=True, 
  maintainer='Josep Rodríguez, Abril Boya i Gael Horowicz', 
  description='Projecte Final IR', 
  license='Apache License 2.0', 
  tests_require=['pytest'], 
  entry_points={ 
    'console_scripts': [ 
      'cartograf_exe = projecte_final_pkg.cartograf:main', 
      'moviment_exe = projecte_final_pkg.moviment:main', 
      'deteccio_exe = projecte_final_pkg.deteccio:main', 
    ], 
  }, 
)
