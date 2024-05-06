# LIDAR SLAM ARVC

## OBJETO
El paquete LIDAR SLAM ARVC tiene la finalidad de realizar mapas offline usando datos de LIDAR.

## JUSTIFICACIÓN
Resulta, en ocasiones, complejo, utilizar los paquetes software de terceros. Son muchas las razones que
justifican esto:
 - La construcción del robot y la ubicación de sus sensores.
 - La sincronización de los sensores.


## DESCRIPCIÓN
El paquete permite la construcción de mapas offline. El backbone de los mapas emplea la librería gtSAM para la construcción.
Como frontend se utilizan unos scripts de python. Damos, a continuación, una guía de uso del software resumida en los siguientes pasos:
* 1- Captura de datos con un robot real o Gazebo.
* 2- Extracción de los datos de Rosbag a formato EUROC/ASL.
* 3- Cálculo de una odometría mejorada con scanmatching.
* 4- Cálculo del mapa con gtSAM
* 5- Visualización del mapa.

#### 1 CAPTURA DE DATOS
El proceso se inicia mediante la captura de un fichero ROSBAG mediante
un robot real o durante una sesión de Gazebo.

#### 2 EXTRACCIÓN DE LOS DATOS A continuación, la información necesaria para el mapa se debe extraer a un formato EUROC/ASL. Para ello se debe/puede utilizar
Se debe utilizar el paquete extract_rosbag (https://github.com/ARVCUMH/extract_rosbag.git) para extraer los datos.
Los datos del fichero Rosbag se escriben en un directorio donde son fáciles de leer.

#### 3 CÁLCULO DE UNA ODOMETRÍA MEJORADA CON EL SCANMATCHER
Se debe ejecutar el fichero run_scanmatcher.py.
Este fichero se puede ejecutar directamente desde cualquier IDE para python (e. g. Pycharm). 

La configuración se debe especificar directamente en la función entrada de datos se espec
scanmatcher(). Se deben modificar los siguientes parámetros.

```
    # Directorio de entrada de los datos
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O2-2024-03-07-13-33-34'
    # caution, this is needed to remove initial LiDAR scans with no other data associated to it
    start_index = 0
    # sample LiDAR scans with delta_time in seconds
    delta_time = 0.3
    # voxel size: pointclouds will be filtered with this voxel size
    voxel_size = None
    # Sanmatcher method
    # method = 'icppointpoint' :ICP con distancia punto a punto
    # method = 'icppointplane' : ICP con distancia punto a plano
    method = 'icp2planes' : ICP con distancia 
    # method = 'fpfh'
``