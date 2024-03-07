# PRÁCTICA DE GAZEBO
Andrés Gracia Guillén


## Plugin "my_wheels":
Navegación del robot siguiendo la trayectoria óptima (línea recta más corta) desde el punto inicial hasta el punto objetivo y evitando los obstáculos a su paso. El robot no conoce el mapa (navegación reactiva).

## Extras:
1. Se ha implementado en el código un cronómetro interno del algoritmo para conocer de forma adecuada el tiempo total de simulación.
2. Se ha probado el plugin en diversos mapas.
3. Todo el trabajo está subido a GitHub accesible a través del siguiente enlace:
   https://github.com/TheAndrenator/Gazebo-simulation-with-Pioneer-robot.git
4. Se ha grabado una demo para cada mapa que se encuentran subidas a YouTube.

## Enlaces a las demos
Map 01: https://youtu.be/pVIRoC8IoFs
Map 02: https://youtu.be/4EXPUBPAt-8
Map 03: https://youtu.be/tQrX_MZRQKg

## Nota
Para ejecutar la simulación, cargar el plugin "my_wheels.cc" y el mapa deseado en formato .xml encontrado en las respectivas carpetas de mapas:
  GAZEBO_PLUGIN_PATH=/ruta/a/plugin/build gazebo --verbose --pause map.world.xml
