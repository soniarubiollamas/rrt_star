sigue la primera práctica para instalar el catkin y tal, y hacer lo del bash
de las siguientes prácticas tb tendrás que instalar algo como el rviz y noetic o algo así (las cosas del dron no hace falta), creo que las importantes están en el lab4 y lab5 (y puede que lab2)

para ejecutar los comandos son:
catkin build rrt_star
roslaunch rrt_stat rrt-star-navigation.launch

si al ejecuta rel roslaunch te da error de que no existe, ejecuta primero esto:
source ~/catkin_ws/devel/setup.bash
