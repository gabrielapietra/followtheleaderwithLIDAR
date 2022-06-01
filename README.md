<h2 align="center"> Follow the leader </h2>

O projeto consiste no desafio de "Seguir o mestre", onde um dos robô é considerado o mestre, sendo controlado pelo teleop, e o outro robô deverá seguí-lo 
no mapa que possui obstáculos. A maneira como o seguidor identifica o mestre é pelo sensor LIDAR.

<h3>Mapa Gazebo</h3>

O mapa foi criado utilizando alguns obstáculos, conforme imagem abaixo. 

Ele pode ser encontrado pelo caminho <i>src/follow_me/world/new_world.world</i> e para executá-lo é necessário o comando:

```
gazebo new_world.world
```
