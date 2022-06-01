<h2 align="center"> :robot: Follow the leader </h2>

O projeto consiste no desafio de <i>Follow the Leader</i>, onde um dos robô é considerado o mestre, sendo controlado pelo <i>eleop</i>, e o outro robô deverá seguí-lo 
no mapa que possui obstáculos. A maneira como o mestre identifica os objetos que estão no mapa é pelo sensor LIDAR, o outro robô irá seguí-lo por conta da biblioteca que possui a técnica de Odometria.
_________________________________________________________________________________________________________________

<h3>:gear: Configurações iniciais </h3>

Foi utilizado o turtlebot3, de modo que foi usado os seguintes recursos dele, inseridos na pasta src:
```
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

<h3>:toolbox: Compilando</h3>

Esse comando foi feito na pasta principal, anterior à src.
```
catkin build
```

<br><br>Foi utilizado três terminais para poder enviar os comandos, que serão indicados por 1), 2) e 3).

<h3>1)Inicializando o master</h3>

```
roscore
```

<h3>Buildando e setando variáveis</h3>

```
source devel/setup.bash
catkin build
```

<h3>Definindo o robô que será utilizado</h3>

```
export TURTLEBOT3_MODEL=burger
```


<h3>2) Mapa Gazebo</h3>

O mapa foi criado utilizando alguns obstáculos, conforme imagem abaixo. 

<img>https://github.com/gabrielapietra/followtheleaderwithLIDAR/blob/master/src/follow_me/world/mapa.png?raw=true</img>

Ele pode ser encontrado pelo caminho <i>src/follow_me/world/new_world.world</i> e para executá-lo é necessário o comando:

```
gazebo new_world.world
```
