# Commands used to generate this directory

rosrun collada_urdf urdf_to_collada $(rospack find atlas)/atlas.urdf atlas.dae
openrave-robot.py atlas.dae --info links > links.txt
