U
    �,b  �                (   @   s  d dl Z d dlmZ d dlmZ d dlmZ d gd adZddd	d
ddddddddddddddddddddddddd d!d"d#dd$d%d&d'd	d(d)g(Z	d a
d ad ag g g gae� ad ad*d+� Zd,d-� Zed.k�re �d/� e jd0ed1d2�Zd3d4d5d6d7d8d9d:d;d<d=d>d?d@dAdBdCdDdEdFdGdHdIdJdKdLdMdNdOdPdQdRdSdTdUdVdWdXdYdZg(td < d[d\d]d1d]d^d\d_d^d[d^d\d1dd`d_d[d\d[d_d_d^d1d^d]d1d\d]ddd_d]dd`d1d`d`d[d`dg(td1< dadadbdadadadadbdbdadbdadbdbdadbdadadadadadadadadbdbdadbdbdbdbdbdadadbdbdbdbdbdbg(tdc< e �e �dd�e�ae �� �se ��  �q�dS )e�    N)�
ModelState)�SetModelState)�UInt8�   �
   g{�G�z�?g{�G�z�?g�������?g������ٿg      пg�G�z�g��(\��տg�G�z��?g333333�?g=
ףp=�?gR���Q�?gq=
ףp�?g�������g��(\���?g{�G�z�g�������?g��(\���?g���(\�ҿg��Q���?g��������gffffffֿg���Q��?g��Q���g������ܿg333333�?g��Q���?g�G�z��?g333333ӿg�p=
ף�?gq=
ףpͿg�������g�p=
ף��g���Q��?g�Q����?g��(\����c              
   C   s�   |dkr$dt dt � t_td7 an|dkrBdt t� t_td7 ad|d d  tt  tjj_	d| d d  tjj_
d	tjj_t�d
� zt�d
t�}|t� W n0 tjk
r� } ztt |�� W 5 d }~X Y nX td7 at�| � d S )N�blueZbox_�   �   �red�   r   �   g�A`��"�?z/gazebo/set_model_state)�str�total_blue_count�	state_msgZ
model_name�total_red_count�rand�boxes_spawnedZposeZposition�x�y�z�rospyZwait_for_serviceZServiceProxyr   ZServiceException�print�info_pubZpublish)�rowZbox_number_in_rowZcolorZ	set_state�e� r   �spawning_final.py�	spawn_box   s"    


r   c                 C   s�   t dk r�| jjtd t  kr�ttd t  d   d7  < ttd t  d  tk r~ttd t  ttd t  d  td t  � ntd� t d7 a nt�	�  t
�d� d S )N�(   r   r	   r   zBox count for row exceededz0All boxes in config spawned, shuttinng down node)�idxZcurrent_realZsecs�data�box_count_in_row�max_box_per_rowr   r   �timerZshutdownr   Zsignal_shutdown)Zeventr   r   r   �check_spawn(   s    .
r$   �__main__Zspawn_boxesz/spawn_infor	   )Z
queue_size�   �   �   �   �%   �0   �1   �F   �G   �N   �O   �Q   �R   �T   �V   �X   �Y   �a   �c   �g   �l   �m   �w   �   �   �   �   �   �   �   �   �   ��   ��   ��   ��   ��   ��   ��   ��   �	   �   �   �   �   �   r   r
   r   g�������?)r   Zgazebo_msgs.msgr   Zgazebo_msgs.srvr   Zstd_msgs.msgr   r!   r"   r   r   r   r   r    r   r   r   r$   �__name__Z	init_nodeZ	Publisherr   �TimerZDurationr#   Zis_shutdownZspinr   r   r   r   �<module>   s.   
T


XXX
