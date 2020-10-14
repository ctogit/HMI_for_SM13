##
# @file dk_SM13.py
#
# @brief Esta función calcula la Cinemática Directa (Direct Kinematics, dk) del
# telemanipulador de dos GDL, SM-13. Esto es, calcula la posición final en X, Y
# de la boquilla en base a los ángulos de la articulaciones POLE y ARM. Para
# ello también tiene en cuenta las longitudes de estas articulaciones y la
# posición de montaje del SM-13.
#
# @param f_pole_ang Ángulo de la articulación POLE (en grados)
# @param f_arm_ang Ángulo de la articulación ARM (en grados)
# @param f_l1 Posición en el eje X donde se instala el SM-13 (en pulgadas)
# @param f_l2 Posición en el eje Y donde se instala el SM-13 (en pulgadas)
# @param f_l3 Longitud del eslabón POLE (en pulgadas)
# @param f_l4 Longitud del eslabón ARM (en pulgadas)
#
# @return f_px_codo Posición X de la unión POLE-ARM (codo)
# @return f_py_codo: posición Y de la unión POLE-ARM (codo)
# @return f_px_sonda: posición final en eje X de la sonda
# @return f_py_sonda: posición final en eje Y de la sonda
#
# @author Cristian Torres Barrios
# creado Vie 18 Sep 23:03:00 2020

import numpy as np
import math
from leer_datos_SM13 import *
from depurador import *

def dk_SM13(f_pole_ang, f_arm_ang, f_l1, f_l2, f_l3, f_l4):
    global iSeveridad
  
    f_qp = np.rad2deg(f_pole_ang)
    f_qa = np.rad2deg(f_arm_ang)
    f_Lx = f_l1
    f_Ly = f_l2
    f_Lp = f_l3
    f_La = f_l4

    # Anm -> matriz de transformación del espacio n al m
    a_A01 = np.array([
        [1, 0, 0, f_Lx],
        [0, math.cos(np.deg2rad(270)), -math.sin(np.deg2rad(270)), 0],
        [0, math.sin(np.deg2rad(270)), math.cos(np.deg2rad(270)), 0],
        [0, 0, 0, 1]
        ])

    a_A12 = np.array([
        [1, 0, 0, 0],
        [0, math.cos(np.deg2rad(90)), -math.sin(np.deg2rad(90)), 0],
        [0, math.sin(np.deg2rad(90)), math.cos(np.deg2rad(90)), f_Ly],
        [0, 0, 0, 1]
        ])

    a_A23 = np.array([
        [math.cos(np.deg2rad(f_qp)), -math.sin(np.deg2rad(f_qp)), 0, f_Lp*math.sin(np.deg2rad(f_qp))],
        [math.sin(np.deg2rad(f_qp)), math.cos(np.deg2rad(f_qp)), 0, -f_Lp*math.cos(np.deg2rad(f_qp))],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])

    a_A34 = np.array([
        [math.cos(np.deg2rad(f_qa)), -math.sin(np.deg2rad(f_qa)), 0, f_La*math.sin(np.deg2rad(f_qa))],
        [math.sin(np.deg2rad(f_qa)), math.cos(np.deg2rad(f_qa)), 0, f_La*math.cos(np.deg2rad(f_qa))],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])

    a_A02 = np.dot(a_A01, a_A12)
    a_A03 = np.dot(a_A02, a_A23)
    
    # Matriz de Transformación Homogénea (T)
    a_T = np.dot(a_A03, a_A34)
    
    f_px_codo = float(a_A03[0,3])
    f_py_codo = float(a_A03[1,3])
    
    f_px_sonda = float(a_T[0,3])
    f_py_sonda = float(a_T[1,3])

    depurador(2, "dk_SM13", "****************************************")
    depurador(2, "dk_SM13", "- Cinemática Directa SM-13")
    depurador(2, "dk_SM13", " ")
    depurador(2, "dk_SM13", "- Ángulo POLE = "+str(f_qp))
    depurador(2, "dk_SM13", "- Ángulo ARM  = "+str(f_qa))
    depurador(2, "dk_SM13", " ")
    depurador(2, "dk_SM13", "- Sonda en x = "+str(f_px_sonda)+", y = "+str(f_py_sonda))
    depurador(2, "dk_SM13", " ")
    
    return f_px_codo, f_py_codo, f_px_sonda, f_py_sonda
    
    
# if __name__ == "__main__":
#     dk_SM13(np.deg2rad(56.375), np.deg2rad(137.143), 47.725, 3.518, 9.5, 15.25)
