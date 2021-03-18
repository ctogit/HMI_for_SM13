##
# @file ik_SM13.py
#
# @brief Esta función calcula la Cinemática Inversa (Inverse Kinematics, ik) del
# telemanipulador de dos GDL, SM-13. Esto es, calcula la posición de los ángulos
# de la articulaciones POLE y ARM que debe alcanzar el tele-manipulador para que
# la boquilla se posicione en un determinado punto X, Y. Para ello también tiene
# en cuenta las longitudes de estas articulaciones y la posición de montaje del
# SM-13.
#
# @param f_x Punto en eje X al que se desea alcanzar con la boquilla (en pulgadas)
# @param f_y punto final en el eje Y que se desea alcanzar con la boquilla (en pulgadas)
# @param f_l1 posición en el eje X donde se instala el SM-13 (en pulgadas)
# @param f_l2 posición en el eje Y donde se instala el SM-13 (en pulgadas)
# @param f_l3 longitud del eslabón POLE (en pulgadas)
# @param f_l4 longitud del eslabón ARM (en pulgadas)
#
# @return f_qp Ángulo de la articulación POLE (en radianes)
# @return f_qa Ángulo de la articulación ARM (en radianes)
#
# @author Cristian Torres Barrios
# creado Vie 18 Sep 19:48:00 2020

import numpy as np
import math
from depurador import *

def ik_SM13(f_x, f_y, f_l1, f_l2, f_l3, f_l4, s_pivot_type):
    global iSeveridad
    
    f_Px = f_x
    f_Py = f_y
    f_Lx = f_l1
    f_Ly = f_l2
    f_Lp = f_l3
    f_La = f_l4
    
    depurador(2, "ik_SM13", "****************************************")
    depurador(2, "ik_SM13", "- Cinemática Inversa SM-13")
    depurador(2, "ik_SM13", " ")
    depurador(2, "ik_SM13", "- Coor X tubo = " + str(f_Px))
    depurador(2, "ik_SM13", "- Coor Y tubo = " + str(f_Py))
    depurador(2, "ik_SM13", "- Lx = " + str(f_l1))
    depurador(2, "ik_SM13", "- Ly = " + str(f_l2))
    depurador(2, "ik_SM13", "- Longitud_POLE = " + str(f_l3))
    depurador(2, "ik_SM13", "- Longitud_ARM  = " + str(f_l4))

    try:
        f_r1 = (f_Lx - f_Px)
        f_r2 = (f_Ly - f_Py)
        f_h = (math.sqrt(f_r1**2 + f_r2**2))
        f_alfa = (math.atan2(f_r2, f_r1))
        f_beta = (math.acos((f_Lp**2 - f_La**2 + f_h**2)/(2*f_Lp*f_h)))

        if (s_pivot_type == "main"):
            f_qp = f_beta - f_alfa
            f_qa = math.acos((f_La**2 + f_Lp**2 - f_h**2)/(2*f_La*f_Lp))
        elif (s_pivot_type == "alternative"):
            f_qp = 2*math.pi - f_alfa - f_beta
            f_qa = 2*math.pi - math.acos((f_La**2 + f_Lp**2 - f_h**2)/(2*f_La*f_Lp))

        # A veces pasa que la matemática da números negativos o mayores a 360ª así que se corrige. 
        if f_qp < 0.0:
            f_qp = f_qp + 2*np.pi
        if f_qa < 0.0:
            f_qa = f_qa + 2*np.pi
        if f_qp >= 2*np.pi:
            f_qp = f_qp - 2*np.pi
        if f_qa >= 2*np.pi:
            f_qa = f_qa - 2*np.pi
        b_success = True

        depurador(2, "ik_SM13", " ")
        depurador(2, "ik_SM13", "- Ángulo POLE = "+ str(np.rad2deg(f_qp)))
        depurador(2, "ik_SM13", "- Ángulo ARM = " + str(np.rad2deg(f_qa)))
        depurador(2, "ik_SM13", " ")

    except Exception as err_msg:
        if err_msg == "math domain error":
            f_qp = 0.0
            f_qa = 0.0
            b_success = False
            depurador(2, "ik_SM13", " ")
            depurador(2, "ik_SM13", "- Ángulos POLE y ARM inalcanzables...")
            depurador(2, "ik_SM13", " ")
        else:
            f_qp = 0.0
            f_qa = 0.0
            b_success = False
            depurador(2, "ik_SM13", " ")
            depurador(2, "ik_SM13", "- Ángulos POLE y ARM inalcanzables: " + str(err_msg))
            depurador(2, "ik_SM13", " ")
    
    return float(f_qp), float(f_qa), b_success
    
if __name__ == "__main__":
    ik_SM13(29.1676, 3.9774, 20.5499, 3.3145, 9.5, 15.52, "alternative")
    # qp = 52.14ª; qa = 85.29ª
