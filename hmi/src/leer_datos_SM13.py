##
# @file leer_datos_SM13.py
#
# @brief Esta función lee un archivo que contiene las características físicas y de
# montaje del telemanipulador SM-13.
#
# @param s_fixture_file Nombre del archivo .csv con los parámetros físicos del
# telemanipulador y sus posibles montajes
# @param ui_montaje Posición de montaje del SM-13 respecto al HX
#
# @return f_Lx Distancia en pulgadas en el eje X desde el punto de referencia 0 del HX
# hasta el centro de la base de montaje del SM-13.
# @return f_Ly Distancia en pulgadas en el eje Y desde el punto de referencia 0 del HX
# hasta el centro de la base de montaje del SM-13.
# @return f_Lp Longitud en pulgadas del eslabón POLE del telemanipulador SM-13
# @return f_La Longitud en pulgadas del eslabón ARM del telemanipulador SM-13
# @return f_w Ancho base SM-13
# @return f_h Alto base SM-13
#
# @author Cristian Torres Barrios
# creado Vie 18 Sep 19:58:00 2020

import csv
from depurador import *

def leer_datos_SM13(s_hx_type, s_fixture_file, ui_montaje = 0):
    global iSeveridad
    a_medidas=[]
    
    with open(s_fixture_file, "rt", encoding='ascii') as f:
        telemanipulador = csv.reader(f, delimiter=";")
        header = next(telemanipulador)
        
        i=0
        for data in telemanipulador:
            a_medidas.append(data[1])
            i += 1
    
    if (ui_montaje == 0):
        
        depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
        depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
        depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
        depurador(3, "Datos_SM13", "- Montaje en X = 0 in")
        depurador(3, "Datos_SM13", "- Montaje en Y = 0 in")
        
        return float(0.0), float(0.0), float(a_medidas[0]), float(a_medidas[1]), float(a_medidas[2]), float(a_medidas[3])

    if (s_hx_type == "_cne_moderador_3211_HX1"):   
        if (ui_montaje == 1):
            
            depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
            depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
            depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
            depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[4])+" in")
            depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[5])+" in")
            
            return float(a_medidas[4]), float(a_medidas[5]), float(a_medidas[0]), float(a_medidas[1]), float(a_medidas[2]), float(a_medidas[3])
        
        if (ui_montaje == 2):
            
            depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
            depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
            depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
            depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[6])+" in")
            depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[7])+" in")
            
            return float(a_medidas[6]), float(a_medidas[7]), float(a_medidas[0]), float(a_medidas[1]), float(a_medidas[2]), float(a_medidas[3])
        
        if (ui_montaje == 3):
            depurador(3, "Datos_SM13", "****************************************")
            depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
            depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
            depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
            depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[8])+" in")
            depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[9])+" in")
                        
            return float(a_medidas[8]), float(a_medidas[9]), float(a_medidas[0]), float(a_medidas[1]), float(a_medidas[2]), float(a_medidas[3])

        if (ui_montaje == 4):
            depurador(3, "Datos_SM13", "****************************************")
            depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
            depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
            depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
            depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[10])+" in")
            depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[11])+" in")
                        
            return float(a_medidas[10]), float(a_medidas[11]), float(a_medidas[0]), float(a_medidas[1]), float(a_medidas[2]), float(a_medidas[3])
    
    if (s_hx_type == "_cne_moderador_3211_HX2"):   
        if (ui_montaje == 1):
            
            depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
            depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
            depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
            depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[12])+" in")
            depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[13])+" in")
            
            return float(a_medidas[12]), float(a_medidas[13]), float(a_medidas[0]), float(a_medidas[1]), float(a_medidas[2]), float(a_medidas[3])
        
        if (ui_montaje == 2):
            
            depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
            depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
            depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
            depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[14])+" in")
            depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[15])+" in")
            
            return float(a_medidas[14]), float(a_medidas[15]), float(a_medidas[0]), float(a_medidas[1]), float(a_medidas[2]), float(a_medidas[3])
        
        if (ui_montaje == 3):
            depurador(3, "Datos_SM13", "****************************************")
            depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
            depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
            depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
            depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[16])+" in")
            depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[17])+" in")
                        
            return float(a_medidas[16]), float(a_medidas[17]), float(a_medidas[0]), float(a_medidas[1]), float(a_medidas[2]), float(a_medidas[3])

        if (ui_montaje == 4):
            depurador(3, "Datos_SM13", "****************************************")
            depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
            depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
            depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
            depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[18])+" in")
            depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[19])+" in")
                        
            return float(a_medidas[18]), float(a_medidas[19]), float(a_medidas[0]), float(a_medidas[1]), float(a_medidas[2]), float(a_medidas[3])
        
    
    depurador(2, "Datos_SM13", " ")
#if __name__ == "__main__":
#    leer_datos_SM13(3)