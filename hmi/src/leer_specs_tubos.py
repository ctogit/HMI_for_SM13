##
# @file leer_specs_tubos.py
#
# @brief Esta función lee un archivo que contiene las características físicas y de
# de los tubos que componene el HX seleccionado.
#
# @param s_tube_file Nombre del archivo .csv con los parámetros físicos de los
# tubos
#
# @return a_specs
# [TUBE_OD, Y_PITCH, X_PITCH, CFG, CALLE_ANCHA, CALLE_ANGOSTA, MAX_NUMBER_ROWS, MAX_NUMBER_COLS,
# ANCHO_X_MAX, ANCHO_X_MIN. ALTO_Y_MAX,ALTO_Y_MIN]
#
# @author Cristian Torres Barrios
# creado Vie 16 Oct 00:18:00 2020

import csv
from depurador import *

def leer_specs_tubos(s_tube_file):
    global iSeveridad
    a_specs=[]
    
    with open(s_tube_file, "rt", encoding='ascii') as f:
        data = csv.reader(f, delimiter=";")
        header = next(data)
        
        i=0
        for d in data:
            a_specs.append(d[1])
            i += 1
    
    
    depurador(3, "Specs_tubos", "****************************************")
    depurador(3, "Specs_tubos", "- Archivo de especificación de tubos")
    depurador(3, "Specs_tubos", "- TUBE_OD         = "+str(a_specs[0])+" in")
    depurador(3, "Specs_tubos", "- Y_PITCH         = "+str(a_specs[1])+" in")
    depurador(3, "Specs_tubos", "- X_PITCH         = "+str(a_specs[2])+" in")
    depurador(3, "Specs_tubos", "- CFG             = "+ a_specs[3])
    depurador(3, "Specs_tubos", "- CALLE_ANCHA     = "+str(a_specs[4])+" in")
    depurador(3, "Specs_tubos", "- CALLE_ANGOSTA   = "+str(a_specs[5])+" in")
    depurador(3, "Specs_tubos", "- MAX_NUMBER_ROWS = "+str(a_specs[6]))
    depurador(3, "Specs_tubos", "- MAX_NUMBER_COLS = "+str(a_specs[7]))
    depurador(3, "Specs_tubos", "- ANCHO_X_MAX     = "+str(a_specs[8])+" in")
    depurador(3, "Specs_tubos", "- ANCHO_X_MIN     = "+str(a_specs[9])+" in")
    depurador(3, "Specs_tubos", "- ALTO_Y_MAX      = "+str(a_specs[10])+" in")
    depurador(3, "Specs_tubos", "- ALTO_Y_MIN      = "+str(a_specs[11])+" in")
    depurador(3, "Specs_tubos", " ")

    return a_specs
        
    
#if __name__ == "__main__":
#    leer_specs_tubos("/home/pi/Desktop/SM-13/hmi/cfg_files/Heat_exchangers/new_cne_moderator_HX1_3211/tube_specs.csv")
