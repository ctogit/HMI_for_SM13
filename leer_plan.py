# -*- coding: utf-8 -*-
"""
Creado Vie 18 Sep 19:48:00 2020

@author: Torres Barrios Cristian

    Nombre función:     leer_plan
    
    Retorna:            a_row_plan: filas de los tubos que están incluidos en plan de inspección
                        a_col_plan: columnas de los tubos que están incluidos en el plan
                        a_tube_plan: identificación de los tubos del plan de inspección

    Parámetros:         s_plan_file: archivo .csv con los datos de filas y columnas de los tubos
                        del plan de inspección.

    Descripción:        Esta función lee un archivo de plan de inspección y devuelve listas de datos
                        de fila, columna y ID de cada tubo incluido en el plan.
"""

import csv
from depurador import *

def leer_plan(s_plan_file):
    global iSeveridad
    
    a_row_plan=[]
    a_col_plan=[]
    a_tube_plan=[]
    
    with open(s_plan_file, "rt", encoding='ascii') as f:
        depurador(3, "Plan de Inspección", "****************************************")
        depurador(3, "Plan de Inspección", "- Accediendo")
        depurador(3, "Plan de Inspección", "- Archivo: "+s_plan_file)
        
    #with open("ArchivosCfg/plan_inspeccion_3.csv", "rt", encoding='ascii') as f:
        plan = csv.reader(f, delimiter=";")
        header = next(plan)
        depurador(3, "Plan de Inspección", header)
        depurador(3, "Plan de Inspección", " ")
          
        i=0
        for data in plan:
            a_row_plan.append(int(data[0]))
            a_col_plan.append(int(data[1]))
            a_tube_plan.append(data[2])
            i += 1
            
    return a_row_plan, a_col_plan, a_tube_plan
        
#if __name__ == "__main__":
#    leer_plan("ArchivosCfg/plan_inspeccion_3.csv")