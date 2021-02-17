##
# @file leer_plan.py
#
# @brief Esta función lee un archivo de plan de inspección y devuelve listas
# de datos de fila, columna y ID de cada tubo incluido en el plan.
#
# @param s_plan_file Nombre del archivo .csv con los datos de filas y columnas
# de los tubos del plan de inspección.
#
# @return a_row_plan Filas de los tubos que están incluidos en plan de inspección
# @return a_col_plan Columnas de los tubos que están incluidos en el plan
# @return a_tube_plan Identificación de los tubos del plan de inspección
#
# @author Cristian Torres Barrios
# creado Vie 18 Sep 19:48:00 2020

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
        
        plan = csv.reader(f, delimiter=";")
        header = next(plan)
        depurador(3, "Plan de Inspección", header)
        depurador(3, "Plan de Inspección", " ")
          
        i=0
        
        for data in plan:
            print(data[0])
            a_row_plan.append(str(data[0]))
            a_col_plan.append(int(data[1]))
            a_tube_plan.append(data[2])

            # Algunos HX tienen filas nombradas con letras así que antes de retornar 
            # se pasan a entero para que no haya problemas
            if (a_row_plan[i] == "A"):
                a_row_plan[i] = int(1)
            if (a_row_plan[i] == "B"):
                a_row_plan[i] = int(2)
            if (a_row_plan[i] == "C"):
                a_row_plan[i] = int(3)
            if (a_row_plan[i] == "D"):
                a_row_plan[i] = int(4)
            if (a_row_plan[i] == "E"):
                a_row_plan[i] = int(5)
            if (a_row_plan[i] == "F"):
                a_row_plan[i] = int(6)
            if (a_row_plan[i] == "G"):
                a_row_plan[i] = int(7)
            if (a_row_plan[i] == "H"):
                a_row_plan[i] = int(8)
            if (a_row_plan[i] == "I"):
                a_row_plan[i] = int(9)
            if (a_row_plan[i] == "J"):
                a_row_plan[i] = int(10)
            if (a_row_plan[i] == "K"):
                a_row_plan[i] = int(11)
            i += 1

            
    return a_row_plan, a_col_plan, a_tube_plan
        
#if __name__ == "__main__":
#    leer_plan("ArchivosCfg/plan_inspeccion_3.csv")