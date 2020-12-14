# @param s_plan_file Nombre del archivo .csv con los datos de filas y columnas
# de los tubos del plan de inspección.
#
# @return a_row_plan Filas de los tubos que están incluidos en plan de inspección
# @return a_col_plan Columnas de los tubos que están incluidos en el plan
# @return a_tube_plan Identificación de los tubos del plan de inspección
#
# @author Cristian Torres Barrios
# creado Vie 18 Sep 19:48:00 2020
#
#                               |---------------|
#      IP_ROW_COL =========== > | ip_creator.py | ============ > IP_ROW_COL_TUBE-ID
#                               |---------------|
#                                       ^
#                                       |
#         TUBESHEET    =================
#

import csv
from depurador import *

def inspection_plan_creator():
    global iSeveridad
    
    a_row_plan=[]
    a_col_plan=[]
    a_row_hx=[]
    a_col_hx=[]
    a_tube_hx=[]

    a_tube_plan=[]
    
    s_hx_file = "/home/gspc/Proyectos/SM-13/hmi/cfg_files/Heat_exchangers/_cne_moderador_3211_HX2/tubesheet.csv"
    s_plan_file = "/home/gspc/Proyectos/SM-13/hmi/cfg_files/Heat_exchangers/_cne_moderador_3211_HX2/M-4_PI-8.csv"
    s_plan_file_output = "/home/gspc/Proyectos/SM-13/hmi/cfg_files/Heat_exchangers/_cne_moderador_3211_HX2/InspPlan8_M_4.csv"

    with open(s_plan_file, "rt", encoding='ascii') as f:
        depurador(3, "Plan de Inspección", "****************************************")
        depurador(3, "Plan de Inspección", "- Accediendo")
        depurador(3, "Plan de Inspección", "- Archivo: "+s_plan_file)
        
        plan = csv.reader(f, delimiter=";")
        header = next(plan)

        depurador(3, "Plan de Inspección", header)
        depurador(3, "Plan de Inspección", " ")
          
        for data in plan:
            a_row_plan.append(int(data[0]))
            a_col_plan.append(int(data[1]))

    with open(s_hx_file, "rt", encoding='ascii') as f:
        depurador(3, "Plan de Inspección", "****************************************")
        depurador(3, "Plan de Inspección", "- Accediendo")
        depurador(3, "Plan de Inspección", "- Archivo: "+s_hx_file)

        hx_list = csv.reader(f, delimiter=";")
        hx_list_header = next(hx_list)

        for data_hx in hx_list:
            a_row_hx.append(int(data_hx[1]))
            a_col_hx.append(int(data_hx[0]))
            a_tube_hx.append(str(data_hx[6]))


    output_file = open(s_plan_file_output, "w+")
    output_file.write("ROW;COL;TUBEx\n")

    for x in range(len(a_row_plan)):
        for y in range(len(a_row_hx)):
            if (a_row_plan[x] == a_row_hx[y]) and (a_col_plan[x] == a_col_hx[y]):
                print(str(a_row_plan[x]) + ";" + str(a_col_plan[x]) + ";" + str(a_tube_hx[y]))
                output_line = str(a_row_plan[x]) + ";" + str(a_col_plan[x]) + ";" + str(a_tube_hx[y] + "\n")
                output_file.write(output_line)
    
    output_file.close()

    return True

if __name__ == "__main__":
    inspection_plan_creator()