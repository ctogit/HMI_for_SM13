##
# @file leer_archivo_hx.py
#
# @brief Esta función lee un archivo de datos de un intercambiador de calor
# hx determinado y devuelve una posición (X,Y) en pulgadas en base la
# fila y columna que se desea alcanzar.
#
# @param ui_row_input Fila del hx
# @param ui_col_input Columna del hx
# @param s_hx_file Ruta al archivo de datos que contiene la info del
# intercambiador de calor (filas, columnas, coordenadas xy, tubos ID, etc)
#
# @return f_px_output Coordenada X del punto (col, row) en pulgadas
# @return f_py_output Coordenada Y del punto (col, row) en pulgadas
# @return a_px_list Lista con todos los valores X (columnas) del hx en pulgadas
# @return a_py_list Lista con todos los valores Y (filas) del hx en pulgadas
# @return b_error 0: operación exitosa, 1: no se encontró coordenada [x,y]
# para la entrada COL,ROW dada.
#
# @author Cristian Torres Barrios
# creado Vie 18 Sep 20:35:00 2020

import csv
from depurador import *

def leer_archivo_hx(ui_col_input, ui_row_input, s_hx_file):
    global iSeveridad
    
    a_xcol_list=[]
    a_yrow_list=[]
    a_px_list=[]
    a_py_list=[]
    a_tube_list=[]
    f_px_output = 0.0
    f_py_output = 0.0
    b_error = 1
    
    #h, hxData = read_spreadsheet("ArchivosCfg/hx3211.xls",csv_dialect="CToDialect", header_size=2) 
    #h, planData = read_spreadsheet(/ArchivosCfg/planInspecion,csv_dialect=CSVDialect, header_size=0)
    
    depurador(3, "Archivo_hx", "****************************************")
    depurador(3, "Archivo_hx", "- Archivo de datos del Intercambiador")
    depurador(3, "Archivo_hx", "- "+s_hx_file)
    depurador(3, "Archivo_hx", "- Buscando COL: "+str(ui_col_input))
    depurador(3, "Archivo_hx", "- Buscando ROW: "+str(ui_row_input))

    with open(s_hx_file, "rt", encoding='ascii') as f:
        hx = csv.reader(f, delimiter=";")
        header = next(hx)
        
        i=0
        for data in hx:
            # Se arman las listas en base a los datos del archivo
            a_xcol_list.append(data[0])              
            a_yrow_list.append(data[1])
            a_px_list.append(data[2])
            a_py_list.append(data[3])
            a_tube_list.append(data[4])
            
            # Se selecciona la coordenada X,Y del archivo en base a COL,ROW de entrada. 
            if (int(a_xcol_list[i]) == ui_col_input) and (int(a_yrow_list[i]) == ui_row_input):           
                f_px_output = float(a_px_list[i])
                f_py_output = float(a_py_list[i])
                # Si se encontró el dato se resetea la variable de error.
                b_error = 0
            i += 1
        
        # Si no se reseteó la variable de error es que no hay coor X,Y con la COL,ROW de entrada
        if b_error == 1:
            depurador(3, "Archivo hx", "- Coordenada inalcanzable...")

        else:
            depurador(3, "Archivo_hx", "- Colmna: "+str(ui_col_input)+", x = "+str(f_px_output))
            depurador(3, "Archivo_hx", "- Fila  : "+str(ui_row_input)+", y = "+str(f_py_output))
        
        depurador(3, "Archivo_hx", " ")
        
        return f_px_output, f_py_output, a_px_list, a_py_list, b_error
    
#if __name__ == "__main__":
#    leer_archivo_hx(2,1,"/home/pi/Desktop/SM-13/cfg_files/hx_3211.csv")