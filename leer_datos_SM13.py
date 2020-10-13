import csv
from depurador import *

def leer_datos_SM13(s_fixture_file, ui_montaje = 0):
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
        
        return float(0.0), float(0.0), float(a_medidas[0]), float(a_medidas[1])
    
    if (ui_montaje == 1):
        
        depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
        depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
        depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
        depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[2])+" in")
        depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[3])+" in")
        
        return float(a_medidas[2]), float(a_medidas[3]), float(a_medidas[0]), float(a_medidas[1])
    
    if (ui_montaje == 2):
        
        depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
        depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
        depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
        depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[4])+" in")
        depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[5])+" in")
        
        return float(a_medidas[4]), float(a_medidas[5]), float(a_medidas[0]), float(a_medidas[1])
    
    if (ui_montaje == 3):
        depurador(2, "Datos_SM13", "****************************************")
        depurador(3, "Datos_SM13", "- Archivo de datos SM-13")
        depurador(3, "Datos_SM13", "- Largo POLE = "+str(a_medidas[0])+" in")
        depurador(3, "Datos_SM13", "- Largo ARM  = "+str(a_medidas[1])+" in")
        depurador(3, "Datos_SM13", "- Montaje en X = "+str(a_medidas[6])+" in")
        depurador(3, "Datos_SM13", "- Montaje en Y = "+str(a_medidas[7])+" in")
        depurador(2, "Datos_SM13", " ")
        
        return float(a_medidas[6]), float(a_medidas[7]), float(a_medidas[0]), float(a_medidas[1])
        
    
#if __name__ == "__main__":
#    leer_datos_SM13(3)