# from control_proceso import *
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import GLib, Gtk, GObject
import os
import time
from simulador_SM13 import *
from leer_archivo_hx import *
from leer_plan import *
from depurador import *
from gi.repository import Pango
import sys
import numpy as np


f_Lx = 0
f_Ly = 0
f_Lp = 9.5
f_La = 15.25
f_pole = 0
f_arm = 0
ui_montaje = 0
ui_modo = 3
b_simulador = 0
s_archivo_fixture = "none"
s_archivo_hs = "none"
s_archivo_plan = "none"
ui_plan_col = 1
ui_plan_row = 1
s_tubo_id = "TUBE.DEFAULT"

def hmi_SM13():
    global iSeveridad
    global ui_montaje
    global f_Lx, f_Ly, f_Lp, f_La
    global b_simulador
    global s_archivo_fixture, s_archivo_hx, s_archivo_plan
    global f_pole, f_arm
    global ui_plan_col, ui_plan_row, s_tubo_id
    global s
    
    #f_Lx, f_Ly, f_Lp, f_La = leer_datos_SM13(ui_montaje)
    
    columns = ["COL",
               "ROW",
               "TUBE"]
    
    builder = Gtk.Builder()
    builder.add_from_file("/home/pi/Desktop/SM-13/GUI/HMI_SM-13.glade")
    
    """
    inspection_tree_view = builder.get_object("inspection_tree_view")
    s_archivo_plan = "ArchivosCfg/plan_inspeccion_"+str(ui_montaje)+".csv"
    a_plan_x, a_plan_y, a_tubos_id = leer_plan(s_archivo_plan)
    
    s_archivo_hx = "ArchivosCfg/hx_3211.csv"
    
    # the data in the model (three strings for each row, one for each
    # column)
    lista_tubos = Gtk.ListStore(str, str, str)
    # append the values in the model
    for i in range(len(a_tubos_id)-30):
        lista_tubos.append([str(a_plan_x[i]), str(a_plan_y[i]), str(a_tubos_id[i])])
    inspection_tree_view.set_model(model=lista_tubos)
    
    for i, column in enumerate(columns):
        # cellrenderer to render the text
        cell = Gtk.CellRendererText()
            
        # the column is created
        col = Gtk.TreeViewColumn(column, cell, text=i)
        
        # the text in the last column should be in boldface
        if i == 2:
            cell.props.weight_set = True
            cell.props.weight = Pango.Weight.BOLD
            col.set_fixed_width(90)
        else:
            col.set_fixed_width(50)

        # and it is appended to the treeview
        inspection_tree_view.append_column(col)
    """
    
    def apagar(button):
#         log.info("******************* Apagando sistema *******************")
#         buzzer(1, 10)
        Gtk.main_quit()
        os.system('sudo shutdown -h now')
        
    def reiniciar(button):
#         log.info("******************* Apagando sistema *******************")
#         buzzer(1, 10)
        Gtk.main_quit()
        os.system('sudo reboot')
            
    def fila_plan_inspeccion(selection):
        global iSeveridad
        global f_Lx, f_Ly, f_Lp, f_La
        global s_archivo_hx
        global ui_plan_col, ui_plan_row, s_tubo_id
        global f_pole, f_arm
        global f_px_tubo, f_py_tubo
        
        # get the model and the iterator that points at the data in the model
        (model, iter) = selection.get_selected()
        # set the label to a new value depending on the selection
        
        ui_plan_col = int(model[iter][0])
        print(str(ui_plan_col))
        ui_plan_row = int(model[iter][1])
        s_tubo_id = model[iter][2]
        
        # Extrae las distancias x,y desde el archivo del hx (en pulgadas) según tubo seleccionado
        f_px_tubo, f_py_tubo, a_lista_px, a_lista_py, b_error_coordenada = leer_archivo_hx(ui_plan_col, ui_plan_row,s_archivo_hx)
        
        # Calcula los ángulos de los ejes POLE y ARM en base a la distancia que hay que alcanzar
        f_pole, f_arm = ik_SM13(f_px_tubo, f_py_tubo, f_Lx, f_Ly, f_Lp, f_La)   
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Se seleccionó "+s_tubo_id+" (fila:"+str(ui_plan_row)+", columna:"+str(ui_plan_col)+")")
        depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(np.rad2deg(f_pole))+"°"+", ARM = "+str(np.rad2deg(f_arm))+"°")
        depurador(1, "HMI", " ")      
        
        return True
    
    def mover_a_seleccion(button):
        global s
        global f_pole, f_arm
        global ui_plan_col, ui_plan_row, s_tubo_id
        global f_px_tubo, f_py_tubo
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Moviendo hacia "+s_tubo_id+" (fila:"+str(ui_plan_row)+", columna:"+str(ui_plan_col)+")")
        depurador(1, "HMI", " ")
        inspection_etiqueta_msg.set_text("Moving to "+s_tubo_id+" (ROW : "+str(ui_plan_row)+", COL : "+str(ui_plan_col)+")")
        
        if(b_simulador == 1):
            depurador(1, "HMI", " - Nuevo ángulo POLE = " + str(f_pole))
            depurador(1, "HMI", " - Nuevo ángulo ARM  = " + str(f_arm))
            s.refrescar_grafico(f_px_tubo, f_py_tubo, f_pole, f_arm)
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Refrescando simulador SM-13")
            depurador(1, "HMI", " ")
            
        #HMIcomRTU(f_pole_cmd, f_arm_cmd)
        
        return True
        
    
    def run_simulador(button):
        global iSeveridad
        global s_archivo_hx
        global s_archivo_fixture
        global b_simulador
        global ui_montaje
        global ui_plan_col, ui_plan_row, s_tubo_id
        global f_px_tubo, f_py_tubo
        global s
         
        if (b_simulador == 0):
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", " - Inicia simulador SM-13")
            
            inspection_etiqueta_simulador.set_text("to OFF")
                
            f_px_tubo, f_py_tubo, a_lista_px, a_lista_py, b_error_coordenada = leer_archivo_hx(ui_plan_col, ui_plan_row, s_archivo_hx)
            depurador(1, "HMI", " - Simulador apuntando a x = " + str(f_px_tubo))
            depurador(1, "HMI", " - Simulador apuntando a y = " + str(f_py_tubo))
            
            s = simulador_SM13(f_px_tubo, f_py_tubo, a_lista_px, a_lista_py, s_archivo_fixture, ui_montaje)
            
            depurador(1, "HMI", " - Simulador refrescando con qp = " + str(f_pole))
            depurador(1, "HMI", " - Simulador refrescando con qa = " + str(f_arm))
            s.refrescar_grafico(f_px_tubo, f_py_tubo, f_pole, f_arm) 
            b_simulador = 1
            
            depurador(1, "HMI", " ")
            
            return True
            
        if (b_simulador == 1):
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", " - Se cierra simulador SM-13")
            depurador(1, "HMI", " ")
            del s
            inspection_etiqueta_simulador.set_text("to ON")
            b_simulador = 0
            
            return True
       
    def ventana_archivos(button):
        global window2
        window2 = builder.get_object("ventana_cfg").show_all()
                
        return True
        
    def cerrar_ventana_archivos(button):
        global window2
        window2 = builder.get_object("ventana_cfg").destroy()
            
        return True
    
    def seleccion_fixture(combobox):
        global iSeveridad
        global s_archivo_fixture
        global f_Lx, f_Ly, f_Lp, f_La
        global ui_montaje
        
        inspection_etiqueta_msg = builder.get_object("inspection_etiqueta_msg")
        inspection_etiqueta_msg.set_text("Listo para comenzar...")
        
        combo_fixture = combobox.get_active_iter()
        model_fixture = combobox.get_model()
        s_archivo_fixture = "ArchivosCfg/"+str(model_fixture[combo_fixture][0])+".csv"
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Archivo fixture: "+s_archivo_fixture)
        depurador(1, "HMI", " ")
        
        f_Lx, f_Ly, f_Lp, f_La = leer_datos_SM13(s_archivo_fixture, ui_montaje)
        
        return True

    def seleccion_hx(combobox):
        global iSeveridad
        global s_archivo_hx
        
        combo_hx = combobox.get_active_iter()
        model_hx = combobox.get_model()
        s_archivo_hx = "ArchivosCfg/"+str(model_hx[combo_hx][0])+".csv"
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Archivo fixture: "+s_archivo_hx)
        depurador(1, "HMI", " ")
        
        return True
    
    def seleccion_plan(combobox):
        global iSeveridad
        global s_archivo_plan
        global ui_montaje
        a_plan_x = [0]
        a_plan_y = [0]
        a_tubos_id = [0]
               
        combo_plan = combobox.get_active_iter()
        model_plan = combobox.get_model()
        s_archivo_plan = "ArchivosCfg/"+str(model_plan[combo_plan][0])+".csv"
        a_plan_x, a_plan_y, a_tubos_id = leer_plan(s_archivo_plan)

        trash, trash, trash, trash, trash, trash, str_montaje = (model_plan[combo_plan][0]).split("_")
        ui_montaje = int(str_montaje)
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Archivo fixture: "+s_archivo_plan)
        depurador(1, "HMI", "- Montaje: "+ str(ui_montaje))
        depurador(1, "HMI", " ")
        
        inspection_tree_view = builder.get_object("inspection_tree_view")
               
        # the data in the model (three strings for each row, one for each
        # column)
        lista_tubos = Gtk.ListStore(str, str, str)
        
        # append the values in the model
        lista_tubos.clear()
        inspection_tree_view.set_model(model=lista_tubos)
        
        for i in range(len(a_tubos_id)):
            lista_tubos.append([str(a_plan_x[i]), str(a_plan_y[i]), str(a_tubos_id[i])])
            inspection_tree_view.set_model(model=lista_tubos)        
        
        for i, column in enumerate(columns):
            # cellrenderer to render the text
            cell = Gtk.CellRendererText()
                
            # the column is created
            col = Gtk.TreeViewColumn(column, cell, text=i)
            
            # the text in the last column should be in boldface
            if i == 2:
                cell.props.weight_set = True
                cell.props.weight = Pango.Weight.BOLD
                col.set_fixed_width(90)
            else:
                col.set_fixed_width(50)

            # and it is appended to the treeview
            inspection_tree_view.append_column(col)
            
        return True

        
    señales = {
        "terminar_aplicacion":Gtk.main_quit,
        "evento_apagar": apagar,
        "evento_reiniciar":reiniciar,
        "evento_seleccion_fila": fila_plan_inspeccion,
        "evento_run_simulador": run_simulador,
        "evento_mover_a_seleccion": mover_a_seleccion,
        "evento_archivos": ventana_archivos,
        "terminar_ventana_cfg": cerrar_ventana_archivos,
        "evento_cerrar_ventana_cfg": cerrar_ventana_archivos,
        "evento_seleccion_fixture": seleccion_fixture,
        "evento_seleccion_hx": seleccion_hx,
        "evento_seleccion_plan": seleccion_plan
    }
    
    builder.connect_signals(señales)
    inspection_etiqueta_msg = builder.get_object("inspection_etiqueta_msg")
    if (s_archivo_fixture == "none" or s_archivo_hx == "none" or s_archivo_plan == "none"):
        inspection_etiqueta_msg.set_text("Seleccione archivos de configuración desde solapa inicio...")
    inspection_etiqueta_simulador = builder.get_object("inspection_etiqueta_simulador")
    
    window = builder.get_object("ventana_principal")
    
    window.show_all()

if __name__ == "__main__":
    hmi_SM13()
    Gtk.main()
    
    