##
# Clase principal que levanta el archivo .glade y construye la
# interfaz gráfica HMI con sus diferentes funcionalidades para controlar
# y monitorear el telemanipulador SM-13
#
# @section hola TODO
# - Solapa FR: faltan implementar botones, ventana pop-up archivos no deja abrir dos veces
#   y cierra todo el programa. Avance 30%.
# - Solapa Manual:  avance 0%.
# - Solapa Inspection: TreeView no reemplaza datos y agrega columnas al cambiar de plan.
#   Falta implementar botón "Next Tube", sección "Lift" y "Jog Control". Avance al 60%.
# - Solapa Info: avance 0%.
#
# @author Cristian Torres Barrios
# creado Vie 30 Sep 23:03:00 2020

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

class hmi_SM13():   
    global simu
    
    ## El constructor del HMI
    def __init__(self):
        # Atributos públicos de la clase hmi_SM13
        ## Contiene la distancia en eje X en pulgadas del montaje de la base del SM-13 respecto a hx.
        # Depende del plan de inspección elegido.
        self.f_Lx = 0.0
        ## Contiene la distancia en eje Y en pulgadas del montaje de la base del SM-13 respecto a hx.
        # Depende del plan de inspección elegido.
        self.f_Ly = 0.0
        ## Contiene la longitud del eslabón POLE del telemanipulador.
        # Se inicializa en 0 pero la modifica el archivo de datos del telemanipulador.
        self.f_Lp = 0.0
        ## Contiene la longitud del eslabón ARM del telemanipulador.
        # Se inicializa en 0 pero la modifica el archivo de datos del telemanipulador.
        self.f_La = 0.0
        ## Contiene el ángulo de rotación de la articulación POLE respecto a la base del telemanipulador.
        self.f_pole = 0.0
        ## Contiene el ángulo de rotación de la articulación ARM respecto a la base del telemanipulador.
        self.f_arm = 0.0
        ## Contiene el tipo de montaje respecto al hx.
        # 0: centro en caso de no especificar hx ni montaje,
        # 1: plan de inspección 1,
        # 2: plan de inspección 2,
        # 3: plan de inspección 3.
        self.ui_montaje = 0
        ## Modo de operación HMI.
        # 1: Free Run
        # 2: Manual
        # 3: Inspection
        self.ui_modo = 3
        ## Contiene el estado del simulador
        # 0: simulador inactivo
        # 1: simulador activo
        self.b_simulador = 0
        ## Contiene la ruta al archivo con datos del telemanipulador
        self.s_archivo_fixture = "none"
        ## Contiene la ruta al archivo con datos del intercambiador de calor
        self.s_archivo_hx = "none"
        ## Contiene la ruta al archivo que contiene el plan del inspección
        self.s_archivo_plan = "none"
        ## Contiene la columna elegida en el mazo de tubos del hx
        self.ui_plan_col = 1
        ## Contiene la fila elegida en el mazo de tubos del hx
        self.ui_plan_row = 1
        ## Contiene el número de tubo elegido en el mazo de tubos del hx
        self.s_tubo_id = "TUBE.DEFAULT"
        ## Contiene la ruta al archivo xml de la interfaz
        self.s_gui_path = "/home/pi/Desktop/SM-13/gui/hmi_SM-13.glade"
        ## Contiene la ruta a la carpeta de archivos de configuración del sistema
        self.s_cfg_files_path = "/home/pi/Desktop/SM-13/cfg_files/"
        ## Crea el objeto Gtk para levantar la interfáz gráfica
        self.builder = Gtk.Builder()
        # Levanta la interfáz gráfica desde el archivo especificado
        self.builder.add_from_file(self.s_gui_path)
        
        señales = {
            "terminar_aplicacion":Gtk.main_quit,
            "evento_apagar": self.apagar,
            "evento_reiniciar":self.reiniciar,
            "evento_seleccion_fila": self.fila_plan_inspeccion,
            "evento_run_simulador": self.run_simulador,
            "evento_mover_a_seleccion": self.mover_a_seleccion,
            "evento_archivos": self.ventana_archivos,
            "terminar_ventana_cfg": self.cerrar_ventana_archivos,
            "evento_cerrar_ventana_cfg": self.cerrar_ventana_archivos,
            "evento_seleccion_fixture": self.seleccion_fixture,
            "evento_seleccion_hx": self.seleccion_hx,
            "evento_seleccion_plan": self.seleccion_plan
        }
        self.builder.connect_signals(señales)
        
        ## Etiqueta pública de mensajes de la solapa Inspection
        self.inspection_etiqueta_msg = self.builder.get_object("inspection_etiqueta_msg")
        if (self.s_archivo_fixture == "none" or self.s_archivo_hx == "none" or self.s_archivo_plan == "none"):
            self.inspection_etiqueta_msg.set_text("Seleccione archivos de configuración desde solapa inicio...")
        
        ## Etiqueta dinámica que se muestra en el botón tipo toggle del simulador
        self.inspection_etiqueta_simulador = self.builder.get_object("inspection_etiqueta_simulador")
        
        ## Ventana principal del HMI
        self.window = self.builder.get_object("ventana_principal")
        
        ## Ventana secundaria de selección de archivos del sistema
        self.window2 = self.builder.get_object("ventana_cfg")
        
        self.window.show()
        
        """
        a_COLUMNS = ["COL", "ROW", "TUBE"]
        
        inspection_tree_view = builder.get_object("inspection_tree_view")
        s_archivo_plan = "s_cfg_files_path/plan_inspeccion_"+str(ui_montaje)+".csv"
        a_plan_x, a_plan_y, a_tubos_id = leer_plan(s_archivo_plan)
        
        s_archivo_hx = "s_cfg_files_path/hx_3211.csv"
        
        # the data in the model (three strings for each row, one for each
        # column)
        lista_tubos = Gtk.ListStore(str, str, str)
        # append the values in the model
        for i in range(len(a_tubos_id)-30):
            lista_tubos.append([str(a_plan_x[i]), str(a_plan_y[i]), str(a_tubos_id[i])])
        inspection_tree_view.set_model(model=lista_tubos)
        
        for i, column in enumerate(a_COLUMNS):
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
    
    ##
    # @brief Función que implementa el botón "Apagar" en solapa Inicio
    # @param button Botón Apagar
    # @return none
    def apagar(self, button):
#         log.info("******************* Apagando sistema *******************")
#         buzzer(1, 10)
        Gtk.main_quit()
        os.system('sudo shutdown -h now')
        return
    
    ##
    # @brief Función que implementa el botón "Reiniciar" en solapa Inicio
    # @param button Botón Reiniciar
    # @return none
    def reiniciar(self, button):
#         log.info("******************* Reiniciando sistema *******************")
#         buzzer(1, 10)
        Gtk.main_quit()
        os.system('sudo reboot')
        return
            
    ##
    # @brief Detecta la fila seleccionada en el TreeView de solapa Inspection,
    # extrae la columna y fila deseada y usa el scrip leer_archivo_hx.py para
    # calcular P[x,y] de esa [col, row]. Finalmente calcula los ángulos de las
    # articulaciones POLE y ARM usando el script de cinemática inversa.
    # @param self Puntero al objeto hmi_SM13
    # @param selection Fila del Inspection GtkTreeView que se hace click.
    # @return none            
    def fila_plan_inspeccion(self, selection):
        
        # get the model and the iterator that points at the data in the model
        (model, iter) = selection.get_selected()
        # set the label to a new value depending on the selection
        
        self.ui_plan_col = int(model[iter][0])
        self.ui_plan_row = int(model[iter][1])
        self.s_tubo_id = model[iter][2]
        
        # Extrae las distancias x,y desde el archivo del hx (en pulgadas) según tubo seleccionado
        self.f_px_tubo, self.f_py_tubo, a_lista_px, a_lista_py, b_error_coordenada = leer_archivo_hx(self.ui_plan_col, self.ui_plan_row, self.s_archivo_hx)
        
        # Calcula los ángulos de los ejes POLE y ARM en base a la distancia que hay que alcanzar
        self.f_pole, self.f_arm = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La)   
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Se seleccionó "+self.s_tubo_id+" (fila:"+str(self.ui_plan_row)+", columna:"+str(self.ui_plan_col)+")")
        depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(np.rad2deg(self.f_pole))+"°"+", ARM = "+str(np.rad2deg(self.f_arm))+"°")
        depurador(1, "HMI", " ")      
        
        return True
    
    ##
    # @brief Función que se activa al presionar botón "Select Tube". Actualiza los ángulos
    # de las articulaciones POLE y ARM para enviar a RTU y refresca el simulador
    # en caso de que este último se encuentre activo.
    # @param self Puntero al objeto HMI
    # @param button Boton "Select Tube"
    # @return none
    def mover_a_seleccion(self, button):
        global simu
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Moviendo hacia "+self.s_tubo_id+" (fila:"+str(self.ui_plan_row)+", columna:"+str(self.ui_plan_col)+")")
        depurador(1, "HMI", " ")
        self.inspection_etiqueta_msg.set_text("Moving to "+self.s_tubo_id+" (ROW : "+str(self.ui_plan_row)+", COL : "+str(self.ui_plan_col)+")")
        
        if(self.b_simulador == 1):
            depurador(1, "HMI", " - Nuevo ángulo POLE = " + str(self.f_pole))
            depurador(1, "HMI", " - Nuevo ángulo ARM  = " + str(self.f_arm))
            simu.refrescar_grafico(self.f_px_tubo, self.f_py_tubo, self.f_pole, self.f_arm)
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Refrescando simulador SM-13")
            depurador(1, "HMI", " ")
            
        #HMIcomRTU(f_pole_cmd, f_arm_cmd)
        
        return True
        
    ##
    # @brief Función que se activa al presionar el botón toggle Simulator. Con un click se
    # crea el objeto que muestra el simulador. Otro click y se destruye el objeto.
    # @param self Puntero al objeto HMI
    # @param button Botón Simulator (to ON / to OFF)
    # @return none
    def run_simulador(self, button):
        global simu
         
        if (self.b_simulador == 0):
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", " - Inicia simulador SM-13")
            
            self.inspection_etiqueta_simulador.set_text("to OFF")
                
            self.f_px_tubo, self.f_py_tubo, a_lista_px, a_lista_py, b_error_coordenada = leer_archivo_hx(self.ui_plan_col, self.ui_plan_row, self.s_archivo_hx)
            depurador(1, "HMI", " - Simulador apuntando a x = " + str(self.f_px_tubo))
            depurador(1, "HMI", " - Simulador apuntando a y = " + str(self.f_py_tubo))
            
            simu = simulador_SM13(self.f_px_tubo, self.f_py_tubo, a_lista_px, a_lista_py, self.s_archivo_fixture, self.ui_montaje)
            
            depurador(1, "HMI", " - Simulador refrescando con qp = " + str(self.f_pole))
            depurador(1, "HMI", " - Simulador refrescando con qa = " + str(self.f_arm))
            simu.refrescar_grafico(self.f_px_tubo, self.f_py_tubo, self.f_pole, self.f_arm) 
            self.b_simulador = 1
            
            depurador(1, "HMI", " ")
            
            return True
            
        if (self.b_simulador == 1):
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", " - Se cierra simulador SM-13")
            depurador(1, "HMI", " ")
            del simu
            self.inspection_etiqueta_simulador.set_text("to ON")
            self.b_simulador = 0
            
            return True
    
    ##
    # @brief Función que se activa al presionar botón "Archivos" en solapa Inicio.
    # Crea una ventana secundaria para permitir seleccionar los archivos de cfg.
    # @param self Puntero al objeto HMI
    # @param button Botón "Archivos"
    # @return none
    def ventana_archivos(self, button):
        self.window2.show()        
        return True
    
    ##
    # @brief Función que se activa al presionar botón "Cerrar" en ventana
    # secundaria de selección de archivos de configuración. Permite esconder
    # la ventana cuando ya no se necesita.
    # @param self Puntero al objeto HMI
    # @param button Botón "Cerrar" en ventana archivos
    # @return none
    def cerrar_ventana_archivos(self, button):
        self.window2.hide()
        return True
    
    ##
    # @brief Función que captura la selección del tipo de telemanipulador
    # en el menu que se desprende en ventana de selección de archivos cfg.
    # @param self Puntero al objeto HMI
    # @param combobox Caja de selección de telemanipulador
    # @return none
    def seleccion_fixture(self, combobox):
        
        self.inspection_etiqueta_msg = self.builder.get_object("inspection_etiqueta_msg")
        self.inspection_etiqueta_msg.set_text("Listo para comenzar...")
        
        combo_fixture = combobox.get_active_iter()
        model_fixture = combobox.get_model()
        self.s_archivo_fixture = self.s_cfg_files_path+str(model_fixture[combo_fixture][0])+".csv"
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Archivo fixture: "+self.s_archivo_fixture)
        depurador(1, "HMI", " ")
        
        self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, trash, trash = leer_datos_SM13(self.s_archivo_fixture, self.ui_montaje)
        
        return True
    
    ##
    # @brief Función que captura la selección del tipo de intercambiador de calor
    # en el menu que se desprende en ventana de selección de archivos cfg.
    # @param self Puntero al objeto HMI
    # @param combobox Caja de selección de tipo de hx
    # @return none
    def seleccion_hx(self, combobox):
        
        combo_hx = combobox.get_active_iter()
        model_hx = combobox.get_model()
        self.s_archivo_hx = self.s_cfg_files_path+str(model_hx[combo_hx][0])+".csv"
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Archivo fixture: "+self.s_archivo_hx)
        depurador(1, "HMI", " ")
        
        return True
    
    ##
    # @brief Función que captura la selección del plan de inspección
    # en el menu que se desprende en ventana de selección de archivos cfg.
    # En base al plan elegido crea un objeto GtkTreeView para mostrar la lista
    # en la solapa Inspection.
    # @param self Puntero al objeto HMI
    # @param combobox Caja de selección de plan de inspección
    # @return none
    def seleccion_plan(self, combobox):

        a_plan_x = [0]
        a_plan_y = [0]
        a_tubos_id = [0]
        
        a_COLUMNS = ["COL", "ROW", "TUBE"]
               
        combo_plan = combobox.get_active_iter()
        model_plan = combobox.get_model()
        self.s_archivo_plan = self.s_cfg_files_path+str(model_plan[combo_plan][0])+".csv"
        a_plan_x, a_plan_y, a_tubos_id = leer_plan(self.s_archivo_plan)

        trash, trash, trash, trash, trash, trash, str_montaje = (model_plan[combo_plan][0]).split("_")
        self.ui_montaje = int(str_montaje)
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Archivo fixture: "+self.s_archivo_plan)
        depurador(1, "HMI", "- Montaje: "+ str(self.ui_montaje))
        depurador(1, "HMI", " ")
        
        inspection_tree_view = self.builder.get_object("inspection_tree_view")
               
        # the data in the model (three strings for each row, one for each
        # column)
        lista_tubos = Gtk.ListStore(str, str, str)
        
        # append the values in the model
        lista_tubos.clear()
        inspection_tree_view.set_model(model=lista_tubos)
        
        for i in range(len(a_tubos_id)):
            lista_tubos.append([str(a_plan_x[i]), str(a_plan_y[i]), str(a_tubos_id[i])])
            inspection_tree_view.set_model(model=lista_tubos)        
        
        for i, column in enumerate(a_COLUMNS):
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


if __name__ == "__main__":
    hmi_SM13()
    Gtk.main()
    
    