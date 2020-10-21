##
# Clase principal que levanta el archivo .glade y construye la
# interfaz gráfica HMI con sus diferentes funcionalidades para controlar
# y monitorear el telemanipulador SM-13
#
# @section TODO
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
from gi.repository import GLib, Gtk, Gdk, GObject
import os
import time
from simulador_SM13 import *
from leer_archivo_hx import *
from leer_plan import *
from leer_specs_tubos import *
from depurador import *
from gi.repository import Pango
import sys
import numpy as np
import time

class hmi_SM13():   
    global simu
    
    ## El constructor del HMI
    def __init__(self):
        # Atributos públicos de la clase hmi_SM13
        ## Contiene el punto X en pulgadas de la posición de la boquilla.
        self.f_px_tubo = 0.0
        ## Contiene el punto Y en pulgadas de la posición de la boquilla.
        self.f_py_tubo = 0.0
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
        ## Contiene la cantidad a incrementar por el Jog Control
        self.f_incremento_jog = 0.0
        ## Contiene el incremento acumulado por el jog control en columnas
        self.f_incremento_acumulado_jog_col = 0.00
        ## Contiene el incremento acumulado por el jog control en filas
        self.f_incremento_acumulado_jog_row = 0.00
        ## Contiene el valor del diámetro exterior de los tubos que forma el hx (varía según este)
        self.f_tube_od = 0.0
        ## Contiene el valor de la separación entre centro y centro de las filas de los tubos que forman el hx (varía según este)
        self.f_y_pitch = 0.0
        ## Contiene el valor de la separación entre centro y centro de las columnas de los tubos que forman el hx (varía según este)
        self.f_x_pitch = 0.0
        ## Contiene el tipo de montaje respecto al hx.
        # 0: centro en caso de no especificar hx ni montaje,
        # 1: plan de inspección 1,
        # 2: plan de inspección 2,
        # 3: plan de inspección 3.
        self.ui_montaje = 0
        ## Variable indica se activa si el motor se mueve pero no la lectura de enconder
        # False: sin alarmas de stall
        # True: alarma de stall
        self.b_stallAlm = False
        ## Contiene el estado del simulador
        # 0: simulador inactivo
        # 1: simulador activo
        self.b_simulador = 0
        ## Contiene la columna elegida en el mazo de tubos del hx
        self.ui_plan_col = 0
        ## Contiene la fila elegida en el mazo de tubos del hx
        self.ui_plan_row = 0
        ## Variable permite iniciar el TreeView una sola vez
        self.b_carga = 0
        ## Contiene el número de tubo elegido en el mazo de tubos del hx
        self.s_tubo_id = "TUBE.DEFAULT"
        ## Carga la lista de columnas del plan de inspección seleccionado
        self.a_plan_col = [0]
        ## Carga la lista de filas del plan de inspección seleccionado
        self.a_plan_row = [0]
        ## Carga la lista de tubos del plan de inspección seleccionado
        self.a_plan_tubos = [0]
        ## Carga la lista de puntos x en pulgadas del hx seleccionado
        self.a_lista_px = [0]
        ## Carga la lista de puntos y en pulgadas del hx seleccionado
        self.a_lista_py = [0]
        ## Variable que habilita/deshabilita el control del telemanipulador
        self.s_ctrlEn = "DISABLE_CONTROL"
        ## Variable que habilita/deshabilita la función stall del sistema
        self.s_stallEn = "STALL_ENABLE"
        ## Variable que indica el modo de operación del HMI
        # [STOP, FREE_RUN, LIFT, AUTOMATIC]
        self.s_mode = "STOP"
        ## Contiene la ruta al archivo xml de la interfaz
        self.s_gui_path = "/home/pi/Desktop/SM-13/hmi/gui/hmi_SM-13.glade"
        ## Contiene la ruta a la carpeta de archivos de configuración del sistema
        self.s_cfg_files_robots_path = "/home/pi/Desktop/SM-13/hmi/cfg_files/Robots/"
        ## Contiene la ruta a la carpeta de archivos de configuración del sistema
        self.s_cfg_files_hx_path = "/home/pi/Desktop/SM-13/hmi/cfg_files/Heat_exchangers/"
        ## Contiene la ruta al archivo con datos del telemanipulador.
        # por defecto se selecciona SM-13
        self.s_archivo_fixture = self.s_cfg_files_robots_path + "Zetec/SM-13.csv"
        ## Contiene la ruta al archivo con datos del intercambiador de calor
        self.s_archivo_hx = "none"
        ## Contiene la ruta al archivo que contiene el plan del inspección
        self.s_archivo_plan = "none"
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
            "evento_seleccion_plan": self.seleccion_plan,
            "evento_cargar_archivos": self.cargar_archivos,
            "evento_next_tube": self.siguiente_tubo,
            "evento_set_offset": self.offset,
            "evento_seleccion_incremento_off": self.seleccion_incremento_jog,
            "evento_seleccion_incremento_fino": self.seleccion_incremento_jog,
            "evento_seleccion_incremento_grueso": self.seleccion_incremento_jog,
            "evento_jog_right": self.jog_control,
            "evento_jog_left": self.jog_control,
            "evento_jog_up": self.jog_control,
            "evento_jog_down": self.jog_control,
            "evento_control_principal": self.fixture_control,
            "evento_stall": self.fixture_control,
            "evento_stop": self.detener_movimiento,
            "evento_red": self.ventana_red,
            "evento_cerrar_ventana_red": self.cerrar_ventana_red,
            "evento_modificar_red": self.modificar_red
        }
        self.builder.connect_signals(señales)
        
        ## Etiqueta pública de mensajes de la solapa Inicio
        self.inicio_etiqueta_msg = self.builder.get_object("inicio_etiqueta_msg")
        ## Etiqueta pública de mensajes de la solapa Free Run
        self.fr_etiqueta_msg = self.builder.get_object("fr_etiqueta_msg")
        ## Etiqueta pública de mensajes de la solapa Manual
        # TODO:self.manual_etiqueta_msg = self.builder.get_object("inspection_etiqueta_msg")
        ## Etiqueta pública de mensajes de la solapa Inspection
        self.inspection_etiqueta_msg = self.builder.get_object("inspection_etiqueta_msg")
     
        
        ## Etiqueta dinámica que se muestra en el botón tipo toggle del simulador
        self.inspection_etiqueta_boton_simulador = self.builder.get_object("inspection_etiqueta_boton_simulador")
        
        ## Etiqueta dinámica que muestra el jog acumulado en filas
        self.inspection_etiqueta_valor_jog_row = self.builder.get_object("inspection_etiqueta_valor_jog_row")
        self.inspection_etiqueta_valor_jog_row.set_text(str(float(self.f_incremento_acumulado_jog_row)))
        
        ## Etiqueta dinámica que muestra el jog acumulado en columnas
        self.inspection_etiqueta_valor_jog_col = self.builder.get_object("inspection_etiqueta_valor_jog_col")
        self.inspection_etiqueta_valor_jog_col.set_text(str(float(self.f_incremento_acumulado_jog_col)))
        
        ## Etiqueta dinámica que muestra el estado del control principal del telemanipulador
        self.inicio_etiqueta_estado_main_control = self.builder.get_object("inicio_etiqueta_estado_main_control")
        
        ## Etiqueta dinámica que muestra el estado de la función stall del telemanipulador
        self.inicio_etiqueta_estado_stall = self.builder.get_object("inicio_etiqueta_estado_stall")
        
        ## Ventana principal del HMI
        self.window = self.builder.get_object("ventana_principal")
        
        ## Ventana secundaria de selección de archivos del sistema
        self.window2 = self.builder.get_object("ventana_secundaria_archivos")
        
        ## Ventana secundaria para visualización o modificación de dirección de red
        self.window3 = self.builder.get_object("ventana_secundaria_red")
        
        ## Vista de arbol para el modo Inspection
        self.inspection_tree_view = self.builder.get_object("inspection_tree_view")
        
        ## Widget del interrupor de control principal (nos aseguramos que inicie desactivado)
        self.inicio_switch_fixture_control = self.builder.get_object("inicio_switch_fixture_control")
        self.inicio_switch_fixture_control.set_active(False)

        # Warning: deprecated!
        #self.window2.override_background_color(0, Gdk.RGBA(0.9,0.0,0.0,1.0))
        
        # Mensaje de bienvenida
        self.inicio_etiqueta_msg.set_text("Welcome to the NFC Human-Machine Interface!")
        self.window.show()      
        
    
    ##
    # @brief Función que implementa el botón "Apagar" en solapa Inicio
    # @param self Puntero al objeto HMI
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
    # @param self Puntero al objeto HMI
    # @param button Botón Reiniciar
    # @return none
    def reiniciar(self, button):
#         log.info("******************* Reiniciando sistema *******************")
#         buzzer(1, 10)
        Gtk.main_quit()
        os.system('sudo reboot')
        return
    
    
    ##
    # @brief Función que implementa los interruptores "Main control" y "Stall"
    # @param self Puntero al objeto HMI
    # @param switch "Main control" o "Stall Function"
    # @param state Estado del switch en cuestión
    # @return none
    def fixture_control(self, switch, state):
        
        # Obtiene el nombre del widget switch presionado
        s_name = switch.get_name()

        if (s_name == "fixture_control_switch"):    
            if switch.get_active():
                self.s_ctrlEn = "CONTROL_ENABLE"
                self.inicio_etiqueta_estado_main_control.set_text("Enabled")
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg("Fixture harness main control enabled...")
            else:
                self.s_ctrlEn = "DISABLE_CONTROL"
                self.inicio_etiqueta_estado_main_control.set_text("Disabled")
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg("Fixture harness main control disabled...")

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Fixture status = " + self.s_ctrlEn)
            depurador(1, "HMI", " ")
            
        elif (s_name == "fixture_stall_switch"):
            if switch.get_active():
                self.s_stallEn = "STALL_ENABLE"
                self.inicio_etiqueta_estado_stall.set_text("Enabled")
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg(msg="Fixture harness stall function enabled...")
            else:
                self.s_stallEn = "DISABLE_STALL"
                self.inicio_etiqueta_estado_stall.set_text("Disabled")
                self.actualizar_etiquetas_msg(msg="Fixture harness stall function disabled...")

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Fixture status = " + self.s_stallEn)
            depurador(1, "HMI", " ")
        
        return True
    
    ##
    # @brief Función que implementa todos los botones de STOP del HMI
    # @param self Puntero al objeto HMI
    # @param button "STOP"
    # @return none
    def detener_movimiento(self, button):
        # Desenergiza relé principal en NFC
        self.s_ctrlEn = "DISABLE_CONTROL"
        # Actualiza la variable modo
        self.s_mode = "STOP"
        # Cambia a OFF el widget del interruptor principal
        self.inicio_switch_fixture_control.set_active(False)
        # Cambia el mensaje de la etiqueta del interruptor
        self.inicio_etiqueta_estado_main_control.set_text("Disabled")
        ## Se indica la acción en las etiquetas principales de msg.
        self.actualizar_etiquetas_msg(msg="Stop, fixture harness main control disabled...")   
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- STOP TOTAL")
        depurador(1, "HMI", "- Fixture status = " + self.s_ctrlEn)
        depurador(1, "HMI", " ")
            
        return True
        
    ##
    # @brief Detecta la fila seleccionada en el TreeView de solapa Inspection,
    # extrae la columna y fila deseada y usa el scrip leer_archivo_hx.py para
    # calcular P[x,y] de esa [col, row]. Finalmente calcula los ángulos de las
    # articulaciones POLE y ARM usando el script de cinemática inversa.
    # @param self Puntero al objeto hmi_SM13
    # @param selection Fila del Inspection GtkTreeView que se hace click.
    # @return none            
    def fila_plan_inspeccion(self, selection):
        
        depurador(1, "HMI", "****************************************")
        # get the model and the iterator that points at the data in the model
        (model, iter) = selection.get_selected()
        # set the label to a new value depending on the selection
        try:
            self.ui_plan_col = int(model[iter][0])
            self.ui_plan_row = int(model[iter][1])
            self.s_tubo_id = model[iter][2]
            depurador(1, "HMI", "- Se seleccionó "+self.s_tubo_id+" (COL:"+str(self.ui_plan_col)+", ROW:"+str(self.ui_plan_row)+")")
        except:
            pass

        if (self.ui_plan_col != 0 and self.ui_plan_row != 0):
            # Extrae las distancias x,y desde el archivo del hx (en pulgadas) según tubo seleccionado
            self.f_px_tubo, self.f_py_tubo, self.a_lista_px, self.a_lista_py, b_error_coordenada = leer_archivo_hx(self.ui_plan_col, self.ui_plan_row, self.s_archivo_hx)
            # Se agrega el jog si lo hubiere
            depurador(1, "HMI", "- Jog aplicado a Px = " + str(round(self.f_incremento_acumulado_jog_col*self.f_x_pitch)) + " in")
            self.f_px_tubo -= self.f_incremento_acumulado_jog_col*self.f_x_pitch
            depurador(1, "HMI", "- Jog aplicado a Py = " + str(round(self.f_incremento_acumulado_jog_row*self.f_y_pitch)) + " in")
            self.f_py_tubo -= self.f_incremento_acumulado_jog_row*self.f_y_pitch
        else:
            # Posición inicial de la boquilla al arrancar el simulador 
            self.f_px_tubo = self.f_Lx - self.f_incremento_acumulado_jog_col*self.f_x_pitch
            self.f_py_tubo = self.f_Ly - self.f_Lp + self.f_La - self.f_incremento_acumulado_jog_row*self.f_y_pitch

        # Calcula los ángulos de los ejes POLE y ARM en base a la distancia que hay que alcanzar
        self.f_pole, self.f_arm = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La)
        
        
        depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(round(np.rad2deg(self.f_pole), 3))+"°"+", ARM = "+str(round(np.rad2deg(self.f_arm), 3))+"°")
        depurador(1, "HMI", "- Para ubicar boquilla en Px = "+str(round(self.f_px_tubo, 3))+" in"+", Py = "+str(round(self.f_py_tubo, 3))+" in")
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
        
        # Verificaciones previas al movimiento
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():
            return True
        
        # Antes de mover verifica si está activado el control principal
        if (self.s_ctrlEn == "DISABLE_CONTROL"):
            self.actualizar_etiquetas_msg("Fixture control is disabled...")
            return True
        
        # Antes de mover verifica si no está atascado el telemanipulador
        if (self.b_stallAlm):
            self.actualizar_etiquetas_msg("Fixture harness is stalled...")
            return True
        
        # TODO chequear ZS también
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Moviendo hacia "+ self.s_tubo_id + " (COL : "+ str(self.ui_plan_col) + ", ROW : "+str(self.ui_plan_row) + ")")
        depurador(1, "HMI", " ")
        self.inspection_etiqueta_msg.set_text("Moving to " + self.s_tubo_id + " (COL : "+ str(self.ui_plan_col) + ", ROW : "+str(self.ui_plan_row) + ")")
        
        if(self.b_simulador == 1):
            depurador(1, "HMI", " - Nuevo ángulo POLE = " + str(self.f_pole))
            depurador(1, "HMI", " - Nuevo ángulo ARM  = " + str(self.f_arm))
            simu.refrescar_grafico(self.f_px_tubo, self.f_py_tubo, self.f_pole, self.f_arm)
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Refrescando simulador SM-13")
            depurador(1, "HMI", " ")
            
        # TODO: enviar los nuevos ángulos a RTU
        
        return True
    
    ##
    # @brief Función que se activa al presionar botón "Next Tube". Identifica
    # la fila actual y selecciona la siguiente en el plan de inspección activo.
    # Luego Actualiza los ángulos de las articulaciones POLE y ARM para enviar
    # a RTU y refresca el simulador en caso de que este último se encuentre activo.
    # @param self Puntero al objeto HMI
    # @param button Boton "Next Tube"
    # @return none
    def siguiente_tubo(self, button):
        global simu
        
        # Verificaciones previas al movimiento
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():
            return True
        
        # Antes de mover verifica si está activado el control principal
        if (self.s_ctrlEn == "DISABLE_CONTROL"):
            self.actualizar_etiquetas_msg("Fixture control is disabled...")
            return True
        
        # Antes de mover verifica si no está atascado el telemanipulador
        if (self.b_stallAlm):
            self.actualizar_etiquetas_msg("Fixture harness is stalled...")
            return True
        
        # TODO chequear ZS también
        
        # Se obtiene el objeto de selección de la vista
        selection = self.inspection_tree_view.get_selection()
        # Se obtiene la fila actualmente seleccionada
        sel = selection.get_selected()
        if not sel[1] == None:
            # Se obtiene el próximo iter
            next = self.lista_tubos.iter_next(sel[1])
            if next:
                # Si hay un next, o sea retorna True la función anterior, se
                # la selecciona como la fila actual.
                selection.select_iter(next)

        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Moviendo hacia "+ self.s_tubo_id + " (COL : "+ str(self.ui_plan_col) + ", ROW : "+str(self.ui_plan_row) + ")")
        depurador(1, "HMI", " ")
        self.inspection_etiqueta_msg.set_text("Moving to " + self.s_tubo_id + " (COL : "+ str(self.ui_plan_col) + ", ROW : "+str(self.ui_plan_row) + ")")
        
        if(self.b_simulador == 1):
            depurador(1, "HMI", " - Nuevo ángulo POLE = " + str(self.f_pole))
            depurador(1, "HMI", " - Nuevo ángulo ARM  = " + str(self.f_arm))
            simu.refrescar_grafico(self.f_px_tubo, self.f_py_tubo, self.f_pole, self.f_arm)
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Refrescando simulador SM-13")
            depurador(1, "HMI", " ")
            
        # TODO: enviar los nuevos ángulos a RTU
        
        return True
        
        
    ##
    # @brief Función que se activa al presionar el botón toggle Simulator. Con un click se
    # crea el objeto que muestra el simulador. Otro click y se destruye el objeto.
    # @param self Puntero al objeto HMI
    # @param button Botón Simulator (to ON / to OFF)
    # @return none
    def run_simulador(self, button):
        global simu
        
        # no arranco el simulador si no se han cargado archivos de configuración antes
        if not self.archivos():
            return True
         
        if (self.b_simulador == 0):
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", " - Inicia simulador SM-13")
            
            self.inspection_etiqueta_boton_simulador.set_text("to OFF")

            if (self.ui_plan_col != 0 and self.ui_plan_row != 0):
                self.f_px_tubo, self.f_py_tubo, self.a_lista_px, self.a_lista_py, b_error_coordenada = leer_archivo_hx(self.ui_plan_col, self.ui_plan_row, self.s_archivo_hx)
                # Se agrega el jog si lo hubiere
                depurador(1, "HMI", "- Jog aplicado a Px = " + str(round(self.f_incremento_acumulado_jog_col*self.f_x_pitch)) + " in")
                self.f_px_tubo -= self.f_incremento_acumulado_jog_col*self.f_x_pitch
                depurador(1, "HMI", "- Jog aplicado a Py = " + str(round(self.f_incremento_acumulado_jog_row*self.f_y_pitch)) + " in")
                self.f_py_tubo -= self.f_incremento_acumulado_jog_row*self.f_y_pitch
            else:
                # Posición inicial de la boquilla al arrancar el simulador 
                self.f_px_tubo = self.f_Lx - self.f_incremento_acumulado_jog_col*self.f_x_pitch
                self.f_py_tubo = self.f_Ly - self.f_Lp + self.f_La - self.f_incremento_acumulado_jog_row*self.f_y_pitch
                self.a_lista_px = [0]
                self.a_lista_py = [0]
                
            depurador(1, "HMI", " - Simulador apuntando a x = " + str(self.f_px_tubo))
            depurador(1, "HMI", " - Simulador apuntando a y = " + str(self.f_py_tubo))
            
            simu = simulador_SM13(self.f_px_tubo , self.f_py_tubo, self.s_archivo_fixture, self.ui_montaje, self.s_archivo_hx)
            
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
            self.inspection_etiqueta_boton_simulador.set_text("to ON")
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
        
        combo_fixture = combobox.get_active_iter()
        model_fixture = combobox.get_model()
        self.s_archivo_fixture = self.s_cfg_files_robots_path+str(model_fixture[combo_fixture][0])+".csv"
        depurador(2, "HMI", "****************************************")
        depurador(2, "HMI", "- Archivo fixture: "+self.s_archivo_fixture)
        depurador(2, "HMI", " ")
        
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
        self.s_archivo_hx = self.s_cfg_files_hx_path+str(model_hx[combo_hx][0])+".csv"
        depurador(2, "HMI", "****************************************")
        depurador(2, "HMI", "- Archivo hx: "+self.s_archivo_hx)
        depurador(2, "HMI", " ")
        
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
                
        self.a_plan_col = [0]
        self.a_plan_row = [0]
        self.a_plan_tubos = [0]
        
        combo_plan = combobox.get_active_iter()
        model_plan = combobox.get_model()
        self.s_archivo_plan = self.s_cfg_files_hx_path+str(model_plan[combo_plan][0])+".csv"
        self.a_plan_col, self.a_plan_row, self.a_plan_tubos = leer_plan(self.s_archivo_plan)

        trash, trash, trash, trash, trash, trash, str_montaje = (model_plan[combo_plan][0]).split("_")
        self.ui_montaje = int(str_montaje)
        
        depurador(2, "HMI", "****************************************")
        depurador(2, "HMI", "- Plan de Inspección: "+self.s_archivo_plan)
        depurador(2, "HMI", "- Montaje: "+ str(self.ui_montaje))
        depurador(2, "HMI", " ")
            
        return True
    
    ##
    # @brief Función que implementa el botón "Aceptar" en ventana de configuración
    # de archivos. Luego crea el GtkTreeView con la lista del plan de inspección en
    # solapa Inspection.
    # @param self Puntero al objeto HMI
    # @param button Botón Load
    # @return none
    def cargar_archivos(self, button):
        global simu
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Archivo hx        : "+self.s_archivo_hx)
        depurador(1, "HMI", "- Plan de Inspección: "+self.s_archivo_plan)
        depurador(1, "HMI", "- Tipo Fixture      : "+self.s_archivo_fixture)
        depurador(1, "HMI", " ")
        
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Faltan cargar archivos...")
            depurador(1, "HMI", " ")
            return True
        
        a_COLUMNS = ["COL", "ROW", "TUBE"]
        
        #inspection_tree_view = self.builder.get_object("inspection_tree_view")
                
        # the data in the model (three strings for each row, one for each
        # column)
        self.lista_tubos = Gtk.ListStore(str, str, str)
        self.lista_tubos.clear()
        
        # append the values in the model                
        for i in range(len(self.a_plan_tubos)):
            self.lista_tubos.append([str(self.a_plan_col[i]), str(self.a_plan_row[i]), str(self.a_plan_tubos[i])])
            self.inspection_tree_view.set_model(model=self.lista_tubos)
        
        if (self.b_carga == 0):
        
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
                self.inspection_tree_view.append_column(col)
        
        # Se actualiza posición del fixture en caso de reelección de plan de inspección
        self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, trash, trash = leer_datos_SM13(self.s_archivo_fixture, self.ui_montaje)
        
        # Se resetea simulador ya que cambió el plan de inspección y, por lo tanto, la posición del FH.
        if (self.b_simulador == 1):
            del simu
            simu = simulador_SM13(self.f_px_tubo, self.f_py_tubo, self.s_archivo_fixture, self.ui_montaje, self.s_archivo_hx)
            simu.refrescar_grafico(self.f_px_tubo, self.f_py_tubo, self.f_pole, self.f_arm)
            
        # Se establecen las especificaciones del mazo de tubos en base al intercambiador elegido
        # Si se cambia el path, procurar que tenga la misma cantidad de carpetas
        trash, s_folder1, s_folder2, s_folder3, s_folder4, s_folder5, s_folder6, s_folder7, s_folder8, trash = self.s_archivo_hx.split("/")
        # Se identifica el path al archivo y se lo lee
        s_tube_specs_path = "/"+s_folder1+"/"+s_folder2+"/"+s_folder3+"/"+s_folder4+"/"+s_folder5+"/"+s_folder6+"/"+s_folder7+"/"+s_folder8+"/tube_specs.csv"
        a_specs_tube = leer_specs_tubos(s_tube_specs_path)
        self.f_tube_od = float(a_specs_tube[0])
        self.f_y_pitch = float(a_specs_tube[1])
        self.f_x_pitch = float(a_specs_tube[2])
        
        if (self.s_ctrlEn == "CONTROL_ENABLE"):
            self.inspection_etiqueta_msg = self.builder.get_object("inspection_etiqueta_msg")
            self.inspection_etiqueta_msg.set_text("Ready to go...")
        
        # permite que las columnas del TreeView se creen una sola vez
        self.b_carga = 1
        
        return True
    
    
    ##
    # @brief Función que implementa la selección de los radioButtons de incremento
    # de Jog.
    # @param self Puntero al objeto HMI
    # @param button Botones radio fino/grueso
    # @return none
    def seleccion_incremento_jog(self, button):
        if button.get_active():
            if (button.get_label() == "off"):
                self.f_incremento_jog = 0.0
            else:
                self.f_incremento_jog = float(button.get_label())
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Incremento jog: "+str(self.f_incremento_jog))
            depurador(1, "HMI", " ")     
        
        return True
    
    ##
    # @brief Función que implementa la sección Jog Control
    # @param self Puntero al objeto HMI
    # @param button Boton Pueden ser cuatro: jog_to_north, jog_to_east, jog_to_south, jog_to_west
    # @return none
    def jog_control(self, button):
        # Verificaciones previas al movimiento
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():
            return True
        
        # Antes de mover verifica si está activado el control principal
        if (self.s_ctrlEn == "DISABLE_CONTROL"):
            self.actualizar_etiquetas_msg("Fixture control is disabled...")
            return True
        
        # Antes de mover verifica si no está atascado el telemanipulador
        if (self.b_stallAlm):
            self.actualizar_etiquetas_msg("Fixture harness is stalled...")
            return True
        
        # Obtiene el nombre del boton jog presionado
        s_boton_jog = button.get_name()
        
        if (s_boton_jog == "jog_to_east"):
            self.f_px_tubo += self.f_incremento_jog*self.f_x_pitch
            self.f_incremento_acumulado_jog_col += self.f_incremento_jog
            
        elif (s_boton_jog == "jog_to_west"):
            self.f_px_tubo -= self.f_incremento_jog*self.f_x_pitch
            self.f_incremento_acumulado_jog_col -= self.f_incremento_jog
            
        if (s_boton_jog == "jog_to_north"):
            self.f_py_tubo -= self.f_incremento_jog*self.f_y_pitch
            self.f_incremento_acumulado_jog_row += self.f_incremento_jog
        
        elif (s_boton_jog == "jog_to_south"):
            
            self.f_py_tubo += self.f_incremento_jog*self.f_y_pitch
            self.f_incremento_acumulado_jog_row -= self.f_incremento_jog
            
        try:
            self.inspection_etiqueta_valor_jog_row.set_text(str(round(self.f_incremento_acumulado_jog_row, 2)))
            self.inspection_etiqueta_valor_jog_col.set_text(str(round(self.f_incremento_acumulado_jog_col, 2)))
        except:
            pass
            
        # Calcula los ángulos de los ejes POLE y ARM en base al corrimiento
        self.f_pole, self.f_arm = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La)   
            
        if (self.b_simulador == 1):
            # Se refresca simulador ya que se está haciendo un ajuste fino de la boquilla.
            # no se suma el jog porque ya se lo hace arriba en esta misma función.
            simu.refrescar_grafico(self.f_px_tubo, self.f_py_tubo, self.f_pole, self.f_arm)
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Ajustando fino con incremento jog: "+str(self.f_incremento_jog))
        depurador(1, "HMI", "- Jog acumulado en COL: "+str(self.f_incremento_acumulado_jog_col))
        depurador(1, "HMI", "- Jog acumulado en ROW: "+str(self.f_incremento_acumulado_jog_row))
        depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(round(np.rad2deg(self.f_pole), 3))+"°"+", ARM = "+str(round(np.rad2deg(self.f_arm), 3))+"°")
        depurador(1, "HMI", "- Para ubicar boquilla en Px = "+str(round(self.f_px_tubo, 3))+" in"+", Py = "+str(round(self.f_py_tubo, 3))+" in")
        depurador(1, "HMI", " ") 
        
        # TODO: enviar los nuevos ángulos a RTU
        
        return True
    
    ##
    # @brief Función que implementa el offset en cuentas de resolver
    # para ángulos de POLE y ARM 0.
    # @param self Puntero al objeto HMI
    # @param button Boton Set Offset
    # @return none
    def offset(self, button):
        
        return True
    
    ##
    # @brief Función que indica si se han cargado archivos de configuración o no
    # @param self puntero al objeto HMI
    # @return True o False si se han cargado o no los tres archivos de configuración
    def archivos(self):

        if (self.s_archivo_hx == 'none' or self.s_archivo_fixture == 'none' or self.s_archivo_plan == 'none'):
            self.inspection_etiqueta_msg.set_text("No configuraton files has been loaded...")
            return False
        else:
            return True
    
    ##
    # @brief Función que optimiza la actualización de mensajes generales
    # simultáneamente en todas las etiquetas de msg de las diferentes solapas
    # @param self puntero al objeto HMI
    # @param msg Mensaje general a imprimir en todas las etiquetas de las solapas
    # @return none
    def actualizar_etiquetas_msg(self, msg): 
        self.inicio_etiqueta_msg.set_text(msg)
        self.inspection_etiqueta_msg.set_text(msg)
        self.fr_etiqueta_msg.set_text(msg)
        #self.manual_etiqueta_msg.set_text(msg)
        
        return True
    
    ##
    # @brief Función que levanta la ventana de configuración de red
    # para mostrar la dirección IP y puerto actuales o para modificarlos.
    # @param self puntero al objeto HMI
    # @param button Botón de red en ventana inicio
    # @return none
    def ventana_red(self, button):
        self.window3.show()
        return True
    
    
    ##
    # @brief Función que se activa al presionar botón "Cerrar" en ventana
    # secundaria de configuración de red. Permite esconder la ventana
    # cuando ya no se necesita.
    # @param self Puntero al objeto HMI
    # @param button Botón "Cerrar" en ventana de red.
    # @return none
    def cerrar_ventana_red(self, button):
        self.window3.hide()
        return True
    
    ##
    # @brief Función que implementa el botón "Modificar" en ventana de configuración
    # de red. Luego actualiza el archivo de texto con la nueva dirección IP y puerto.
    # @param self Puntero al objeto HMI
    # @param button Botón "modificar"
    # @return none
    def modificar_red(self, button):
        
        ## Entradas para modificación de dirección IP y Puerto de red
        #entrada_ip_0 = self.builder.get_object("red_entrada_ip_0")
        #entrada_ip_1 = self.builder.get_object("red_entrada_ip_1")
        #entrada_ip_2 = self.builder.get_object("red_entrada_ip_2")
        entrada_ip_0 = self.builder.get_object("red_boton_spin_ip_0")
        entrada_ip_1 = self.builder.get_object("red_boton_spin_ip_1")
        entrada_ip_2 = self.builder.get_object("red_boton_spin_ip_2")
        entrada_ip_3 = self.builder.get_object("red_boton_spin_ip_3")
        entrada_puerto = self.builder.get_object("red_boton_spin_puerto")
        
        self.ui_ip_0 = int(entrada_ip_0.get_value())
        self.ui_ip_1 = int(entrada_ip_1.get_value())
        self.ui_ip_2 = int(entrada_ip_2.get_value())
        self.ui_ip_3 = int(entrada_ip_3.get_value())
        self.ui_puerto = int(entrada_puerto.get_value())
     
        print("ip3: "+str(self.ui_ip_3))
        print("ip2: "+str(self.ui_ip_2))
        print("ip1: "+str(self.ui_ip_1))
        print("ip0: "+str(self.ui_ip_0))
        print("puerto: "+str(self.ui_puerto))
        
        return True
        

if __name__ == "__main__":
    hmi_SM13()
    Gtk.main()
    
    