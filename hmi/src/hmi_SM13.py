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
from HMIcomRTU import *
import socket
import zumbador

class hmi_SM13():   
    global simu
    
    ## El constructor del HMI
    def __init__(self):
        # CONSTANTES DEL SISTEMA
        ## Constante contiene el tiempo para actualizar etiquetas del reloj
        self.ui_REFRESCO_ms_RELOJ = 5000
        ## Constante contiene el tiempo entre tramas de telemetría
        self.ui_REFRESCO_ms_TELEMETRIA = 1000
        ## Constante contiene el tiempo de refresco de valores actuales de ángulos POLE y ARM del 
        # simulador
        self.ui_REFRESCO_ms_SIMULADOR = 1000
        ## Constante indica el tiempo en que una variable queda activada antes
        # de pasar el modo a STOP otra vez
        self.ui_DELAY_ms_PULSADOR = 1200
        ## Constante indica cada cuantos refrescos de telemetría se hace un intento de conexión
        self.ui_REINTENTAR = 10
        ## Constante grados máximos de una vuelta de ARM o POLE
        self.f_MAX_GRADOS = 359.999
        ## Constante máximo valor de conversión del RDC (16 bits)
        self.ui_MAX_CUENTAS = 65536       
        # ATRIBUTOS PÚBLICOS DE LA CLASE PRINCIPAL
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
        self.f_pole = np.deg2rad(270.0)
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
        ## Variable de tipo float contiene la posición, en grados, a la que se desea comandar el FH-ARM
        self.f_posCmdArm = 0.0
        ## Variable de tipo float contiene la posición, en grados, a la que se desea comandar el FH-POLE
        self.f_posCmdPole = 270.0
        ## Variable de tipo float contiene la posición, en radianes, de la articulación FH-ARM
        self.f_posActArm = 0.0
        ## Variable de tipo float contiene la posición, en radianes, de la articulación FH-POLE
        self.f_posActPole = np.deg2rad(270.0)
        ## Variable indica la velocidad con la que mueve la articulación ARM.
        self.f_velActArm = 0.0
        ## Variable indica la velocidad con la que mueve la articulación POLE.
        self.f_velActPole = 0.0
        ## Variable para evitar que se hagan intentos de conexión tan seguidos cuando no
        # hay comunicación HMI-RTU.
        self.ui_intentos = self.ui_REINTENTAR
        ## Contiene el tipo de montaje respecto al hx.
        # 0: centro en caso de no especificar hx ni montaje,
        # 1: plan de inspección 1,
        # 2: plan de inspección 2,
        # 3: plan de inspección 3.
        self.ui_montaje = 0
        ## Contiene el 1er byte de la dirección IP que apunta a RTU.
        #Se inicializa en 0 pero el sistema la actualiza al
        # arranque cuando lee el archivo network.csv.
        self.ui_ip_0 = 0
        ## Contiene 2do byte de la dirección IP
        self.ui_ip_1 = 0
        ## Contiene 3er byte de la dirección IP
        self.ui_ip_2 = 0
        ## Contiene 4to byte de la dirección IP
        self.ui_ip_3 = 0
        ## Contiene la dirección del puerto.
        self.ui_puerto = 0
        ## Contiene la columna elegida en el mazo de tubos del hx
        self.ui_plan_col = 0
        ## Contiene la fila elegida en el mazo de tubos del hx
        self.ui_plan_row = 0
        ## Variable contiene la velocidad deseada con que se quiere mover el FH-ARM en modo FR.
        self.ui_velCmdArm = 0
        ## Variable contiene la velocidad deseada con que se quiere mover el FH-POLE en modo FR.
        self.ui_velCmdPole = 0
        ## Variable contiene el valor actual deL RDC del eje POLE 
        self.ui_rdcActPole = 0
        ## Variable contiene el valor actual deL RDC del eje ARM 
        self.ui_rdcActArm = 0
        ## Variable contiene el valor comandado para el eje POLE 
        self.ui_rdcCmdPole = 0
        ## Variable contiene el valor comandado para el eje ARM 
        self.ui_rdcCmdArm = 0
        ## Variable contiene el valor de offset para el eje POLE configurado al inicio cuando levanta
        # el archivo robots_home_offsets.csv 
        self.ui_pole_rdc_offset = 0
        ## Variable contiene el valor de offset para el eje ARM configurado al inicio cuando levanta
        # el archivo robots_home_offsets.csv 
        self.ui_arm_rdc_offset = 0
        ## Variable indica se activa si el motor se mueve pero no la lectura de enconder
        # False: sin alarmas de stall
        # True: alarma de stall
        self.b_stallAlm = False
        ## Contiene el estado del simulador
        # 0: simulador inactivo
        # 1: simulador activo
        self.b_simulador = 0
        ## Variable booleana indica el estado del ZS superior del eje LIFT
        self.b_limitUp = False
        ## Variable booleana indica el estado del ZS inferior del eje LIFT
        self.b_limitDwn = False
        ## Variable tipo boleana que indica el estado de la conexión con la RTU
        # 0: Desconexión
        # 1: Conexión
        self.b_connect = False
        ## Variable permite iniciar el TreeView una sola vez
        self.b_carga = 0
        ## Variable acusa alarma de límite de movimiento horario en eje ARM (vía software)
        self.b_cwLimitArm = False
        ## Variable acusa alarma de límite de movimiento anti-horario en eje ARM (vía software)
        self.b_ccwLimitArm = False
        ## Variable acusa alarma de límite de movimiento horario en eje POLE (vía software)
        self.b_cwLimitPole = False
        ## Variable acusa alarma de límite de movimiento anti-horario en eje POLE (vía software)
        self.b_ccwLimitPole = False
        ## Variable acusa alarma de final de carrera superior en eje LIFT (vía ZS)
        self.b_limitUp = False
        ## Variable acusa alarma de final de carrera inferior en eje LIFT (vía ZS)
        self.b_limitDown = False
        ## Variable booleana para permitir o rechazar sonidos de la interfaz
        self.b_beeps = True
        ## Variable booleana indica si los càlculos de cinemática inversa fueron exitosos o no
        self.b_ik_success = False
        ## Variable 
        self.ui_status = 0
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
        ## Variable contiene el tipo de solución matemática que va a usar el
        # algoritmo de cinemática inversa para que el quiebre del ARM se de la
        # forma principal o alternativa.
        self.s_pivot_type = "main"
        ## Variable que indica el modo de operación del HMI
        # [STOP, FREE_RUN, LIFT, AUTOMATIC]
        self.s_mode = "STOP"
        ## Variable indica la actividad del eje LIFT
        self.s_liftDir = "LIFT_UP"
        ## Variable indica el eje del FH
        # Puede valer ARM o POLE
        self.s_freeRunAxis = "ARM"
        ## Variable indica el sentido de giro con la que se desea mover las articulaciones
        # ARM o POLE del FH.
        # Puede valer DIR_CW o CCW_DIR
        self.s_freeRunDir = "DIR_CW"
        ## Contiene la ruta a la carpeta SM-13.
        # Cambiar esta línea para correr el HMI en otro sistema y
        # procurar que la ruta tenga la misma cantidad de carpetas, ej: /Carpeta1/Carpeta2/Carpeta3/SM-13
        # Verificar que la ruta a la carpeta de sources queda .../SM-13/hmi/src/...
        self.s_project_path = "/home/gspc/Proyectos/SM-13"
        ## Contiene la ruta al archivo xml de la interfaz
        self.s_gui_path = self.s_project_path + "/hmi/gui/hmi_SM-13.glade"
        ## Contiene la ruta a la carpeta de archivos de configuración del sistema
        self.s_cfg_files_robots_path = self.s_project_path + "/hmi/cfg_files/Robots/"
        ## Contiene la ruta a la carpeta de archivos de configuración del sistema
        self.s_cfg_files_hx_path = self.s_project_path + "/hmi/cfg_files/Heat_exchangers/"
        ## Contiene la ruta al archivo con datos del telemanipulador.
        # por defecto se selecciona SM-13
        self.s_archivo_fixture = self.s_cfg_files_robots_path + "Zetec/SM-13.csv"
        ## Contiene la ruta al archivo con datos del intercambiador de calor
        self.s_archivo_hx = "none"
        ## Contiene el nombre del HX
        self.s_hx_type = 'none'
        ## Contiene la ruta al archivo que contiene el plan del inspección
        self.s_archivo_plan = "none"
        ## Crea el objeto Gtk para levantar la interfáz gráfica
        self.builder = Gtk.Builder()
        # Levanta la interfáz gráfica desde el archivo especificado
        self.builder.add_from_file(self.s_gui_path)
        
        ## Variable tipo <socket> que almacena la identidad del socket 
        # generado en la conexión establecida con la RTU.
        self.s_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ## Variable usada para guardar la dirección IP de la RTU
        self.s_ip = "0.0.0.0"
        ## Variable usada para guardar el puerto de conexión con la RTU
        self.s_port = "0"
        
        # Se abre el archivo network.csv en modo lectura, se lo lee y se cierra. 
        network_file = open(self.s_project_path + "/hmi/cfg_files/network.csv", "r")
        address_lines = network_file.readlines()
        network_file.close()
        # Se extraen la variables que contendrán la dir IP y el PUERTO
        self.s_ip, self.s_port = address_lines[1].split(";")
        
        ## Array con las variables tipo float y enteras a enviar a RTU
        self.a_HMIDataByte = self.f_posCmdArm, self.f_posCmdPole, self.ui_velCmdArm, self.ui_velCmdPole
        ## Array con las variables tipo string a enviar a RTU
        self.a_HMIDataString = self.s_mode, self.s_freeRunAxis, self.s_freeRunDir, self.s_ctrlEn, self.s_stallEn, self.s_liftDir
        
        
        # Todas las señales que manejan los eventos que surgen de presionar botones en la HMI.
        señales = {
            "terminar_aplicacion":Gtk.main_quit,
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
            "evento_set_offset": self.cal_home_offset,
            "evento_seleccion_incremento_off": self.seleccion_incremento_jog,
            "evento_seleccion_incremento_fino": self.seleccion_incremento_jog,
            "evento_seleccion_incremento_grueso": self.seleccion_incremento_jog,
            "evento_jog_right": self.jog_control,
            "evento_jog_left": self.jog_control,
            "evento_jog_up": self.jog_control,
            "evento_jog_down": self.jog_control,
            "evento_control_principal": self.fixture_control,
            "evento_stall": self.fixture_control,
            "evento_stop": self.stop_total,
            "evento_red": self.ventana_red,
            "evento_cerrar_ventana_red": self.cerrar_ventana_red,
            "evento_modificar_red": self.modificar_red,
            "evento_lift_up": self.control_lift,
            "evento_lift_down": self.control_lift,
            "evento_lift_down_total": self.control_lift,
            "evento_lift_up_total": self.control_lift,
            "evento_pole_cw_push": self.control_free_run,
            "evento_pole_ccw_push": self.control_free_run,
            "evento_arm_cw_push": self.control_free_run,
            "evento_arm_ccw_push": self.control_free_run,
            "evento_pole_cw_toggle": self.control_free_run,
            "evento_pole_ccw_toggle": self.control_free_run,
            "evento_arm_cw_toggle": self.control_free_run,
            "evento_arm_ccw_toggle": self.control_free_run,
            "evento_toggle_zumbador": self.control_zumbador,
            "evento_mover_tubo_manual": self.mover_a_seleccion,
            "evento_boton_manual_down_right": self.control_manual,
            "evento_boton_manual_down_left": self.control_manual,
            "evento_boton_manual_up_left": self.control_manual,
            "evento_boton_manual_up_right": self.control_manual,
            "evento_boton_manual_up": self.control_manual,
            "evento_boton_manual_down": self.control_manual,
            "evento_boton_manual_left": self.control_manual,
            "evento_boton_manual_right": self.control_manual,
            "evento_stow": self.fixture_control,
            "evento_pivot": self.fixture_control
        }
        self.builder.connect_signals(señales)
        
        self.entrada_ip_0 = self.builder.get_object("red_boton_spin_ip_0")
        self.entrada_ip_1 = self.builder.get_object("red_boton_spin_ip_1")
        self.entrada_ip_2 = self.builder.get_object("red_boton_spin_ip_2")
        self.entrada_ip_3 = self.builder.get_object("red_boton_spin_ip_3")
        self.entrada_puerto = self.builder.get_object("red_boton_spin_puerto")
        
        # Se descompone la dir ip
        s_ip_3, s_ip_2, s_ip_1, s_ip_0 = self.s_ip.split(".")
        # y se muestran en el "entry" de la ventana de cfg de red.
        self.entrada_ip_0.set_value(int(s_ip_0))
        self.entrada_ip_1.set_value(int(s_ip_1))
        self.entrada_ip_2.set_value(int(s_ip_2))
        self.entrada_ip_3.set_value(int(s_ip_3))
        self.entrada_puerto.set_value(int(self.s_port))
        
        ## Etiqueta pública de mensajes de la solapa Inicio
        self.inicio_etiqueta_msg = self.builder.get_object("inicio_etiqueta_msg")
        ## Etiqueta pública de mensajes de la solapa Free Run
        self.fr_etiqueta_msg = self.builder.get_object("fr_etiqueta_msg")
        ## Etiqueta pública de mensajes de la solapa Manual
        self.manual_etiqueta_msg = self.builder.get_object("manual_etiqueta_msg")
        ## Etiqueta pública de mensajes de la solapa Inspection
        self.inspection_etiqueta_msg = self.builder.get_object("inspection_etiqueta_msg")

        # DEFINICIONES DE ETIQUETAS DE HORA
        ## Etiqueta de actualización de hora en solapa Inicio
        self.inicio_etiqueta_hora = self.builder.get_object("inicio_etiqueta_hora")
        ## Etiqueta de actualización de hora en solapa Free Run
        self.fr_etiqueta_hora = self.builder.get_object("fr_etiqueta_hora")
        # Etiqueta de actualización de hora en solapa Manual
        self.manual_etiqueta_hora = self.builder.get_object("manual_etiqueta_hora")
        # Etiqueta de actualización de hora en solapa Inspection
        self.inspection_etiqueta_hora = self.builder.get_object("inspection_etiqueta_hora")

        # DEFINICIONES DE ETIQUETAS DE FECHA
        ## Etiqueta de actualización de fecha en solapa Inicio
        self.inicio_etiqueta_fecha = self.builder.get_object("inicio_etiqueta_fecha")
        ## Etiqueta de actualización de fecha en solapa Free Run
        self.fr_etiqueta_fecha = self.builder.get_object("fr_etiqueta_fecha")
        # ## Etiqueta de actualización de fecha en solapa Manual
        self.manual_etiqueta_fecha = self.builder.get_object("manual_etiqueta_fecha")
        ## Etiqueta de actualización de fecha en solapa Inspection
        self.inspection_etiqueta_fecha = self.builder.get_object("inspection_etiqueta_fecha")
        
        ## Etiqueta dinámica que muestra el estado del simulador en solapa Inicio
        self.inicio_etiqueta_estado_simulador = self.builder.get_object("inicio_etiqueta_estado_simulador")
        
        ## Etiqueta dinámica que muestra el jog acumulado en filas en solapa Inspection
        self.inspection_etiqueta_valor_jog_row = self.builder.get_object("inspection_etiqueta_valor_jog_row")
        self.inspection_etiqueta_valor_jog_row.set_text(str(float(self.f_incremento_acumulado_jog_row)))
        
        ## Etiqueta dinámica que muestra el jog acumulado en filas en solapa Manual
        self.manual_etiqueta_valor_jog_row = self.builder.get_object("manual_etiqueta_valor_jog_row")
        self.manual_etiqueta_valor_jog_row.set_text(str(float(self.f_incremento_acumulado_jog_row)))

        ## Etiqueta dinámica que muestra el jog acumulado en columnas en solapa Inspection
        self.inspection_etiqueta_valor_jog_col = self.builder.get_object("inspection_etiqueta_valor_jog_col")
        self.inspection_etiqueta_valor_jog_col.set_text(str(float(self.f_incremento_acumulado_jog_col)))

        ## Etiqueta dinámica que muestra el jog acumulado en columnas en solapa Manual
        self.manual_etiqueta_valor_jog_col = self.builder.get_object("manual_etiqueta_valor_jog_col")
        self.manual_etiqueta_valor_jog_col.set_text(str(float(self.f_incremento_acumulado_jog_col)))
        
        ## Etiqueta dinámica que muestra el estado del control principal del telemanipulador
        self.inicio_etiqueta_estado_main_control = self.builder.get_object("inicio_etiqueta_estado_main_control")
        
        ## Etiqueta dinámica que muestra el estado de la función stall del telemanipulador
        self.inicio_etiqueta_estado_stall = self.builder.get_object("inicio_etiqueta_estado_stall")
        
        ## Etiquetas dinámicas que muestran el valor de encoder del eje ARM
        self.fr_etiqueta_valor_enc_arm = self.builder.get_object("fr_etiqueta_valor_enc_arm")
        self.manual_etiqueta_valor_enc_arm = self.builder.get_object("manual_etiqueta_valor_enc_arm")
        self.inspection_etiqueta_valor_enc_arm = self.builder.get_object("inspection_etiqueta_valor_enc_arm")
        
        ## Etiquetas dinámicas que muestran el valor de ángulo del eje ARM
        self.fr_etiqueta_valor_ang_arm = self.builder.get_object("fr_etiqueta_valor_ang_arm")
        self.manual_etiqueta_valor_ang_arm = self.builder.get_object("manual_etiqueta_valor_ang_arm")
        self.inspection_etiqueta_valor_ang_arm = self.builder.get_object("inspection_etiqueta_valor_ang_arm")
        
        ## Etiquetas dinámicas que muestran el valor de encoder del eje POLE
        self.fr_etiqueta_valor_enc_pole = self.builder.get_object("fr_etiqueta_valor_enc_pole")
        self.manual_etiqueta_valor_enc_pole = self.builder.get_object("manual_etiqueta_valor_enc_pole")
        self.inspection_etiqueta_valor_enc_pole = self.builder.get_object("inspection_etiqueta_valor_enc_pole")
        
        ## Etiquetas dinámicas que muestran el valor de ángulo del eje POLE
        self.fr_etiqueta_valor_ang_pole = self.builder.get_object("fr_etiqueta_valor_ang_pole")
        self.manual_etiqueta_valor_ang_pole = self.builder.get_object("manual_etiqueta_valor_ang_pole")
        self.inspection_etiqueta_valor_ang_pole = self.builder.get_object("inspection_etiqueta_valor_ang_pole")

        ## Etiqueta del botón de activación/desactivación del zumbador
        self.inicio_etiqueta_toggle_zumbador = self.builder.get_object("inicio_etiqueta_toggle_zumbador")

        ## Etiqueta de COL actual en solapa Manual
        self.manual_etiqueta_valor_actual_col = self.builder.get_object("manual_etiqueta_valor_actual_col")
        self.manual_etiqueta_valor_actual_col.set_text("-")
        ## Etiqueta de COL deseada en solapa Manual
        self.manual_etiqueta_valor_deseado_col = self.builder.get_object("manual_etiqueta_valor_desired_col")
        self.manual_etiqueta_valor_deseado_col.set_text("-")
        ## Etiqueta de COL actual en solapa Manual
        self.manual_etiqueta_valor_actual_row = self.builder.get_object("manual_etiqueta_valor_actual_row")
        self.manual_etiqueta_valor_actual_row.set_text("-")
        ## Etiqueta de ROW deseada en solapa Manual
        self.manual_etiqueta_valor_deseado_row = self.builder.get_object("manual_etiqueta_valor_desired_row")
        self.manual_etiqueta_valor_deseado_row.set_text("-")
        ## Etiqueta que indica el tipo de pivot actual seleccionado
        self.inicio_etiqueta_estado_pivot = self.builder.get_object("inicio_etiqueta_estado_pivot")
        ## Ventana principal del HMI
        self.window = self.builder.get_object("ventana_principal")
        
        ## Ventana secundaria de selección de archivos del sistema
        self.window2 = self.builder.get_object("ventana_secundaria_archivos")
        
        ## Ventana secundaria para visualización o modificación de dirección de red
        self.window3 = self.builder.get_object("ventana_secundaria_red")
        
        ## Vista de arbol para el modo Inspection
        self.inspection_tree_view = self.builder.get_object("inspection_tree_view")
        
        ## Widget del interrupor de control principal (nos aseguramos que inicie desactivado)
        self.inicio_toggle_fixture_control = self.builder.get_object("inicio_toggle_fixture_control")
        self.inicio_toggle_fixture_control.set_active(False)

        ## Widget del botón Stall en solapa Inicio. Incia habilitado, es decir, se controla que no
        # haya una alarma de stall en todo movimiento.
        self.inicio_toggle_fixture_stall = self.builder.get_object("inicio_toggle_fixture_stall")
        self.inicio_toggle_fixture_stall.set_active(True)
        
        ## Widget botón lift up total de la solapa Free Run
        self.b_fr_boton_lift_up_total = self.builder.get_object("fr_boton_lift_up_tot")
        ## Widget botón lift down total de la solapa Free Run
        self.b_fr_boton_lift_down_total = self.builder.get_object("fr_boton_lift_dwn_tot")
        # Widget botón lift up total de la solapa Manual
        self.b_manual_boton_lift_up_total = self.builder.get_object("manual_boton_lift_up_tot")
        # Widget botón lift down total de la solapa Manual
        self.b_manual_boton_lift_down_total = self.builder.get_object("manual_boton_lift_dwn_tot")
        ## Widget botón lift up total de la solapa Inspection
        self.b_inspection_boton_lift_up_total = self.builder.get_object("inspection_boton_lift_up_tot")
        ## Widget botón lift down total de la solapa Inspection
        self.b_inspection_boton_lift_down_total = self.builder.get_object("inspection_boton_lift_dwn_tot")
        
        ## Widget botón toggle POLE CW la solapa Free Run
        self.b_boton_toggle_pole_cw = self.builder.get_object("fr_toggle_pole_cw")
        ## Widget botón toggle POLE CCW la solapa Free Run
        self.b_boton_toggle_pole_ccw = self.builder.get_object("fr_toggle_pole_ccw")
        ## Widget botón toggle ARM CW la solapa Free Run
        self.b_boton_toggle_arm_cw = self.builder.get_object("fr_toggle_arm_cw")
        ## Widget botón toggle ARM CCW la solapa Free Run
        self.b_boton_toggle_arm_ccw = self.builder.get_object("fr_toggle_arm_ccw")

        ## Widget botón toggle que activa o desactiva sonidos de la interfaz
        self.inicio_id_toggle_zumbador = self.builder.get_object("inicio_id_toggle_zumbador")
        self.inicio_id_toggle_zumbador.set_active(True)

        ## Widget botón toggle que activa o desactiva simulador en solapa Inicio
        self.inicio_id_toggle_simulador = self.builder.get_object("inicio_id_toggle_simulador")
        self.inicio_id_toggle_simulador.set_active(False)
        
        ## Widget escala vel ARM
        self.ui_fr_escala_vel_arm = self.builder.get_object("fr_escala_vel_arm")
        # Se agregan las marcas de escala
        self.ui_fr_escala_vel_arm.add_mark(8, Gtk.PositionType.LEFT, "8   ")
        self.ui_fr_escala_vel_arm.add_mark(7, Gtk.PositionType.LEFT, "7   ")
        self.ui_fr_escala_vel_arm.add_mark(6, Gtk.PositionType.LEFT, "6   ")
        self.ui_fr_escala_vel_arm.add_mark(5, Gtk.PositionType.LEFT, "5   ")
        self.ui_fr_escala_vel_arm.add_mark(4, Gtk.PositionType.LEFT, "4   ")
        self.ui_fr_escala_vel_arm.add_mark(3, Gtk.PositionType.LEFT, "3   ")
        self.ui_fr_escala_vel_arm.add_mark(2, Gtk.PositionType.LEFT, "2   ")
        self.ui_fr_escala_vel_arm.add_mark(1, Gtk.PositionType.LEFT, "1   ")
        self.ui_fr_escala_vel_arm.add_mark(0, Gtk.PositionType.LEFT, "0   ")
        
        ## Widget escala vel
        self.ui_fr_escala_vel_pole = self.builder.get_object("fr_escala_vel_pole")
        # Se agregan las marcas de escala
        self.ui_fr_escala_vel_pole.add_mark(8, Gtk.PositionType.RIGHT, "   8")
        self.ui_fr_escala_vel_pole.add_mark(7, Gtk.PositionType.RIGHT, "   7")
        self.ui_fr_escala_vel_pole.add_mark(6, Gtk.PositionType.RIGHT, "   6")
        self.ui_fr_escala_vel_pole.add_mark(5, Gtk.PositionType.RIGHT, "   5")
        self.ui_fr_escala_vel_pole.add_mark(4, Gtk.PositionType.RIGHT, "   4")
        self.ui_fr_escala_vel_pole.add_mark(3, Gtk.PositionType.RIGHT, "   3")
        self.ui_fr_escala_vel_pole.add_mark(2, Gtk.PositionType.RIGHT, "   2")
        self.ui_fr_escala_vel_pole.add_mark(1, Gtk.PositionType.RIGHT, "   1")
        self.ui_fr_escala_vel_pole.add_mark(0, Gtk.PositionType.RIGHT, "   0")

        # Warning: deprecated!
        #self.window2.override_background_color(0, Gdk.RGBA(0.9,0.0,0.0,1.0))
        
        # Mensaje de bienvenida
        self.inicio_etiqueta_msg.set_text("Welcome to the NFC Human-Machine Interface!")
        self.window.show()

        # Se solicita actualizar las etiquetas del reloj y luego refrescarlas cada 
        # REFRESCO_ms_RELOJ milisegundos
        self.leer_reloj()
        GLib.timeout_add(self.ui_REFRESCO_ms_RELOJ, self.leer_reloj)
        # Beep 
        zumbador.beep_primordial()

        # Función telemetría se ejecuta cada REFRESCO_ms_TELEMETRIA milisegundos
        GLib.timeout_add(self.ui_REFRESCO_ms_TELEMETRIA, self.telemetria)

         # Función telemetría se ejecuta cada REFRESCO_ms_TELEMETRIA milisegundos
        GLib.timeout_add(self.ui_REFRESCO_ms_SIMULADOR, self.refrescar_simulador)
        

    def control_zumbador(self, button):
        # se verifica si el botón esta presionado o suelto
        b_toggle_zumbador = button.get_active()
        depurador(3, "HMI", "****************************************")

        if (b_toggle_zumbador == True):
            self.b_beeps = True
            self.inicio_etiqueta_msg.set_text("HMI beeps enabled")
            self.inicio_etiqueta_toggle_zumbador.set_text("Sounds Yes")
            depurador(1, "HMI", "- zumbador activado")
            zumbador.beep_button()
            

        else:
            self.b_beeps = False
            self.inicio_etiqueta_msg.set_text("HMI beeps disabled")
            self.inicio_etiqueta_toggle_zumbador.set_text("Sounds No ")
            depurador(3, "HMI", "- zumbador desactivado")

        depurador(1, "HMI", " ")

        return True
    
    
    ##
    # @brief Función que implementa los botones toggle "Main control" y "Stall"
    # y el botón pulsador Stow
    # @param self Puntero al objeto HMI
    # @param button "Main control", "Stall Function" o "Stow"
    # @return none
    def fixture_control(self, button):
        
        # Obtiene el nombre del widget switch presionado
        s_name = button.get_name()

        if (s_name == "fixture_control_button"):    
            if button.get_active():
                self.s_ctrlEn = "CONTROL_ENABLE"
                self.inicio_etiqueta_estado_main_control.set_text("Enabled")
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg("Fixture harness main control enabled...")

                if self.b_beeps == True:
                    zumbador.beep_button()

            else:
                self.s_ctrlEn = "DISABLE_CONTROL"
                self.inicio_etiqueta_estado_main_control.set_text("Disabled")
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg("Fixture harness main control disabled...")

                if self.b_beeps == True:
                    zumbador.beep_stop()

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Fixture status = " + self.s_ctrlEn)
            depurador(1, "HMI", " ")
            
        elif (s_name == "fixture_stall_button"):
            if button.get_active():
                self.s_stallEn = "STALL_ENABLE"
                self.inicio_etiqueta_estado_stall.set_text("Enabled")
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg("Fixture harness stall function enabled...")

                if self.b_beeps == True:
                    zumbador.beep_button()

            else:
                self.s_stallEn = "DISABLE_STALL"
                self.inicio_etiqueta_estado_stall.set_text("Disabled")
                self.actualizar_etiquetas_msg("Fixture harness stall function disabled...")

                if self.b_beeps == True:
                    zumbador.beep_button()

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Fixture status = " + self.s_stallEn)
            depurador(1, "HMI", " ")

        elif (s_name == "fixture_stow_button"):
            # Antes de mover verifica si está activado el control principal
            if (self.s_ctrlEn == "DISABLE_CONTROL"):
                if self.b_beeps == True:
                    zumbador.beep_stop()

                self.actualizar_etiquetas_msg("Fixture control is disabled...")

                return True
            
            # Antes de mover verifica si no está atascado el telemanipulador
            elif (self.b_stallAlm == True and self.s_stallEn == "STALL_ENABLE"):
                if self.b_beeps == True:
                    zumbador.beep_stop()

                self.actualizar_etiquetas_msg("Fixture harness is stalled...")

                return True   

            else:
                self.f_pole = np.deg2rad(270)
                self.f_arm = np.deg2rad(0)
                self.f_posCmdPole = 270
                self.f_posCmdArm = 0
                simu.refrescar_pos_comandada(self.f_pole, self.f_arm)

                self.actualizar_etiquetas_msg("Moving robot to STOW position...")

        elif (s_name == "fixture_pivot_button"):
            if self.b_beeps == True:
                    zumbador.beep_button()

            if (self.s_pivot_type == "main"):
                self.s_pivot_type = "alternative"
            elif (self.s_pivot_type == "alternative"):
                self.s_pivot_type = "main"

            self.actualizar_etiquetas_msg("Fixture " + self.s_pivot_type + " pivot type selected")
            self.inicio_etiqueta_estado_pivot.set_text(self.s_pivot_type)

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Fixture pivot = " + self.s_pivot_type)
            depurador(1, "HMI", " ")
        
        return True
    
    ##
    # @brief Función que implementa todos los botones de STOP del HMI
    # @param self Puntero al objeto HMI
    # @param button "STOP"
    # @return none
    def stop_total(self, button):
        # Desenergiza relé principal en NFC

        if(self.b_beeps == True):
            # Beep 
            zumbador.beep_stop()

        self.s_ctrlEn = "DISABLE_CONTROL"
        # Actualiza la variable modo
        self.s_mode = "STOP"
        # Cambia a OFF el widget del interruptor principal
        self.inicio_toggle_fixture_control.set_active(False)
        # Cambia el mensaje de la etiqueta del interruptor
        self.inicio_etiqueta_estado_main_control.set_text("Disabled")
        ## Se indica la acción en las etiquetas principales de msg.
        self.actualizar_etiquetas_msg("Stop, fixture harness main control disabled...") 

        # Si no está habilitado el control principal aseguramos que no queden 
        # presionados los botones toggle de las diferentes solapas
        self.b_fr_boton_lift_up_total.set_active(False)
        self.b_fr_boton_lift_down_total.set_active(False)
        self.b_manual_boton_lift_up_total.set_active(False)
        self.b_manual_boton_lift_down_total.set_active(False)
        self.b_inspection_boton_lift_up_total.set_active(False)
        self.b_inspection_boton_lift_down_total.set_active(False)  
        self.b_boton_toggle_arm_cw.set_active(False)
        self.b_boton_toggle_arm_ccw.set_active(False)
        self.b_boton_toggle_pole_cw.set_active(False)
        self.b_boton_toggle_pole_ccw.set_active(False)
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- STOP TOTAL")
        depurador(1, "HMI", "- Fixture status = " + self.s_ctrlEn)
        depurador(1, "HMI", " ")
            
        return True
        
    ##
    # @brief Detecta la fila seleccionada en el TreeView de solapa Inspection,
    # @param self Puntero al objeto hmi_SM13
    # @param selection Fila del Inspection GtkTreeView que se hace click.
    # @return none            
    def fila_plan_inspeccion(self, selection):
        
        depurador(1, "HMI", "****************************************")
        # get the model and the iterator that points at the data in the model
        (model, iter) = selection.get_selected()
        # set the label to a new value depending on the selection
        try:
            self.ui_plan_row = int(model[iter][0])
            self.ui_plan_col = int(model[iter][1])
            self.s_tubo_id = model[iter][2]
            depurador(1, "HMI", "- Se seleccionó "+self.s_tubo_id+"ROW:"+str(self.ui_plan_row)+(", COL:"+str(self.ui_plan_col)))
        except:
            pass     
        
        return True
    
    ##
    # @brief Función que se activa al presionar botón "Select Tube". 
    # Toma la COL y ROW deseadas (vía manual: control_manual o inspection:fila_plan_inspection 
    # y usa el scrip leer_archivo_hx.py para calcular P[x,y] de esa [col, row]. Calcula 
    # los ángulos de las articulaciones POLE y ARM usando el script de cinemática 
    # inversa. Finalmente actualiza los ángulos de las articulaciones POLE y ARM 
    # para enviar a RTU y refresca el simulador en caso de que este último se 
    # encuentre activo.
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
            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Fixture control is disabled...")

            return True
        
        # Antes de mover verifica si no está atascado el telemanipulador
        if (self.b_stallAlm == True and self.s_stallEn == "STALL_ENABLE"):
            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Fixture harness is stalled...")

            return True
        
        # TODO chequear ZS también

        if self.b_beeps == True:
                zumbador.beep_button()

        # Cálculo cinemático de los angulos necesarios para alcanzar COL, ROW deseados
        # en base a el archivo de intercambiador elegido y la posición de montaje del robot
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
        self.f_pole, self.f_arm, self.b_ik_success = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, self.s_pivot_type)
        
        if not self.b_ik_success:
            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Can not reach that position...")

            return True

        else:
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(round(np.rad2deg(self.f_pole), 3))+"°"+", ARM = "+str(round(np.rad2deg(self.f_arm), 3))+"°")
            depurador(1, "HMI", "- Para ubicar boquilla en Px = "+str(round(self.f_px_tubo, 3))+" in"+", Py = "+str(round(self.f_py_tubo, 3))+" in")
            depurador(1, "HMI", " ") 
            
            
            depurador(1, "HMI", "- Moviendo hacia "+ self.s_tubo_id + " (COL : "+ str(self.ui_plan_col) + ", ROW : "+str(self.ui_plan_row) + ")")
            depurador(1, "HMI", " ")
            self.actualizar_etiquetas_msg("Moving to tube ROW : "+ str(self.ui_plan_row) + ", COL : "+str(self.ui_plan_col))
            
            if(self.b_simulador == 1):
                depurador(1, "HMI", " - Nuevo ángulo POLE = " + str(self.f_pole))
                depurador(1, "HMI", " - Nuevo ángulo ARM  = " + str(self.f_arm))
                simu.refrescar_pos_comandada(self.f_pole, self.f_arm)
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Refrescando simulador SM-13")
                depurador(1, "HMI", " ")
            
            # Se convierten a [º] los ángulos POLE y ARM, se redondea a 3 decimales y
            # se actualizan las variables posCmd así las toma el módulo HMIcomRTU y las envía a RTU.    
            self.f_posCmdPole = (round(np.rad2deg(self.f_pole), 3))
            self.f_posCmdArm = (round(np.rad2deg(self.f_arm), 3))

            # Al presionar botón "Select Tube" se envían los ángulos al robot y se igualan 
            # los valores de COL y ROW desedos con actuales.
            self.manual_etiqueta_valor_actual_col.set_text(str(self.ui_plan_col)) 
            self.manual_etiqueta_valor_actual_row.set_text(str(self.ui_plan_row))
        
            return True
    
    ##
    # @brief Función que se activa al presionar botón "Next Tube". Identifica
    # la fila actual y selecciona la siguiente en el plan de inspección activo.
    # Usa el scrip leer_archivo_hx.py para calcular P[x,y] del siguiente [col, row]. 
    # Calcula los ángulos de las articulaciones POLE y ARM usando el script de cinemática 
    # inversa. Finalmente a# Al presionar botón "Select Tube" se envían los ángulos al robot y se igualan 
    # los valores de COL y ROW desedos con actuales.
    # Finalmente actualiza los ángulos de las articulaciones POLE y ARM 
    # para enviar a RTU y refresca el simulador en caso de que este último se 
    # encuentre activo.
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
            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Fixture control is disabled...")

            return True
        
        # Antes de mover verifica si no está atascado el telemanipulador
        if (self.b_stallAlm == True and self.s_stallEn == "STALL_ENABLE"):
            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Fixture harness is stalled...")

            return True
        
        # TODO chequear ZS también

        if self.b_beeps == True:
                zumbador.beep_button()
        
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

        # Cálculo cinemático de los angulos necesarios para alcanzar COL, ROW deseados
        # en base a el arch# Al presionar botón "Select Tube" se envían los ángulos al robot y se igualan 
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
        self.f_pole, self.f_arm, self.b_ik_success = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, self.s_pivot_type)
        
        if not self.b_ik_success:
            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Can not reach that position...")
            return True

        else:
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(round(np.rad2deg(self.f_pole), 3))+"°"+", ARM = "+str(round(np.rad2deg(self.f_arm), 3))+"°")
            depurador(1, "HMI", "- Para ubicar boquilla en Px = "+str(round(self.f_px_tubo, 3))+" in"+", Py = "+str(round(self.f_py_tubo, 3))+" in")
            depurador(1, "HMI", " ") 


            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Moviendo hacia "+ self.s_tubo_id + " (COL : "+ str(self.ui_plan_col) + ", ROW : "+str(self.ui_plan_row) + ")")
            depurador(1, "HMI", " ")
            self.actualizar_etiquetas_msg("Moving to tube ROW : "+ str(self.ui_plan_row) + ", COL : "+str(self.ui_plan_col))
            
            if(self.b_simulador == 1):
                depurador(1, "HMI", " - Nuevo ángulo POLE = " + str(self.f_pole))
                depurador(1, "HMI", " - Nuevo ángulo ARM  = " + str(self.f_arm))
                simu.refrescar_pos_comandada(self.f_pole, self.f_arm)
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Refrescando simulador SM-13")
                depurador(1, "HMI", " ")
                
            # Se convierten a [º] los ángulos POLE y ARM, se redondea a 3 decimales y
            # se actualizan las variables posCmd así las toma el módulo HMIcomRTU y las envía a RTU.    
            self.f_posCmdPole = (round(np.rad2deg(self.f_pole), 3))
            self.f_posCmdArm = (round(np.rad2deg(self.f_arm), 3))

            # Al presionar botón "Next Tube" se envían los ángulos al robot y se igualan 
            # los valores de COL y ROW desedos con actuales.
            self.manual_etiqueta_valor_actual_col.set_text(str(self.ui_plan_col)) 
            self.manual_etiqueta_valor_actual_row.set_text(str(self.ui_plan_row))
            
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
            if self.b_beeps == True:
                zumbador.beep_stop()

            # Aseguramos que el botón toggle del simulador no quede presionado
            self.inicio_id_toggle_simulador.set_active(False)
            return True
         
        if (self.b_simulador == 0):
            if self.b_beeps == True:
                zumbador.beep_primordial()

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", " - Inicia simulador SM-13")
            
            self.inicio_etiqueta_estado_simulador.set_text("Enabled")

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
            
            simu = simulador_SM13(self.s_archivo_fixture, self.ui_montaje, self.s_archivo_hx)
            
            depurador(1, "HMI", " - Simulador refrescando con qp = " + str(self.f_pole))
            depurador(1, "HMI", " - Simulador refrescando con qa = " + str(self.f_arm))
            simu.refrescar_pos_comandada(self.f_pole, self.f_arm)
            self.b_simulador = 1
            
            depurador(1, "HMI", " ")
            
            return True
            
        if (self.b_simulador == 1):
            if self.b_beeps == True:
                zumbador.beep_stop()

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", " - Se cierra simulador SM-13")
            depurador(1, "HMI", " ")
            del simu
            self.inicio_etiqueta_estado_simulador.set_text("Disabled")
            self.b_simulador = 0
            
            return True

    ##
    # @brief Función que actualiza los valores de ángulos actuales de POLE y ARM
    # @param self Puntero al objeto HMI
    def refrescar_simulador(self):
        global simu

        if (self.b_simulador):
            simu.refrescar_pos_actual(self.f_posActPole, self.f_posActArm)

        return True

    
    ##
    # @brief Función que se activa al presionar botón "Archivos" en solapa Inicio.
    # Crea una ventana secundaria para permitir seleccionar los archivos de cfg.
    # @param self Puntero al objeto HMI
    # @param button Botón "Archivos"
    # @return none
    def ventana_archivos(self, button):
        if self.b_beeps == True:
            zumbador.beep_button()

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
        if self.b_beeps == True:
            zumbador.beep_button()

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
        
        self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, trash, trash = leer_datos_SM13(self.s_hx_type, self.s_archivo_fixture, self.ui_montaje)
        
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
        self.s_hx_type = str(model_hx[combo_hx][0])
        self.s_archivo_hx = self.s_cfg_files_hx_path+str(model_hx[combo_hx][0])+"/tubesheet.csv"
        depurador(2, "HMI", "****************************************")
        depurador(2, "HMI", "- Archivo hx: "+self.s_archivo_hx)
        depurador(2, "HMI", " ")

        # Se trae el objeto combobox para elegir los planes de inspección en función del HX
        combo_plan = self.builder.get_object("cfg_combobox_inspection_plan")
        try:
            # Se limpia de la lista de planes de algún HX anterior
            combo_plan.remove_all()
        except:
            pass

        # Se agregan las opciones de planes según el HX seleccionado
        if str(self.s_hx_type) == "_cne_moderador_3211_HX1":
            combo_plan.append_text("_cne_moderador_3211_HX1/InspPlan1_M_1")
            combo_plan.append_text("_cne_moderador_3211_HX1/InspPlan2_M_1")
            combo_plan.append_text("_cne_moderador_3211_HX1/InspPlan3_M_2")
            combo_plan.append_text("_cne_moderador_3211_HX1/InspPlan4_M_2")
            combo_plan.append_text("_cne_moderador_3211_HX1/InspPlan5_M_3")
            combo_plan.append_text("_cne_moderador_3211_HX1/InspPlan6_M_3")
            combo_plan.append_text("_cne_moderador_3211_HX1/InspPlan7_M_4")
            combo_plan.append_text("_cne_moderador_3211_HX1/InspPlan8_M_4")
        if str(self.s_hx_type) == "_cne_moderador_3211_HX2":
            combo_plan.append_text("_cne_moderador_3211_HX2/InspPlan1_M_1")
            combo_plan.append_text("_cne_moderador_3211_HX2/InspPlan2_M_1")
            combo_plan.append_text("_cne_moderador_3211_HX2/InspPlan3_M_2")
            combo_plan.append_text("_cne_moderador_3211_HX2/InspPlan4_M_2")
            combo_plan.append_text("_cne_moderador_3211_HX2/InspPlan5_M_3")
            combo_plan.append_text("_cne_moderador_3211_HX2/InspPlan6_M_3")
            combo_plan.append_text("_cne_moderador_3211_HX2/InspPlan7_M_4")
            combo_plan.append_text("_cne_moderador_3211_HX2/InspPlan8_M_4")

        
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
        self.a_plan_row, self.a_plan_col, self.a_plan_tubos = leer_plan(self.s_archivo_plan)

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

        if self.b_beeps == True:
            zumbador.beep_primordial()
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Archivo hx        : "+self.s_archivo_hx)
        depurador(1, "HMI", "- Plan de Inspección: "+self.s_archivo_plan)
        depurador(1, "HMI", "- Tipo Fixture      : "+self.s_archivo_fixture)
        
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Faltan cargar archivos...")
            depurador(1, "HMI", " ")
            return True
        
        a_COLUMNS = ["ROW", "COL", "TUBE"]
        
        #inspection_tree_view = self.builder.get_object("inspection_tree_view")
                
        # the data in the model (three strings for each row, one for each
        # column)
        self.lista_tubos = Gtk.ListStore(str, str, str)
        self.lista_tubos.clear()
        
        # append the values in the model                
        for i in range(len(self.a_plan_tubos)):
            self.lista_tubos.append([str(self.a_plan_row[i]), str(self.a_plan_col[i]), str(self.a_plan_tubos[i])])
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
        self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, trash, trash = leer_datos_SM13(self.s_hx_type, self.s_archivo_fixture, self.ui_montaje)
        
        # Se resetea simulador ya que cambió el plan de inspección y, por lo tanto, la posición del FH.
        if (self.b_simulador == 1):
            del simu
            simu = simulador_SM13(self.s_archivo_fixture, self.ui_montaje, self.s_archivo_hx)
            simu.refrescar_pos_comandada(self.f_pole, self.f_arm)
            
        # Se establecen las especificaciones del mazo de tubos en base al intercambiador elegido
        # Si se cambia el path, procurar que tenga la misma cantidad de carpetas
        trash, s_folder1, s_folder2, s_folder3, s_folder4, s_folder5, s_folder6, s_folder7, s_folder8, trash = self.s_archivo_hx.split("/")
        # Se identifica el path al archivo y se lo lee
        s_tube_specs_path = "/"+s_folder1+"/"+s_folder2+"/"+s_folder3+"/"+s_folder4+"/"+s_folder5+"/"+s_folder6+"/"+s_folder7+"/"+s_folder8+"/tube_specs.csv"
        a_specs_tube = leer_specs_tubos(s_tube_specs_path)
        self.f_tube_od = float(a_specs_tube[0])
        self.f_y_pitch = float(a_specs_tube[1])
        self.f_x_pitch = float(a_specs_tube[2])
        
        # permite que las columnas del TreeView se creen una sola vez
        self.b_carga = 1

        # Luego de la selección de archivos se abre robots_home_offsets.csv en modo lectura, se lo lee y se cierra. 
        offsets_file = open(self.s_project_path + "/hmi/cfg_files/Robots/robots_home_offsets.csv", "r")
        offsets_lines = offsets_file.readlines()
        offsets_file.close()
        # Se extraen la variables que contendrán la dir IP y el PUERTO
        trash, self.ui_pole_rdc_offset, self.ui_arm_rdc_offset = offsets_lines[1].split(";")
        depurador(1, "HMI", "- Se cargó POLE offset: "+ str(self.ui_pole_rdc_offset) + ", ARM offset: " + str(self.ui_arm_rdc_offset))
        depurador(1, "HMI", "- ")
        
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
                if self.b_beeps == True:
                    zumbador.beep_stop()

                self.f_incremento_jog = 0.0
                self.inspection_etiqueta_msg.set_text("Jog control disabled...")
                self.manual_etiqueta_msg.set_text("Jog control disabled...")
                # Restablecemos la posición de la boquilla sin ningún Jog.
                self.f_px_tubo -= self.f_incremento_acumulado_jog_col*self.f_x_pitch*2
                self.f_py_tubo += self.f_incremento_acumulado_jog_row*self.f_y_pitch
                
                # Calcula los ángulos de los ejes POLE y ARM en base al corrimiento
                self.f_pole, self.f_arm, self.b_ik_success = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, self.s_pivot_type)   
                if not self.b_ik_success:
                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    self.actualizar_etiquetas_msg("Coordinate unreacheable...")

                    return True

                if (self.b_simulador == 1):
                    # Se refresca simulador ya que se está haciendo un ajuste fino de la boquilla.
                    # no se suma el jog porque ya se lo hace arriba en esta misma función.
                    simu.refrescar_pos_comandada(self.f_pole, self.f_arm) 

                # Se convierten a [º] los ángulos POLE y ARM, se redondea a 3 decimales y
                # se actualizan las variables posCmd así las toma el módulo HMIcomRTU y las envía a RTU.    
                self.f_posCmdPole = (round(np.rad2deg(self.f_pole), 3))
                self.f_posCmdArm = (round(np.rad2deg(self.f_arm), 3))
                
                # Se resetea el Jog acumulado y se actualizan las etiquetas
                self.f_incremento_acumulado_jog_col = 0.0
                self.f_incremento_acumulado_jog_row = 0.0

                # Se actualizan etiquetas en solapas Manual e Inspection
                self.manual_etiqueta_valor_jog_row.set_text(str(round(self.f_incremento_acumulado_jog_row, 2)))
                self.manual_etiqueta_valor_jog_col.set_text(str(round(self.f_incremento_acumulado_jog_col, 2)))
                self.inspection_etiqueta_valor_jog_row.set_text(str(round(self.f_incremento_acumulado_jog_row, 2)))
                self.inspection_etiqueta_valor_jog_col.set_text(str(round(self.f_incremento_acumulado_jog_col, 2)))
            else:
                if self.b_beeps == True:
                    zumbador.beep_button()

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

            if self.b_beeps == True:
                zumbador.beep_stop()

            return True
        
        # Antes de mover verifica si está activado el control principal
        if (self.s_ctrlEn == "DISABLE_CONTROL"):
            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Fixture control is disabled...")

            return True
        
        # Antes de mover verifica si no está atascado el telemanipulador
        if (self.b_stallAlm == True and self.s_stallEn == "STALL_ENABLE"):
            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Fixture harness is stalled...")

            return True
        
        # Si no hay jog acumulado no hay nada que hacer aquí.
        if (self.f_incremento_jog == 0.0):
            self.inspection_etiqueta_msg.set_text("Select an increment to jog the end effector...")
            self.manual_etiqueta_msg.set_text("Select an increment to jog the end effector...")
            return True

        if self.b_beeps == True:
            zumbador.beep_button()
            
            
        # Obtiene el nombre del boton jog presionado
        s_boton_jog = button.get_name()
        
        if (s_boton_jog == "jog_to_east"):
            self.f_px_tubo += self.f_incremento_jog*self.f_x_pitch*2
            self.f_incremento_acumulado_jog_col += self.f_incremento_jog
            
        elif (s_boton_jog == "jog_to_west"):
            self.f_px_tubo -= self.f_incremento_jog*self.f_x_pitch*2
            self.f_incremento_acumulado_jog_col -= self.f_incremento_jog
            
        if (s_boton_jog == "jog_to_north"):
            self.f_py_tubo -= self.f_incremento_jog*self.f_y_pitch
            self.f_incremento_acumulado_jog_row += self.f_incremento_jog
        
        elif (s_boton_jog == "jog_to_south"):
            self.f_py_tubo += self.f_incremento_jog*self.f_y_pitch
            self.f_incremento_acumulado_jog_row -= self.f_incremento_jog
            
        try:
            # Se actualizan etiquetas en solapas Manual e Inspection
            self.manual_etiqueta_valor_jog_row.set_text(str(round(self.f_incremento_acumulado_jog_row, 2)))
            self.manual_etiqueta_valor_jog_col.set_text(str(round(self.f_incremento_acumulado_jog_col, 2)))
            self.inspection_etiqueta_valor_jog_row.set_text(str(round(self.f_incremento_acumulado_jog_row, 2)))
            self.inspection_etiqueta_valor_jog_col.set_text(str(round(self.f_incremento_acumulado_jog_col, 2)))
        except:
            pass
        
        self.inspection_etiqueta_msg.set_text("Jogging the end effector position...")
        self.manual_etiqueta_msg.set_text("Jogging the end effector position...")
            
        # Calcula los ángulos de los ejes POLE y ARM en base al corrimiento
        self.f_pole, self.f_arm, self.b_ik_success = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, self.s_pivot_type)   
        if not self.b_ik_success:
            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Can not move positioner in that direction...")
            
            return True

        if (self.b_simulador == 1):
            # Se refresca simulador ya que se está haciendo un ajuste fino de la boquilla.
            # no se suma el jog porque ya se lo hace arriba en esta misma función.
            simu.refrescar_pos_comandada(self.f_pole, self.f_arm)
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Ajustando fino con incremento jog: "+str(self.f_incremento_jog))
        depurador(1, "HMI", "- Jog acumulado en COL: "+str(self.f_incremento_acumulado_jog_col))
        depurador(1, "HMI", "- Jog acumulado en ROW: "+str(self.f_incremento_acumulado_jog_row))
        depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(round(np.rad2deg(self.f_pole), 3))+"°"+", ARM = "+str(round(np.rad2deg(self.f_arm), 3))+"°")
        depurador(1, "HMI", "- Para ubicar boquilla en Px = "+str(round(self.f_px_tubo, 3))+" in"+", Py = "+str(round(self.f_py_tubo, 3))+" in")
        depurador(1, "HMI", " ") 
        
        # se actualizan las variables posCmd con el JOG aplicado.    
        self.f_posCmdPole = (round(np.rad2deg(self.f_pole), 3))
        self.f_posCmdArm = (round(np.rad2deg(self.f_arm), 3))
        
        return True
    
    ##
    # @brief Función que almacena el offset que hay entre cuentas de 
    # resolver POLE y ARM cuando el robot se encuentra plegado.
    # @param self Puntero al objeto HMI
    # @param button Boton Set Offset
    # @return none
    def cal_home_offset(self, button):
        if self.b_beeps == True:
            zumbador.beep_primordial()

        offsets_lines = [0]
        new_offsets_lines = [0]
        
        depurador(1, "HMI", "****************************************")
        
        # Se abre el archivo robots_home_offsets.csv en modo lectura, se lo lee y se cierra. 
        offsets_file = open(self.s_project_path + "/hmi/cfg_files/Robots/robots_home_offsets.csv", "r")
        offsets_lines = offsets_file.readlines()
        offsets_file.close()
        
        trash, s_x_pole_offset, s_x_arm_offset = offsets_lines[1].split(";")
        
        # Se comprueban cambios.
        if (self.ui_rdcActPole == int(s_x_pole_offset) and self.ui_rdcActArm == int(s_x_arm_offset)):
            # Si no hubo cambios en los nuevos offsets no se sobreescribe el archivo.
            depurador(1, "HMI", "- No se modificaron offsets")
            depurador(1, "HMI", "- POLE offset actual: "+ s_x_pole_offset + ", ARM offset actual: " + s_x_arm_offset)
            depurador(1, "HMI", " ")
            return True
            
        else:
            # Si hay diferencias, se elimina la fila con los offsets anteriores.
            del offsets_lines[1]

            a_robot = self.s_archivo_fixture.rsplit("/",1)
            s_robot, trash = a_robot[1].split(".")
            
            # Como hay cambios, se abre el archivo robots_home_offsets.csv en modo escritura,
            # se actualizan los cambios y se cierra. 
            new_offsets_file = open(self.s_project_path + "/hmi/cfg_files/Robots/robots_home_offsets.csv", "w+")
            new_offsets_line = str(s_robot) + ";" + str(self.ui_rdcActPole) + ";" + str(self.ui_rdcActArm)
            new_offsets_file.write("ROBOT;POLE HOME OFFSET;ARM HOME OFFSET\n")
            new_offsets_file.write(new_offsets_line)
            new_offsets_file.close()
            

            # Se abre el archivo robots_home_offsets.csv en modo lectura, se lo lee
            # para asegurarse que se implementó el cambio de offsets y se lo cierra. 
            new_offsets_file = open(self.s_project_path + "/hmi/cfg_files/Robots/robots_home_offsets.csv", "r")
            new_offsets_lines = new_offsets_file.readlines()
            new_offsets_file.close()
            
            # Se actualizan la variables principales que contienen los valores de offset 
            trash, self.ui_pole_rdc_offset, self.ui_arm_rdc_offset = new_offsets_lines[1].split(";")
        
            depurador(1, "HMI", "- POLE offset anterior: "+ s_x_pole_offset + ",     ARM offset anterior: " + s_x_arm_offset)
            depurador(1, "HMI", "- POLE offset nuevo   : "+ self.ui_pole_rdc_offset + ", ARM offset nuevo   : " + self.ui_arm_rdc_offset)
        
        return True
    
    ##
    # @brief Función que indica si se han cargado archivos de configuración o no
    # @param self puntero al objeto HMI
    # @return True o False si se han cargado o no los tres archivos de configuración
    def archivos(self):

        if (self.s_archivo_hx == 'none' or self.s_archivo_fixture == 'none' or self.s_archivo_plan == 'none'):
            self.actualizar_etiquetas_msg("No configuration files has been loaded...")

            if self.b_beeps == True:
                zumbador.beep_stop()

            return False
        else:
            return True
    
    ##
    # @brief Función que optimiza la actualización de mensajes generales
    # simultáneamente en todas las etiquetas de msg de las diferentes solapas
    # @param self puntero al objeto HMI
    # @param s_msg Mensaje general a imprimir en todas las etiquetas de las solapas
    # @return none
    def actualizar_etiquetas_msg(self, s_msg): 
        self.inicio_etiqueta_msg.set_text(s_msg)
        self.fr_etiqueta_msg.set_text(s_msg)
        self.manual_etiqueta_msg.set_text(s_msg)
        self.inspection_etiqueta_msg.set_text(s_msg)
          
        return True

    ##
    # @brief Función que optimiza la actualización de hora y fecha en las 
    # etiquetas de las diferentes solapas
    # @param self puntero al objeto HMI
    # @param s_string_de_hora Hora en formato string
    # @param s_string_de_fecha Fecha en formato string
    # @return none
    def actualizar_etiquetas_reloj(self, s_string_de_hora, s_string_de_fecha): 
        self.inicio_etiqueta_hora.set_text(s_string_de_hora)
        self.inspection_etiqueta_hora.set_text(s_string_de_hora)
        self.fr_etiqueta_hora.set_text(s_string_de_hora)
        self.manual_etiqueta_hora.set_text(s_string_de_hora)

        self.inicio_etiqueta_fecha.set_text(s_string_de_fecha)
        self.inspection_etiqueta_fecha.set_text(s_string_de_fecha)
        self.fr_etiqueta_fecha.set_text(s_string_de_fecha)
        self.manual_etiqueta_fecha.set_text(s_string_de_fecha)
        
        return True

    
    ##
    # @brief Función que actualiza todas las etiquetas enc y ang de las solapas
    # free run, manual e inspection.
    # @param self puntero al objeto HMI
    # @param f_ang_pole ángulo de la articulación POLE en grados
    # @param f_ang_arm ángulo de la articulación ARM en grados
    # @return none
    def actualizar_etiquetas_enc_ang(self, f_ang_pole, f_ang_arm):
        
        self.ui_encActArm = int((f_ang_arm/self.f_MAX_GRADOS)*self.ui_MAX_CUENTAS)
        self.ui_encActPole = int((f_ang_pole/self.f_MAX_GRADOS)*self.ui_MAX_CUENTAS)
        
        self.fr_etiqueta_valor_ang_arm.set_text(str(round(f_ang_arm, 3)))
        self.manual_etiqueta_valor_ang_arm.set_text(str(round(f_ang_arm, 3)))
        self.inspection_etiqueta_valor_ang_arm.set_text(str(round(f_ang_arm, 3)))
                    
        self.fr_etiqueta_valor_enc_arm.set_text(str(round(self.ui_encActArm, 0)))
        self.manual_etiqueta_valor_enc_arm.set_text(str(round(self.ui_encActArm, 0)))
        self.inspection_etiqueta_valor_enc_arm.set_text(str(round(self.ui_encActArm, 0)))
                    
        self.fr_etiqueta_valor_ang_pole.set_text(str(round(f_ang_pole, 3)))
        self.manual_etiqueta_valor_ang_pole.set_text(str(round(f_ang_pole, 3)))
        self.inspection_etiqueta_valor_ang_pole.set_text(str(round(f_ang_pole, 3)))
                    
        self.fr_etiqueta_valor_enc_pole.set_text(str(round(self.ui_encActPole, 0)))
        self.manual_etiqueta_valor_enc_pole.set_text(str(round(self.ui_encActPole, 0)))
        self.inspection_etiqueta_valor_enc_pole.set_text(str(round(self.ui_encActPole, 0)))
        
        return True
    
    ##
    # @brief Función que levanta la ventana de configuración de red
    # para mostrar la dirección IP y puerto actuales o para modificarlos.
    # @param self puntero al objeto HMI
    # @param button Botón de red en ventana inicio
    # @return none
    def ventana_red(self, button):
        if self.b_beeps == True:
            zumbador.beep_button()

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
        if self.b_beeps == True:
            zumbador.beep_button()

        self.window3.hide()

        return True
    
    ##
    # @brief Función que implementa el botón "Modificar" en ventana de configuración
    # de red. Luego actualiza el archivo de texto network.csv y las variables IP y puerto.
    # @param self Puntero al objeto HMI
    # @param button Botón "modificar"
    # @return none
    def modificar_red(self, button):
        if self.b_beeps == True:
            zumbador.beep_primordial()

        address_lines = [0]
        new_address_lines = [0]
        
        depurador(1, "HMI", "****************************************")
        ## Entradas para modificación de dirección IP y Puerto de red
        
        ui_ip_0 = int(self.entrada_ip_0.get_value())
        ui_ip_1 = int(self.entrada_ip_1.get_value())
        ui_ip_2 = int(self.entrada_ip_2.get_value())
        ui_ip_3 = int(self.entrada_ip_3.get_value())
        ui_puerto = int(self.entrada_puerto.get_value())
        
        # Variables temporales que sirven para comparar cambios.
        s_temp_ip = str(ui_ip_3)+"."+str(ui_ip_2)+"."+str(ui_ip_1)+"."+str(ui_ip_0)
        s_temp_puerto = str(ui_puerto)
        
        # Se abre el archivo network.csv en modo lectura, se lo lee y se cierra. 
        network_file = open(self.s_project_path + "/hmi/cfg_files/network.csv", "r")
        address_lines = network_file.readlines()
        network_file.close()
        
        s_xip, s_xpuerto = address_lines[1].split(";")
        
        # Se comprueban cambios.
        if (s_temp_ip == s_xip and s_temp_puerto == s_xpuerto):
            # Si no hubo cambios en la dirección de red no se sobreescribe el archivo.
            depurador(1, "HMI", "- No se modificaron parámetros de red")
            depurador(1, "HMI", "- IP actual: "+ s_xip + ", Puerto actual: " + s_xpuerto)
            depurador(1, "HMI", " ")
            return True
            
        else:
            # Si hay diferencias, se elimina la fila con la IP y PORT anteriores.
            del address_lines[1]
            
            # Como hay cambios, se abre el archivo network.csv en modo escritura,
            # se actualizan los cambios y se cierra. 
            new_network_file = open(self.s_project_path + "/hmi/cfg_files/network.csv", "w+")
            new_address_line = s_temp_ip + ";" + s_temp_puerto
            new_network_file.write("IP address;Port\n")
            new_network_file.write(new_address_line)
            new_network_file.close()
            
            # Se abre el archivo network.csv en modo lectura, se lo lee
            # para asegurarse que se implementó el cambio de IP, PORT y se lo cierra. 
            new_network_file = open(self.s_project_path + "/hmi/cfg_files/network.csv", "r")
            new_address_lines = new_network_file.readlines()
            new_network_file.close()
            
            # Se actualizan la variables principales que contienen la IP y el PORT
            self.s_ip, self.s_port = new_address_lines[1].split(";")
        
            depurador(1, "HMI", "- IP anterior: "+ s_xip + ", Puerto anterior: " + s_xpuerto)
            depurador(1, "HMI", "- IP nueva   : "+ self.s_ip + ", Puerto nuevo: " + self.s_port)
            
        return True
    
    ##
    # @brief Función que implementa el comando del eje LIFT
    # @param self Puntero al objeto HMI
    # @param button Botones Lift: "UP/DOWN/UP-TOT/DOWN-TOT"
    # @return none
    def control_lift(self, button):
        # Antes de mover verifica si está activado el control principal
        if (self.s_ctrlEn == "DISABLE_CONTROL"):

            if self.b_beeps == True:
                zumbador.beep_stop()

            self.actualizar_etiquetas_msg("Fixture control is disabled...")

            # Si no está habilitado el control principal aseguramos que no queden 
            # presionados los botones toggle de las diferentes solapas
            self.b_fr_boton_lift_up_total.set_active(False)
            self.b_fr_boton_lift_down_total.set_active(False)
            self.b_manual_boton_lift_up_total.set_active(False)
            self.b_manual_boton_lift_down_total.set_active(False)
            self.b_inspection_boton_lift_up_total.set_active(False)
            self.b_inspection_boton_lift_down_total.set_active(False)

            return True        
        
        if self.b_beeps == True:
            zumbador.beep_button()

        # Obtiene el nombre del boton jog presionado
        s_boton_lift = button.get_name()
        
        # botones tipo pulsador afectados por temporizador
        if (s_boton_lift == "lift_up"):
            if(self.b_limitUp == True):
                self.s_mode = "STOP"
                self.inspection_etiqueta_msg.set_text("Lift upper limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite superior alcanzado en LIFT")
                depurador(1, "HMI", " ")
                return
            else:
                # si ZS no acusa alarma desde RTU se mueve lift
                self.s_mode = "LIFT"
                self.s_liftDir = "LIFT_UP"
                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos)
                
        elif (s_boton_lift == "lift_down"):
            if(self.b_limitDwn == True):
                self.s_mode = "STOP"
                self.inspection_etiqueta_msg.set_text("Lift lower limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite inferior alcanzado en LIFT")
                depurador(1, "HMI", " ")
                return
            else:
                # si ZS no acusa alarma desde RTU se mueve lift
                self.s_mode = "LIFT"
                self.s_liftDir = "LIFT_DOWN"
                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos)
            
        # botones tipo toggle no afectados por temporizador            
        if (s_boton_lift == "lift_up_total"):
            # se verifica si el botón esta presionado o suelto
            b_boton_lift_up_tot = button.get_active()
            if (b_boton_lift_up_tot == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                self.b_fr_boton_lift_down_total.set_active(False)
                self.b_manual_boton_lift_down_total.set_active(False)
                self.b_inspection_boton_lift_down_total.set_active(False)
                if(self.b_limitUp == True):
                    self.s_mode = "STOP"
                    self.inspection_etiqueta_msg.set_text("Lift upper limit alarm!")
                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite superior alcanzado en LIFT")
                    depurador(1, "HMI", " ")
                    return
                else:
                    # si ZS no acusa alarma desde RTU se mueve lift
                    self.s_mode = "LIFT"
                    self.s_liftDir = "LIFT_UP"
            if (b_boton_lift_up_tot == False):
                self.detener_movimientos()
                return
                
        elif (s_boton_lift == "lift_down_total"):
            # se verifica si el botón esta presionado o suelto
            b_boton_lift_down_tot = button.get_active()
            if (b_boton_lift_down_tot == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                self.b_fr_boton_lift_up_total.set_active(False)
                self.b_manual_boton_lift_up_total.set_active(False)
                self.b_inspection_boton_lift_up_total.set_active(False)
                if(self.b_limitDwn == True):
                    self.s_mode = "STOP"
                    self.actualizar_etiquetas_msg("Lift lower limit alarm!")
                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite inferior alcanzado en LIFT")
                    depurador(1, "HMI", " ")
                    return
                else:
                    # si ZS no acusa alarma desde RTU se mueve lift
                    self.s_mode = "LIFT"
                    self.s_liftDir = "LIFT_DOWN"
            if (b_boton_lift_down_tot == False):
                self.detener_movimientos()
                return
            
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Moving "+ self.s_liftDir)
        depurador(1, "HMI", " ")
        self.actualizar_etiquetas_msg("Moving "+ self.s_liftDir + "...")
    
        return 
    
    ##
    # @brief Función que coloca la variable de direccion del eje LIFT en STOP.
    # también sirve para detener el temporizado de los pulsadores asociados
    # al eje LIFT mediante el retorno de un False.
    # @param self Puntero al objeto HMI
    # @return False y detiene el GLib.timeout_add() que creo la tarea.
    def detener_movimientos(self):
        
        self.s_mode = "STOP"
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Modo: " + self.s_mode)
        depurador(1, "HMI", " ")
        
        self.actualizar_etiquetas_msg("Stopping movement...")
    
        # retorna False para detener el temporizador
        return False
    
    ##
    # @brief Función que administra los botones de control ARM y POLE
    # en solapa Free Run.
    # @param self Puntero al objeto HMI
    # @param button Botones de la solapa free run.
    # push_pole_cw; push_pole_ccw; push_arm_cw; push_arm_ccw;
    # toggle_pole_cw; toggle_pole_ccw; toggle_arm_cw; toggle_arm_ccw; 
    def control_free_run(self, button):
        # Antes de mover verifica si está activado el control principal
        if (self.s_ctrlEn == "DISABLE_CONTROL"):
            self.actualizar_etiquetas_msg("Fixture control is disabled...")

            # Si no está habilitado el control principal aseguramos que no queden 
            # presionados los botones toggle de las diferentes solapas
            self.b_boton_toggle_arm_cw.set_active(False)
            self.b_boton_toggle_arm_ccw.set_active(False)
            self.b_boton_toggle_pole_cw.set_active(False)
            self.b_boton_toggle_pole_ccw.set_active(False)

            if self.b_beeps == True:
                zumbador.beep_stop()

            return True
        
        # Se verifica la velocidad seteada para POLE y ARM
        self.ui_velCmdArm = int(self.ui_fr_escala_vel_arm.get_value())
        self.ui_velCmdPole = int(self.ui_fr_escala_vel_pole.get_value())        
        
        # Obtiene el nombre del boton jog presionado
        s_boton_fr = button.get_name()

        # botones tipo pulsador afectados por temporizador
        if (s_boton_fr == "fr_push_arm_cw"):
            # Si la velocidad seteada es 0 no hay nada que hacer aquí
            if(self.ui_velCmdArm == 0):
                self.s_mode = "STOP"            
                self.fr_etiqueta_msg.set_text("The set speed is 0 for this movement...")

                if self.b_beeps == True:
                    zumbador.beep_stop()

                return
                
            # Si hay un límite de movimiento por software se cancela el movimiento
            elif(self.b_cwLimitArm == True):
                self.s_mode = "STOP"
                self.fr_etiqueta_msg.set_text("ARM CW limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite horario alcanzado en ARM")
                depurador(1, "HMI", " ")

                if self.b_beeps == True:
                    zumbador.beep_alarm()

                return
    
            else:
                # si no hay alarmas de software desde RTU se mueve ARM CW
                self.s_mode = "FREE_RUN"
                self.s_freeRunAxis = "ARM"
                self.s_freeRunDir = "DIR_CW"

                if self.b_beeps == True:
                    zumbador.beep_button()

                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos)
                
        elif (s_boton_fr == "fr_push_arm_ccw"):
            
            # Si la velocidad seteada es 0 no hay nada que hacer aquí
            if(self.ui_velCmdArm == 0):
                self.s_mode = "STOP"
                self.fr_etiqueta_msg.set_text("The set speed is 0 for this movement...")

                if self.b_beeps == True:
                    zumbador.beep_stop()

                return
                
            elif (self.b_ccwLimitArm == True):
                self.s_mode = "STOP"
                self.fr_etiqueta_msg.set_text("ARM CCW limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite anti-horario alcanzado en ARM")
                depurador(1, "HMI", " ")

                if self.b_beeps == True:
                    zumbador.beep_alarm()

                return
            
            else:
                # si no hay alarmas de software desde RTU se mueve lift
                self.s_mode = "FREE_RUN"
                self.s_freeRunAxis = "ARM"
                self.s_freeRunDir = "CCW_DIR"

                if self.b_beeps == True:
                    zumbador.beep_button()

                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos)
                
        if (s_boton_fr == "fr_push_pole_cw"):
            # Si la velocidad seteada es 0 no hay nada que hacer aquí
            if(self.ui_velCmdPole == 0):
                self.s_mode = "STOP"

                if self.b_beeps == True:
                    zumbador.beep_stop()

                self.fr_etiqueta_msg.set_text("The set speed is 0 for this movement...")
                return
            
            if(self.b_cwLimitPole == True):
                self.s_mode = "STOP"
                self_etiqueta_msg.set_text("POLE CW limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite horario alcanzado en POLE")
                depurador(1, "HMI", " ")

                if self.b_beeps == True:
                    zumbador.beep_alarm()

                return
    
            else:
                # si no hay alarmas de software desde RTU se mueve ARM CW
                self.s_mode = "FREE_RUN"
                self.s_freeRunAxis = "POLE"
                self.s_freeRunDir = "DIR_CW"

                if self.b_beeps == True:
                    zumbador.beep_button()

                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos)
                
        elif (s_boton_fr == "fr_push_pole_ccw"):
            # Si la velocidad seteada es 0 no hay nada que hacer aquí
            if(self.ui_velCmdPole == 0):
                self.s_mode = "STOP"
                self.fr_etiqueta_msg.set_text("The set speed is 0 for this movement...")

                if self.b_beeps == True:
                    zumbador.beep_stop()

                return
            
            elif (self.b_ccwLimitPole == True):
                self.s_mode = "STOP"
                self.fr_etiqueta_msg.set_text("POLE CCW limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite anti-horario alcanzado en POLE")
                depurador(1, "HMI", " ")

                if self.b_beeps == True:
                    zumbador.beep_alarm()

                return
            else:
                # si no hay alarmas de software desde RTU se mueve POLE
                self.s_mode = "FREE_RUN"
                self.s_freeRunAxis = "POLE"
                self.s_freeRunDir = "CCW_DIR"

                if self.b_beeps == True:
                    zumbador.beep_button()

                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos)
            
        # botones tipo toggle NO afectados por temporizador            
        if (s_boton_fr == "fr_toggle_arm_cw"):
            # se verifica si el botón esta presionado o suelto
            b_toggle_arm_cw = button.get_active()
            if (b_toggle_arm_cw == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                self.b_boton_toggle_arm_ccw.set_active(False)
                # Si la velocidad seteada es 0 no hay nada que hacer aquí
                if(self.ui_velCmdArm == 0):
                    self.s_mode = "STOP"

                    # Si no hay velocidad seleccionada aseguramos que no quede presionado el toggle
                    self.b_boton_toggle_arm_cw.set_active(False)

                    self.fr_etiqueta_msg.set_text("The set speed is 0 for this movement...")

                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    return
                
                if(self.b_cwLimitArm == True):
                    self.s_mode = "STOP"
                    self.fr_etiqueta_msg.set_text("ARM CW limit alarm!")
                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite horario alcanzado en ARM")
                    depurador(1, "HMI", " ")

                    if self.b_beeps == True:
                        zumbador.beep_alarm()

                    return
                else:
                    # si no hay alarmas de límite por software desde RTU se mueve ARM CW
                    self.s_mode = "FREE_RUN"
                    self.s_freeRunAxis = "ARM"
                    self.s_freeRunDir = "DIR_CW"

                    if self.b_beeps == True:
                        zumbador.beep_button()

            if (b_toggle_arm_cw == False):
                self.detener_movimientos()
                return
                
        elif (s_boton_fr == "fr_toggle_arm_ccw"):
            # se verifica si el botón esta presionado o suelto
            b_toggle_arm_ccw = button.get_active()
            if (b_toggle_arm_ccw == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                self.b_boton_toggle_arm_cw.set_active(False)
                # Si la velocidad seteada es 0 no hay nada que hacer aquí
                if(self.ui_velCmdArm == 0):
                    self.s_mode = "STOP"

                    # Si no hay velocidad seleccionada aseguramos que no quede presionado el toggle
                    self.b_boton_toggle_arm_ccw.set_active(False)

                    self.fr_etiqueta_msg.set_text("The set speed is 0 for this movement...")

                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    return
                
                elif(self.b_ccwLimitArm == True):
                    self.s_mode = "STOP"
                    self.fr_etiqueta_msg.set_text("ARM CCW limit alarm!")
                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite anti-horario alcanzado en ARM")
                    depurador(1, "HMI", " ")

                    if self.b_beeps == True:
                        zumbador.beep_alarm()

                    return
                else:
                    # si no hay alarmas de límite por software desde RTU se mueve ARM CCW
                    self.s_mode = "FREE_RUN"
                    self.s_freeRunAxis = "ARM"
                    self.s_freeRunDir = "CCW_DIR"

                    if self.b_beeps == True:
                        zumbador.beep_button()

            if (b_toggle_arm_ccw == False):
                self.detener_movimientos()
                return
            
        if (s_boton_fr == "fr_toggle_pole_cw"):
            # se verifica si el botón esta presionado o suelto
            b_toggle_pole_cw = button.get_active()
            if (b_toggle_pole_cw == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                self.b_boton_toggle_pole_ccw.set_active(False)
                # Si la velocidad seteada es 0 no hay nada que hacer aquí
                if(self.ui_velCmdPole == 0):
                    self.s_mode = "STOP"

                    # Si no hay velocidad seleccionada aseguramos que no quede presionado el toggle
                    self.b_boton_toggle_pole_cw.set_active(False)

                    self.fr_etiqueta_msg.set_text("The set speed is 0 for this movement...")

                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    return
                
                elif(self.b_cwLimitPole == True):
                    self.s_mode = "STOP"
                    self.fr_etiqueta_msg.set_text("POLE CW limit alarm!")
                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite horario alcanzado en POLE")
                    depurador(1, "HMI", " ")

                    if self.b_beeps == True:
                        zumbador.beep_alarm()

                    return
                else:
                    # si no hay alarmas de límite por software desde RTU se mueve POLE CW
                    self.s_mode = "FREE_RUN"
                    self.s_freeRunAxis = "POLE"
                    self.s_freeRunDir = "DIR_CW"

                    if self.b_beeps == True:
                        zumbador.beep_button()

            if (b_toggle_pole_cw == False):
                self.detener_movimientos()
                return
                
        elif (s_boton_fr == "fr_toggle_pole_ccw"):
            # se verifica si el botón esta presionado o suelto
            b_toggle_pole_ccw = button.get_active()
            if (b_toggle_pole_ccw == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                self.b_boton_toggle_pole_cw.set_active(False)
                # Si la velocidad seteada es 0 no hay nada que hacer aquí
                if(self.ui_velCmdPole == 0):
                    self.s_mode = "STOP"

                    # Si no hay velocidad seleccionada aseguramos que no quede presionado el toggle
                    self.b_boton_toggle_pole_ccw.set_active(False)

                    self.fr_etiqueta_msg.set_text("The set speed is 0 for this movement...")

                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    return
                
                if(self.b_ccwLimitPole == True):
                    self.s_mode = "STOP"
                    self.fr_etiqueta_msg.set_text("POLE CCW limit alarm!")
                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite anti-horario alcanzado en POLE")
                    depurador(1, "HMI", " ")

                    if self.b_beeps == True:
                        zumbador.beep_alarm()

                    return
                else:
                    # si no hay alarmas de límite por software desde RTU se mueve POLE CCW
                    self.s_mode = "FREE_RUN"
                    self.s_freeRunAxis = "POLE"
                    self.s_freeRunDir = "CCW_DIR"

                    if self.b_beeps == True:
                        zumbador.beep_button()

            if (b_toggle_pole_ccw == False):
                self.detener_movimientos()
                return
            
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Moviendo "+ self.s_freeRunAxis + " en sentido " + self.s_freeRunDir)
        depurador(1, "HMI", " ")
        self.fr_etiqueta_msg.set_text("Moving "+ self.s_freeRunAxis + " , " + self.s_freeRunDir)

        return True


    ##
    # @brief Función que administra los botones de control COL y ROW
    # en solapa Manual.
    # @param self Puntero al objeto HMI
    # @param button Botones de la solapa Manual, sección Guide Tube Position.
    def control_manual(self, button):
        global simu

        # Verificaciones previas al movimiento
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():

            if self.b_beeps == True:
                zumbador.beep_stop()

            return True
        
        # Antes de mover verifica si está activado el control principal
        if (self.s_ctrlEn == "DISABLE_CONTROL"):
            self.actualizar_etiquetas_msg("Fixture control is disabled...")

            if self.b_beeps == True:
                zumbador.beep_stop()

            return True
        
        # Antes de mover verifica si no está atascado el telemanipulador
        if (self.b_stallAlm):
            self.actualizar_etiquetas_msg("Fixture harness is stalled...")

            if self.b_beeps == True:
                zumbador.beep_stop()

            return True
        
        # TODO chequear ZS también
                
        # Si no hay inhabilitaciones se prosigue...

        # Obtiene el nombre del boton presionado
        s_boton_manual = button.get_name()

        # botones tipo pulsador que afectan COL, ROW deseadas
        if (s_boton_manual == "manual_col_(+)"):

            if self.b_beeps == True:
                zumbador.beep_button()

            # Se incrementa en una unidad la columna deseada
            self.ui_plan_col += 1

            # Se actualiza la etiqueta de COL deseada
            self.manual_etiqueta_valor_deseado_col.set_text(str(self.ui_plan_col))

            return

        if (s_boton_manual == "manual_col_(-)"):

            if self.b_beeps == True:
                zumbador.beep_button()

            # Se decrementa en una unidad la columna deseada
            self.ui_plan_col -= 1

            # Se controla que el valor no sea 0 o negativo
            if self.ui_plan_col <= 0:
                self.ui_plan_col = 1

            # Se actualiza la etiqueta de COL deseada
            self.manual_etiqueta_valor_deseado_col.set_text(str(self.ui_plan_col))

            return
        
        if (s_boton_manual == "manual_row_(+)"):

            if self.b_beeps == True:
                zumbador.beep_button()

            # Se incrementa en una unidad la fila deseada
            self.ui_plan_row += 1

            # Se actualiza la etiqueta de ROW deseada
            self.manual_etiqueta_valor_deseado_row.set_text(str(self.ui_plan_row))

            return
            
        if (s_boton_manual == "manual_row_(-)"):

            if self.b_beeps == True:
                zumbador.beep_button()

            # Se incrementa en una unidad la fila deseada
            self.ui_plan_row -= 1

            # Se controla que el valor no sea 0 o negativo
            if self.ui_plan_row <= 0:
                self.ui_plan_row = 1

            # Se actualiza la etiqueta de ROW deseada
            self.manual_etiqueta_valor_deseado_row.set_text(str(self.ui_plan_row))

            return

        if (s_boton_manual == "manual_col_(+)_row_(+)"):

            if self.b_beeps == True:
                zumbador.beep_button()

            # Se incrementan en una unidad COL y ROW deseadas
            self.ui_plan_col += 1
            self.ui_plan_row += 1

            # Se actualizan las etiquetas de COL, ROW deseadaS
            self.manual_etiqueta_valor_deseado_col.set_text(str(self.ui_plan_col))
            self.manual_etiqueta_valor_deseado_row.set_text(str(self.ui_plan_row))

            return

        if (s_boton_manual == "manual_col_(+)_row_(-)"):

            if self.b_beeps == True:
                zumbador.beep_button()

            # Se incrementa COL y se decrementea ROW
            self.ui_plan_col += 1
            self.ui_plan_row -= 1

            # Se controla que el valor de row no sea 0 o negativo
            if self.ui_plan_row <= 0:
                self.ui_plan_row = 1

            # Se actualizan las etiquetas de COL, ROW deseadaS
            self.manual_etiqueta_valor_deseado_col.set_text(str(self.ui_plan_col))
            self.manual_etiqueta_valor_deseado_row.set_text(str(self.ui_plan_row))

            return

        if (s_boton_manual == "manual_col_(-)_row_(-)"):

            if self.b_beeps == True:
                zumbador.beep_button()

            # Se decrementan COL y ROW
            self.ui_plan_col -= 1
            self.ui_plan_row -= 1

            # Se controla que el valor deseado no sea 0 o negativo
            if self.ui_plan_col <= 0:
                self.ui_plan_col = 1
            if self.ui_plan_row <= 0:
                self.ui_plan_row = 1

            # Se actualizan las etiquetas de COL, ROW deseadaS
            self.manual_etiqueta_valor_deseado_col.set_text(str(self.ui_plan_col))
            self.manual_etiqueta_valor_deseado_row.set_text(str(self.ui_plan_row))

            return

        if (s_boton_manual == "manual_col_(-)_row_(+)"):

            if self.b_beeps == True:
                zumbador.beep_button()

            # Se incrementa COL y se decrementea ROW
            self.ui_plan_col -= 1
            self.ui_plan_row += 1

            # Se controla que el valor de col no sea 0 o negativo
            if self.ui_plan_col <= 0:
                self.ui_plan_col = 1

            # Se actualizan las etiquetas de COL, ROW deseadaS
            self.manual_etiqueta_valor_deseado_col.set_text(str(self.ui_plan_col))
            self.manual_etiqueta_valor_deseado_row.set_text(str(self.ui_plan_row))

            return

        return True

    ##
    # @brief Función se ejecuta cada un minuto para actualizar las etiquetas de hora,
    # fecha y de mensajes de estado de conexiòn con RTU
    # @param self Puntero al objeto HMI
    def leer_reloj(self):
        s_fecha, s_hora = str(dt.datetime.now()).split(' ')
        s_anio, s_mes, s_dia = s_fecha.split('-')
        s_hr, s_min, s_seg = s_hora.split(':') 

        # Se actualizarn las etiquetas de hora y fecha en todas las solapas
        self.actualizar_etiquetas_reloj(s_hr+":"+s_min, s_dia+"/"+s_mes+"/"+s_anio) 

        # Si no hay conexión con RTU se avisa constantemente, si se conectó, la variable
        # print_status permite que se muestre el aviso de conexión solo una vez.
        if(self.b_connect  == False):
            self.actualizar_etiquetas_msg("Attempting to connect to NFC...")
            self.b_print_status = True
        elif(self.b_connect == True and self.b_print_status == True):
            if(self.b_beeps == True):
                zumbador.beep_primordial()

            self.actualizar_etiquetas_msg("HMI connected to NFC !")

            self.b_print_status = False

        return True       

    
    ##
    # @brief Función que se repite periódicamente, inicia la conexión con RTU
    # y gestiona el intercambio de datos entre HMI y RTU.
    # @param self Puntero al objeto HMI
    def telemetria(self):

        # Código de simulación de variación en ángulos actuales
        if self.f_posCmdPole < 0:
            self.f_posCmdPole = self.f_posCmdPole + 360
        if self.f_posCmdArm < 0:
            self.f_posCmdArm = self.f_posCmdArm + 360

        if self.f_posActArm == np.deg2rad(self.f_posCmdArm):
            pass
        else:
            if np.abs(np.deg2rad(self.f_posCmdArm)) > self.f_posActArm:
                self.f_posActArm += 0.125
                if self.f_posActArm >= np.deg2rad(360):
                    self.f_posActArm = 0
            elif np.abs(np.deg2rad(self.f_posCmdArm)) < self.f_posActArm:
                self.f_posActArm -= 0.125
                if self.f_posActArm < 0:
                    self.f_posActArm = 0

            if np.abs(self.f_posActArm - np.deg2rad(self.f_posCmdArm)) <= 0.125:
                self.f_posActArm = np.deg2rad(self.f_posCmdArm)

        if self.f_posActPole == np.deg2rad(self.f_posCmdPole):
            pass
        else:
            if np.abs(np.deg2rad(self.f_posCmdPole)) > self.f_posActPole:
                self.f_posActPole += 0.125
                if self.f_posActPole >= np.deg2rad(360):
                    self.f_posActPole = 0
            elif np.abs(np.deg2rad(self.f_posCmdPole)) < self.f_posActPole:
                self.f_posActPole -= 0.125
                if self.f_posActPole < 0:
                    self.f_posActPole = 0

            if np.abs(self.f_posActPole - np.deg2rad(self.f_posCmdPole)) <= 0.125:
                self.f_posActPole = np.deg2rad(self.f_posCmdPole)

        # Se actualizan las etiquetas de cuentas y ángulos en las tres solapas
        self.actualizar_etiquetas_enc_ang(np.rad2deg(self.f_posActPole), np.rad2deg(self.f_posActArm))
        # fin código de simulación de ángulos actuales SM-13


        # Se evita que el intento de conexión tenga una frecuencia tan alta
        # como la del envío-recepción de tramas.
        if (self.ui_intentos >= self.ui_REINTENTAR):
            self.ui_intentos = 0
            
        if (self.b_connect == False) and (self.ui_intentos == 0):
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Intentando conectar con RTU...")
            depurador(1, "HMI", "- IP actual: " + self.s_ip + " / Port: " + self.s_port)
            try:
                self.s_sock, self.b_connect = RTU_connect(self.b_connect, self.s_ip, int(self.s_port))
                pass
            except socket.error as e:
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- No se puede establecer comunicación con RTU: " + str(e))
                depurador(1, "HMI", " ")
                
            depurador(1, "HMI", "- Estado conexión con RTU: " + str(self.b_connect))
            depurador(1, "HMI", " ")

        elif (self.b_connect == True):
            try:
                # Se refresca el paquete de DataBytes a enviar a RTU
                self.a_HMIDataByte = self.f_posCmdArm, self.f_posCmdPole, self.ui_velCmdArm, self.ui_velCmdPole
                
                # Se refresca el paquete de DataString a enviar a RTU
                self.a_HMIDataString = self.s_mode, self.s_freeRunAxis, self.s_freeRunDir, self.s_ctrlEn, self.s_stallEn, self.s_liftDir
                
                # Se envían los paquetes DataBytes y DataStrings hacia RTU y se recibe un paquete proveniente de RTU 
                self.a_RTUData, self.b_connect, self.s_sock = enviar_a_y_recibir_de_rtu(self.a_HMIDataByte, self.a_HMIDataString, self.b_connect, self.s_sock, '192.168.0.193', 5020)
                depurador(3, "HMI", "--> Paquete Data_Byte  : " + str(self.a_HMIDataByte))
                depurador(3, "HMI", "--> Paquete Data_String: " + str(self.a_HMIDataString))
                depurador(3, "HMI", "<-- Paquete RTU_Data   : " + str(self.a_RTUData))
                
                if self.b_connect == True:
                    self.f_posActArm = self.a_RTUData[0]
                    self.f_posActPole = self.a_RTUData[1]
                    self.f_velActArm = self.a_RTUData[2]
                    self.f_velActPole = self.a_RTUData[3]
                    self.b_cwLimitArm = self.a_RTUData[4]
                    self.b_ccwLimitArm = self.a_RTUData[5]
                    self.b_cwLimitPole = self.a_RTUData[6]
                    self.b_ccwLimitPole = self.a_RTUData[7]
                    self.b_limitUp = self.a_RTUData[8]
                    self.b_limitDown = self.a_RTUData[9]
                    self.b_stallAlm = self.a_RTUData[10]
                    self.ui_status = self.a_RTUData[11]
                    
                    # Se actualizan las etiquetas de cuentas y ángulos en las tres solapas
                    self.actualizar_etiquetas_enc_ang(self.f_posActPole, self.f_posActArm)
                               
                    
            except Exception as e:
                self.b_connect = False
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Error durante la comunicación con RTU: " + str(e))
                depurador(1, "HMI", " ")
        
        self.ui_intentos += 1
                
        return True
                
                
if __name__ == "__main__":
    hmi_SM13()
    Gtk.main()
    
    
