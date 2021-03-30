#!/usr/bin/env python3

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
#
#                           RX FROM RTU
#   --------------------------------------
#   f_resActArm         <--     a_RTUDataRx[0] *
#   f_resActPole        <--     a_RTUDataRx[1] *
#   f_temperatura       <--     a_RTUDataRx[2] *
#   f_velActPole        <--     a_RTUDataRx[3] *
#   b_cwLimitArm        <--     a_RTUDataRx[4] *
#   b_ccwLimitArm       <--     a_RTUDataRx[5] *
#   b_cwLimitPole       <--     a_RTUDataRx[6] *
#   b_ccwLimitPole      <--     a_RTUDataRx[7] *
#   b_limitUp           <--     a_RTUDataRx[8] *
#   b_limitDown         <--     a_RTUDataRx[9] *
#   b_stallAlm          <--     a_RTUDataRx[10] *
#   b_onCondition       <--     a_RTUDataRx[11] *
#   ui_armRdcStatus     <--     a_RTUDataRx[12] *
#   ui_poleRdcStatus    <--     a_RTUDataRx[13] *
#   ui_status           <--     a_RTUDataRx[14] *


#
#   TX TO RTU                           
#   --------------------------------------
#   a_HMIDataByte[0]    <--     f_posCmdArm *
#   a_HMIDataByte[1]    <--     f_posCmdPole *
#   a_HMIDataByte[2]    <--     ui_velCmdPole *
#   a_HMIDataByte[3]    <--     ui_velCmdPole *
#   a_HMIDataString[0]  <--     s_mode *
#   a_HMIDataString[1]  <--     s_freeRunAxis *
#   a_HMIDataString[2]  <--     s_freeRunDir *
#   a_HMIDataString[3]  <--     s_ctrlEn *
#   a_HMIDataString[4]  <--     s_stallEn *
#   a_HMIDataString[5]  <--     s_liftDir *
#   a_HMIDataString[6]  <--     s_setCal *
#   a_HMIDataString[7]  <--     s_cmdFlag *
    
#
#   IMPORTANTE TENER EN CUENTA:
#   HMI to RTU
#   a_DataByte[0] (ARM  en grados) ---> a_DataByteTx[0] (convertido a cuentas resolver y enviado por trama. Se tiene en cuenta offset)
#   a_DataByte[1] (POLE en grados) ---> a_DataByteTx[0] (convertido a cuentas resolver y enviado por trama. Se tiene en cuenta offset)
#   Dentro de HMIcomRTU a_DataByteTx_MSB = MSB(a_DataByteTx[0]) y a_DataByteTx_LSB = LSB(a_DataByteTx[0]). Lo mismo para a_DataByteTx[1]
#
#   RTU to HMI
#   a_RTUDataRx[0] (cuentas resolver ARM ) ---> a_RTUDataRx[0] (se convierte a grados teniendo en cuenta offsets)
#   a_RTUDataRx[1] (cuentas resolver POLE) ---> a_RTUDataRx[1] (se convierte a grados teniendo en cuenta offsets)
#   Dentro de HMIcomRTU a_RTUDataRx[0]=a_RTUDataRxRx_MSB+a_RTUDataRxRx_LSB. Lo mismo para a_RTUDataRx[1].

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import GLib, Gtk, Gdk, GObject
import os
import time
from simulador_SM13 import *
from ik_SM13 import *
from leer_archivo_hx import *
from leer_plan import *
from leer_specs_tubos import *
from conversor import *
from depurador import *
from gi.repository import Pango
import sys
import numpy as np
import time
from HMIcomRTU import *
import socket
import zumbador
import multiprocessing
import threading
import registrador

global simu

global a_HMIDataByte
global a_HMIDataString
global b_connect
global b_on_condition
global s_sock
global s_port
global s_ip
global ui_pole_rdc_offset
global ui_arm_rdc_offset

# Variables asociadas a los valores provenientes desde la RTU vía ethernet.
# Valores ethernet desde RTU
global f_resActArm
global f_resActPole
global f_angActPole
global f_angActArm
global f_temperatura 
global b_cwLimitArm 
global b_ccwLimitArm 
global b_cwLimitPole 
global b_ccwLimitPole 
global b_limitUp 
global b_limitDown 
global b_stallAlm 
global b_onCondition 
global ui_armRdcStatus 
global ui_poleRdcStatus
global ui_rtuStatus 

# Valores ethernet desde RTU   
f_angActArm = 0
f_angActPole = 0 
f_temperatura = 0  
b_cwLimitArm = 0 
b_ccwLimitArm = 0 
b_cwLimitPole = 0 
b_ccwLimitPole = 0 
b_limitUp = 0 
b_limitDown = 0 
b_stallAlm = 0 
b_onCondition = 0 
ui_armRdcStatus = 0 
ui_poleRdcStatus = 0
ui_rtuStatus = 0 


class hmi_SM13():   
    registrador.info(" ")
    registrador.info("********* Starting HUMAN-MACHINE-INTERFACE for SM-13 **********")
    ## El constructor del HMI
    def __init__(self):
        global simu
        global a_HMIDataByte
        global a_HMIDataString
        global b_connect
        global b_on_condition
        global s_sock
        global s_port
        global s_ip
        global b_simulador
        global ui_pole_rdc_offset
        global ui_arm_rdc_offset
        
        # Valores ethernet desde RTU
        global f_angActArm
        global f_angActPole
        
        
        # Se inicializan variables globales comartidas por HMI y TM
        ## Variable global contiene la lista de datos que llegan desde RTU:
        # [f_angActArm, f_angActPole, f_velActArm, f_velActPole, 
        # b_cwLimitArm, b_ccwLimitArm, b_cwLimitPole, b_ccwLimitPole, b_limitUp, b_limitDown, b_stallAlm, b_onCondition, 
        # ui_armRdcStatus, ui_poleRdcStatus, ui_rtuStatus ]
        f_angActArm = 0;
        f_angActPole = 0;
        
        
        
        
        #   f_resActArm     <--     a_RTUDataRx[0] *
        #   f_resActPole    <--     a_RTUDataRx[1] *
        #   f_temperatura   <--     a_RTUDataRx[2] *
        #   f_velActPole    <--     a_RTUDataRx[3] *
        #   b_cwLimitArm    <--     a_RTUDataRx[4] *
        #   b_ccwLimitArm   <--     a_RTUDataRx[5] *
        #   b_cwLimitPole   <--     a_RTUDataRx[6] *
        #   b_ccwLimitPole  <--     a_RTUDataRx[7] *
        #   b_limitUp       <--     a_RTUDataRx[8] *
        #   b_limitDown     <--     a_RTUDataRx[9] *
        #   b_stallAlm      <--     a_RTUDataRx[10] *
        #   b_onCondition   <--     a_RTUDataRx[11] *
        #   ui_armRdcStatus  <--     a_RTUDataRx[12] *
        #   ui_poleRdcStatus <--     a_RTUDataRx[13] *
        #   ui_rtuStatus     <--     a_RTUDataRx[14] *
        ## Variable global para que la vea HMI y el hilo TM. Almacena los datos 
        # de ángulos y velocidad comandados hacia la RTU:
        # [f_posCmdArm, f_posCmdPole, ui_velCmdArm, ui_velCmdPole]
        a_HMIDataByte = [0, 0, 0, 0]
        ## Variable global para que la vea HMI y el hilo TM. Almacena una lista de  
        #  eventos que surguen desde HMI y son comunicados a RTU:
        # [s_mode, s_freeRunAxis, s_freeRunDir, s_ctrlEn, s_stallEn, s_liftDir]
        a_HMIDataString = ["STOP", "ARM", "DIR_CW", "DISABLE_CONTROL", "STALL_ENABLE", "LIFT_UP","NOP_CAL"]
        ## Variable global la ve tanto HMI como TM. Contiene el estado del simulador
        # 0: simulador inactivo
        # 1: simulador activo
        b_simulador = 0
        ## Variable global la ve tanto HMI como TM. Indica si hay poco error de cuenta entre CMD y ACT
        # 0: diferencia mayor que ui_CMD_ACT_ERROR
        # 1: Fixture is ON_CONDITION (diferencia menor que ui_CMD_ACT_ERROR)
        b_on_condition = False
        ## Variable contiene el valor de offset para el eje POLE configurado al inicio cuando levanta
        # el archivo robots_home_offsets.csv 
        ui_pole_rdc_offset = 0
        ## Variable contiene el valor de offset para el eje ARM configurado al inicio cuando levanta
        # el archivo robots_home_offsets.csv 
        ui_arm_rdc_offset = 0


        # CONSTANTES DEL SISTEMA
        ## Constante contiene el tiempo para actualizar etiquetas del reloj
        self.ui_REFRESCO_ms_RELOJ = 500
        ## Constante contiene el tiempo de refresco de valores actuales de ángulos POLE y ARM del 
        # simulador
        self.ui_REFRESCO_ms_SIMULADOR = 1000
        ## Constante indica el tiempo en que una variable queda activada antes
        # de pasar el modo a STOP otra vez
        self.ui_DELAY_ms_PULSADOR = 300
        ## Constante grados máximos de una vuelta de ARM o POLE
        self.f_MAX_GRADOS = 360
        ## Constante máximo valor de conversión del RDC (16 bits)
        self.ui_MAX_CUENTAS = 65536 
        ## Constante contiene el ángulo de STOW para POLE              
        self.f_POLE_STOW_ANG = 1
        ## Constante contiene el ángulo de STOW para ARM
        self.f_ARM_STOW_ANG = 1
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
        self.f_pole = 0.0
        ## Contiene el ángulo de rotación de la articulación ARM respecto a la base del telemanipulador.
        self.f_arm = 0.0
        ## Contiene las cuentas "teóricas" del POLE respecto a la base del telemanipulador para propósitos de calibración
        self.ui_pole_res_for_cal = 0
        ## Contiene las cuentas "teóricas" del ARM respecto a la base del telemanipulador para propósitos de calibración
        self.ui_arm_res_for_cal = 0
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
        #a_HMIDataByte[2] = 0
        ## Variable contiene la velocidad deseada con que se quiere mover el FH-POLE en modo FR.
        #a_HMIDataByte[3] = 0
        ## Variable booleana indica el estado del ZS inferior del eje LIFT
        self.b_limitDwn = False
        ## Variable booleana para enclavar la impresión una sola vez, tanto de pantalla como en log
        self.b_print_status = False
        ## Variable tipo boleana que indica el estado de la conexión con la RTU
        # 0: Desconexión
        # 1: Conexión
        b_connect = False
        ## Variable permite iniciar el TreeView una sola vez
        self.b_carga = 0
        ## Variable booleana para permitir o rechazar sonidos de la interfaz
        self.b_beeps = True
        ## Variable booleana indica si los càlculos de cinemática inversa fueron exitosos o no
        self.b_ik_success = False
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
        ## Variable contiene el tipo de solución matemática que va a usar el
        # algoritmo de cinemática inversa para que el quiebre del ARM se de la
        # forma principal o alternativa.
        self.s_name_pivot_type = "main"
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
        s_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ## Variable usada para guardar la dirección IP de la RTU
        s_ip = "0.0.0.0"
        ## Variable usada para guardar el puerto de conexión con la RTU
        s_port = "0"
        
        # Se abre el archivo network.csv en modo lectura, se lo lee y se cierra. 
        network_file = open(self.s_project_path + "/hmi/cfg_files/network.csv", "r")
        address_lines = network_file.readlines()
        network_file.close()
        # Se extraen la variables que contendrán la dir IP y el PUERTO
        s_ip, s_port = address_lines[1].split(";")
        
        
        # Todas las señales que manejan los eventos que surgen de presionar botones en la HMI.
        señales = {
            "terminar_aplicacion":self.cerrar_hmi,
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
        s_ip_3, s_ip_2, s_ip_1, s_ip_0 = s_ip.split(".")
        # y se muestran en el "entry" de la ventana de cfg de red.
        self.entrada_ip_0.set_value(int(s_ip_0))
        self.entrada_ip_1.set_value(int(s_ip_1))
        self.entrada_ip_2.set_value(int(s_ip_2))
        self.entrada_ip_3.set_value(int(s_ip_3))
        self.entrada_puerto.set_value(int(s_port))
        
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

        ## Etiqueta dinámica que muestra el estado de conexión HMI-RTU
        self.inicio_etiqueta_estado_red = self.builder.get_object("inicio_etiqueta_estado_red")
        
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
        
        ## Etiqueta de actualización de Temperatura en solapa manual
        self.manual_etiqueta_valor_temperatura = self.builder.get_object("manual_etiqueta_valor_temperatura")
        
        ## Etiqueta de actualización de Temperatura en solapa inspection
        self.inspection_etiqueta_valor_temperatura = self.builder.get_object("inspection_etiqueta_valor_temperatura")
        
        ## Ventana principal del HMI
        self.window = self.builder.get_object("ventana_principal")
        #win_name = Gtk.Label("SM-13 HUMAN-MACHINE INTERFACE")
        #self.window.add(win_name)
        
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
        for x in range(9):
            self.ui_fr_escala_vel_arm.add_mark(x, Gtk.PositionType.LEFT, str(x).ljust(4, ' '))
        
        ## Widget escala vel
        self.ui_fr_escala_vel_pole = self.builder.get_object("fr_escala_vel_pole")
        # Se agregan las marcas de escala
        for x in range(9):
            self.ui_fr_escala_vel_pole.add_mark(x, Gtk.PositionType.RIGHT, str(x).rjust(4, ' '))

        # Warning: deprecated!
        #self.window2.override_background_color(0, Gdk.RGBA(0.9,0.0,0.0,1.0))
        
        # Mensaje de bienvenida
        self.actualizar_etiquetas_msg("Welcome to the NFC Human-Machine Interface!", "green")
        self.window.show()

        # Se solicita actualizar las etiquetas del reloj y luego refrescarlas cada 
        # REFRESCO_ms_RELOJ milisegundos
        self.leer_reloj()
        GLib.timeout_add(self.ui_REFRESCO_ms_RELOJ, self.leer_reloj)
        # Beep 
        zumbador.beep_primordial()

         # Función refrescar_simulador se ejecuta cada REFRESCO_ms_SIMULADOR milisegundos
        GLib.timeout_add(self.ui_REFRESCO_ms_SIMULADOR, self.refrescar_simulador)

    ##
    # @brief Función que implementa el botón "Sounds" para activar/desactivar alertas sonoras del sistema
    # @param self Puntero al objeto HMI
    # @param button "Sounds"
    # @return none
    def control_zumbador(self, button):
        # se verifica si el botón esta presionado o suelto
        b_toggle_zumbador = button.get_active()
        depurador(3, "HMI", "****************************************")

        if (b_toggle_zumbador == True):
            self.b_beeps = True
            self.actualizar_etiquetas_msg("HMI beeps enabled", "green")
            self.inicio_etiqueta_toggle_zumbador.set_text("Sounds Yes")
            depurador(1, "HMI", "- zumbador activado")
            zumbador.beep_button()
            

        else:
            self.b_beeps = False
            self.actualizar_etiquetas_msg("HMI beeps disabled", "green")
            self.inicio_etiqueta_toggle_zumbador.set_text("Sounds No ")
            depurador(1, "HMI", "- zumbador desactivado")

        depurador(1, "HMI", " ")

        return True
    
    
    ##
    # @brief Función que implementa los botones toggle "Main control" y "Stall"
    # y el botón pulsador Stow
    # @param self Puntero al objeto HMI
    # @param button "Main control", "Stall Function" o "Stow"
    # @return none
    def fixture_control(self, button):
        global simu
        
        global a_HMIDataByte
        global a_HMIDataString
        global b_connect
        global s_sock
        global s_port
        global s_ip
        global b_on_condition
        global b_stallAlm
        
        # Obtiene el nombre del widget switch presionado
        s_name = button.get_name()

        if (s_name == "fixture_control_button"):    
            if button.get_active():
                a_HMIDataString[3] = "CONTROL_ENABLE"
                self.inicio_etiqueta_estado_main_control.set_text("Enabled")
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg("Fixture harness main control enabled...", "green")
                registrador.info("Fixture harness main control is enabled")

                if self.b_beeps == True:
                    zumbador.beep_button()

            else:
                # Se limpia la bandera de on_condition
                b_on_condition = False
                a_HMIDataString[3] = "DISABLE_CONTROL"
                self.inicio_etiqueta_estado_main_control.set_text("Disabled")
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg("Fixture harness main control disabled...", "red")
                registrador.info("Fixture harness main control is disabled")

                if self.b_beeps == True:
                    zumbador.beep_stop()

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Fixture status = " + a_HMIDataString[3])
            depurador(1, "HMI", " ")
            
        elif (s_name == "fixture_stall_button"):
            if button.get_active():
                a_HMIDataString[4] = "STALL_ENABLE"
                self.inicio_etiqueta_estado_stall.set_text("Enabled")
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg("Fixture harness stall function enabled...", "green")
                registrador.info("Fixture harness stall function is enabled")

                if self.b_beeps == True:
                    zumbador.beep_button()

            else:
                # Se limpia la bandera de on_condition
                b_on_condition = False
                a_HMIDataString[4] = "DISABLE_STALL"
                self.inicio_etiqueta_estado_stall.set_text("Disabled")
                self.actualizar_etiquetas_msg("Fixture harness stall function disabled...", "red")
                registrador.info("Fixture harness stall function is disabled")                

                if self.b_beeps == True:
                    zumbador.beep_button()

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Fixture status = " + a_HMIDataString[4])
            depurador(1, "HMI", " ")

        elif (s_name == "fixture_stow_button"):
            # Antes de mover verifica si está activado el control principal
            if (a_HMIDataString[3] == "DISABLE_CONTROL"):
                if self.b_beeps == True:
                    zumbador.beep_stop()

                self.actualizar_etiquetas_msg("Fixture control is disabled...", "red")

                return True
            
            # Antes de mover verifica si no está atascado el telemanipulador
            elif (b_stallAlm == True and a_HMIDataString[4] == "STALL_ENABLE"):
                if self.b_beeps == True:
                    zumbador.beep_stop()

                # En caso de atasque, se desactiva el control principal, se despinta el botón y se advierte
                a_HMIDataString[3] = "DISABLE_CONTROL"
                self.inicio_toggle_fixture_control.set_active(False)
                self.inicio_etiqueta_estado_main_control.set_text("Disabled")
                # Se limpia la bandera de on_condition
                b_on_condition = False
                # Se indica en todas las etiquetas de msg del hmi
                self.actualizar_etiquetas_msg("Fixture harness is stalled, main control disabled...", "red")
                registrador.warning("Stall alarm, stow movement denied")

                return True   

            else:
                # Si se permite, se mueven los ejes POLE y ARM a la posición de reposo
                a_HMIDataByte[1] = self.f_POLE_STOW_ANG # para evitar que la cuenta de resolver rebalse, se evita mandar a 0º justo sino que se corta un poco antes
                a_HMIDataByte[0] = self.f_ARM_STOW_ANG
                simu.refrescar_pos_comandada(np.deg2rad(a_HMIDataByte[1]), np.deg2rad(a_HMIDataByte[0]))

                b_on_condition = False
                self.actualizar_etiquetas_msg("Moving robot to STOW position...", "green")
                registrador.info("Moving robot to STOW position")
                
                a_HMIDataString[0] = "AUTOMATIC"

        elif (s_name == "fixture_pivot_button"):
            if self.b_beeps == True:
                    zumbador.beep_button()

            if (self.s_name_pivot_type == "main"):
                self.s_name_pivot_type = "alternative"
            elif (self.s_name_pivot_type == "alternative"):
                self.s_name_pivot_type = "main"

            self.actualizar_etiquetas_msg("Fixture " + self.s_name_pivot_type + " pivot type selected", "green")
            self.inicio_etiqueta_estado_pivot.set_text(self.s_name_pivot_type)
            registrador.info("Fixture " + self.s_name_pivot_type + " pivot type selected")

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Fixture pivot = " + self.s_name_pivot_type)
            depurador(1, "HMI", " ")
        
        return True
    
    ##
    # @brief Función que implementa todos los botones de STOP del HMI
    # @param self Puntero al objeto HMI
    # @param button "STOP"
    # @return none
    def stop_total(self, button):
        global a_HMIDataString

        # Desenergiza relé principal en NFC
        if(self.b_beeps == True):
            # Beep 
            zumbador.beep_stop()

        a_HMIDataString[3] = "DISABLE_CONTROL"
        # Actualiza la variable modo
        a_HMIDataString[0] = "STOP"
        # Cambia a OFF el widget del interruptor principal
        self.inicio_toggle_fixture_control.set_active(False)
        # Cambia el mensaje de la etiqueta del interruptor
        self.inicio_etiqueta_estado_main_control.set_text("Disabled")
        # Se limpia la bandera de on_condition
        b_on_condition = False
        ## Se indica la acción en las etiquetas principales de msg.
        self.actualizar_etiquetas_msg("Stop, fixture harness main control disabled...", "red")
        registrador.info("Total stop, fixture harness main control disabled.")

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
        depurador(1, "HMI", "- Fixture status = " + a_HMIDataString[3])
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
            depurador(1, "HMI", "- Se seleccionó "+self.s_tubo_id+", ROW:"+str(self.ui_plan_row)+(", COL:"+str(self.ui_plan_col)))
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
        
        global a_HMIDataString
        global a_HMIDataByte
        global b_simulador
        global b_on_condition
        
        # Verificaciones previas al movimiento
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():
            return True
        
        # Antes de mover verifica si está activado el control principal
        if (a_HMIDataString[3] == "DISABLE_CONTROL"):
            if self.b_beeps == True:
                zumbador.beep_stop()

            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("Fixture control is disabled...", "red")

            return True
        
        # TODO chequear ZS también

        if self.b_beeps == True:
                zumbador.beep_button()
        
        # Cálculo cinemático de los angulos necesarios para alcanzar COL, ROW deseados
        # en base a el archivo de intercambiador elegido y la posición de montaje del robot
        if (self.ui_plan_col != 0 and self.ui_plan_row != 0):
            # Extrae las distancias x,y desde el archivo del hx (en pulgadas) según tubo seleccionado
            self.f_px_tubo, self.f_py_tubo, self.a_lista_px, self.a_lista_py, b_error_coordenada = leer_archivo_hx(self.ui_plan_col, self.ui_plan_row, self.s_archivo_hx)
            depurador(1, "HMI", "- Px sin jog = " + str(self.f_px_tubo) + " in")
            depurador(1, "HMI", "- Py sin jog = " + str(self.f_py_tubo) + " in")

            self.corregir_ordenadas_plan_par()

            #if self.s_archivo_plan == 

            # Se agrega el jog si lo hubiere
            depurador(1, "HMI", "- Jog aplicado a Px = " + str(round(self.f_incremento_acumulado_jog_col*self.f_x_pitch, 2)) + " in")
            self.f_px_tubo += self.f_incremento_acumulado_jog_col*self.f_x_pitch
            depurador(1, "HMI", "- Jog aplicado a Py = " + str(round(self.f_incremento_acumulado_jog_row*self.f_y_pitch, 2)) + " in")
            self.f_py_tubo += self.f_incremento_acumulado_jog_row*self.f_y_pitch

            depurador(1, "HMI", "- Px con jog = " + str(self.f_px_tubo) + " in")
            depurador(1, "HMI", "- Py con jog = " + str(self.f_py_tubo) + " in")
        else:
            # Posición inicial de la boquilla al arrancar el simulador 
            self.f_px_tubo = self.f_Lx - self.f_incremento_acumulado_jog_col*self.f_x_pitch
            self.f_py_tubo = self.f_Ly - self.f_Lp + self.f_La - self.f_incremento_acumulado_jog_row*self.f_y_pitch

        # Si no existe coordenada X,Y para la selección COL,ROW no se ingresa al algoritmo de cinemática inversa
        # a calcular nuevos ángulos.
        if not b_error_coordenada:
            # Calcula los ángulos de los ejes POLE y ARM en base a la distancia que hay que alcanzar
            self.f_pole, self.f_arm, self.b_ik_success = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, self.s_name_pivot_type)

            # Se convierten los ágnulos de las articulaciones a grados y se redondea a 3 decimales
            self.f_pole = round(np.rad2deg(self.f_pole), 3)
            self.f_arm = round(np.rad2deg(self.f_arm), 3)

        else:
            if self.b_beeps == True:
                zumbador.beep_stop()

            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("Can not reach that position...", "red")
            registrador.error("Can't find coordinate [px, py] at selected col " + str(self.ui_plan_col) + ", row " + str(self.ui_plan_row))
            return True


        if not self.b_ik_success:
            if self.b_beeps == True:
                zumbador.beep_stop()

            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("Can not reach that position...", "red")
            registrador.error("Inverse kinematics error at px " + str(self.f_px_tubo) + ", py " + str(self.f_py_tubo))

            return True

        else:
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(self.f_pole)+"°"+", ARM = "+str(self.f_arm)+"°")
            depurador(1, "HMI", "- Para ubicar boquilla en Px = "+str(round(self.f_px_tubo, 3))+" in"+", Py = "+str(round(self.f_py_tubo, 3))+" in")
            depurador(1, "HMI", " ") 
            
            
            depurador(1, "HMI", "- Moviendo hacia "+ self.s_tubo_id + " (COL : "+ str(self.ui_plan_col) + ", ROW : "+str(self.ui_plan_row) + ")")
            depurador(1, "HMI", " ")
            
            b_on_condition = False
            self.actualizar_etiquetas_msg("Moving to tube ROW : "+ str(self.ui_plan_row) + ", COL : "+str(self.ui_plan_col), "green")
            registrador.info("Moving to tube ROW : "+ str(self.ui_plan_row) + ", COL : "+str(self.ui_plan_col))
            
            
            if(b_simulador == 1):
                depurador(1, "HMI", " - Nuevo ángulo POLE = " + str(self.f_pole))
                depurador(1, "HMI", " - Nuevo ángulo ARM  = " + str(self.f_arm))
                simu.refrescar_pos_comandada(np.deg2rad(self.f_pole), np.deg2rad(self.f_arm))
                depurador(3, "HMI", "****************************************")
                depurador(3, "HMI", "- Refrescando simulador SM-13")
                depurador(3, "HMI", " ")
            
            # Se convierten a [º] los ángulos POLE y ARM, se redondea a 3 decimales y
            # se actualizan las variables posCmd así las toma el módulo HMIcomRTU y las envía a RTU.    
            a_HMIDataByte[1] = self.f_pole
            a_HMIDataByte[0] = self.f_arm

            # Para propósitos de calibración se guarda el ángulo de un tubo que podría ser utilizado para calibrar el offset del sistema
            # y se lo pasa a cuentas (considerando 0 cuentas en 0º)
            self.ui_pole_res_for_cal = (self.f_pole*self.ui_MAX_CUENTAS)/self.f_MAX_GRADOS
            self.ui_arm_res_for_cal = (self.f_arm*self.ui_MAX_CUENTAS)/self.f_MAX_GRADOS

           
            # Si todo salió bien, se activa el modo Automático, pero antes se asegura aparagar motor LIFT
            if a_HMIDataString[0] == "LIFT":
                a_HMIDataString[0] = "STOP"
                self.actualizar_etiquetas_msg("Stopping LIFT first, try again", "red")
                self.b_fr_boton_lift_up_total.set_active(False)
                self.b_fr_boton_lift_down_total.set_active(False)
                self.b_manual_boton_lift_up_total.set_active(False)
                self.b_manual_boton_lift_down_total.set_active(False)
                self.b_inspection_boton_lift_up_total.set_active(False)
                self.b_inspection_boton_lift_down_total.set_active(False)
            else:
                a_HMIDataString[0] = "AUTOMATIC"

            # Al presionar botón "Select Tube" se envían los ángulos al robot y se igualan 
            # los valores de COL y ROW desedos con actuales.
            self.manual_etiqueta_valor_actual_col.set_text(str(self.ui_plan_col)) 
            self.manual_etiqueta_valor_actual_row.set_text(str(self.ui_plan_row))
        
            return True
    
    ##
    # @brief Función que corrige la posición py de los tubos con plan de inspección par. 
    # @param self Puntero al objeto HMI
    # @return none
    def corregir_ordenadas_plan_par(self):
        s_name_pi_montaje = self.s_archivo_plan.split('/')            
        s_name_pi = str(s_name_pi_montaje[len(s_name_pi_montaje)-1]).split('_')
        s_pi = s_name_pi[0]
        if s_pi == "InspPlan2" or s_pi == "InspPlan4" or s_pi == "InspPlan6":
            self.f_py_tubo = self.f_py_tubo - 9.0 - (self.ui_plan_row - 1)*2*0.6625

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
        
        global a_HMIDataString
        global a_HMIDataByte
        global b_simulador
        global b_on_condition
        
        # Verificaciones previas al movimiento
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():
            return True
        
        # Antes de mover verifica si está activado el control principal
        if (a_HMIDataString[3] == "DISABLE_CONTROL"):
            if self.b_beeps == True:
                zumbador.beep_stop()

            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("Fixture control is disabled...", "red")

            return True
        
        # TODO chequear ZS también

        if self.b_beeps == True:
                zumbador.beep_button()
        
        # Se obtiene el objeto de selección de la vista
        selection = self.inspection_tree_view.get_selection()
        # Se obtiene la fila actualmente seleccionada
        sel = selection.get_selected()
        if not sel[1] == None:
            # Se obtiene self.f_poleel próximo iter
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
            depurador(1, "HMI", "- Px sin jog = " + str(self.f_px_tubo) + " in")
            depurador(1, "HMI", "- Py sin jog = " + str(self.f_py_tubo) + " in")

            self.corregir_ordenadas_plan_par()

            # Se agrega el jog si lo hubiere
            depurador(1, "HMI", "- Jog aplicado a Px = " + str(round(self.f_incremento_acumulado_jog_col*self.f_x_pitch, 2)) + " in")
            self.f_px_tubo += self.f_incremento_acumulado_jog_col*self.f_x_pitch
            depurador(1, "HMI", "- Jog aplicado a Py = " + str(round(self.f_incremento_acumulado_jog_row*self.f_y_pitch, 2)) + " in")
            self.f_py_tubo += self.f_incremento_acumulado_jog_row*self.f_y_pitch

            depurador(1, "HMI", "- Px con jog = " + str(self.f_px_tubo) + " in")
            depurador(1, "HMI", "- Py con jog = " + str(self.f_py_tubo) + " in")
        else:
            # Posición inicial de la boquilla al arrancar el simulador 
            self.f_px_tubo = self.f_Lx - self.f_incremento_acumulado_jog_col*self.f_x_pitch
            self.f_py_tubo = self.f_Ly - self.f_Lp + self.f_La - self.f_incremento_acumulado_jog_row*self.f_y_pitch

        # Calcula los ángulos de los ejes POLE y ARM en base a la distancia que hay que alcanzar
        self.f_pole, self.f_arm, self.b_ik_success = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, self.s_name_pivot_type)
        
        # Se convierten los ágnulos de las articulaciones a grados y se redondea a 3 decimales
        self.f_pole = round(np.rad2deg(self.f_pole), 3)
        self.f_arm = round(np.rad2deg(self.f_arm), 3)

        if not self.b_ik_success:
            if self.b_beeps == True:
                zumbador.beep_stop()

            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("Can not reach that position...", "red")
            registrador.error(" Inverse kinematics error at px " + str(self.f_px_tubo) + ", py " + str(self.f_py_tubo))
            return True

        else:
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(self.f_pole)+"°"+", ARM = "+str(self.f_arm)+"°")
            depurador(1, "HMI", "- Para ubicar boquilla en Px = "+str(round(self.f_px_tubo, 3))+" in"+", Py = "+str(round(self.f_py_tubo, 3))+" in")
            depurador(1, "HMI", " ") 


            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Moviendo hacia "+ self.s_tubo_id + " (COL : "+ str(self.ui_plan_col) + ", ROW : "+str(self.ui_plan_row) + ")")
            depurador(1, "HMI", " ")
            
            b_on_condition = False
            self.actualizar_etiquetas_msg("Moving to tube ROW : "+ str(self.ui_plan_row) + ", COL : "+str(self.ui_plan_col), "green")
            registrador.info("Moving to tube ROW : "+ str(self.ui_plan_row) + ", COL : "+str(self.ui_plan_col))
            
            if(b_simulador == 1):
                depurador(1, "HMI", " - Nuevo ángulo POLE = " + str(self.f_pole))
                depurador(1, "HMI", " - Nuevo ángulo ARM  = " + str(self.f_arm))
                simu.refrescar_pos_comandada(np.deg2rad(self.f_pole), np.deg2rad(self.f_arm))
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Refrescando simulador SM-13")
                depurador(1, "HMI", " ")
                
            # Se convierten a [º] los ángulos POLE y ARM, se redondea a 3 decimales y
            # se actualizan las variables posCmd así las toma el módulo HMIcomRTU y las envía a RTU.    
            a_HMIDataByte[1] = self.f_pole
            a_HMIDataByte[0] = self.f_arm

            # Para propósitos de calibración se guarda el ángulo de un tubo que podría ser utilizado para calibrar el offset del sistema
            # y se lo pasa a cuentas (considerando 0 cuentas en 0º)
            self.ui_pole_res_for_cal = (self.f_pole*self.ui_MAX_CUENTAS)/self.f_MAX_GRADOS
            self.ui_arm_res_for_cal = (self.f_arm*self.ui_MAX_CUENTAS)/self.f_MAX_GRADOS

            # Si todo salió bien, se activa el modo Automático, pero antes se asegura aparagar motor LIFT
            if a_HMIDataString[0] == "LIFT":
                a_HMIDataString[0] = "STOP"
                self.actualizar_etiquetas_msg("Stopping LIFT first, try again...", "red")
                self.b_fr_boton_lift_up_total.set_active(False)
                self.b_fr_boton_lift_down_total.set_active(False)
                self.b_manual_boton_lift_up_total.set_active(False)
                self.b_manual_boton_lift_down_total.set_active(False)
                self.b_inspection_boton_lift_up_total.set_active(False)
                self.b_inspection_boton_lift_down_total.set_active(False)
            else:
                a_HMIDataString[0] = "AUTOMATIC"

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
        global b_simulador
        
        # no arranco el simulador si no se han cargado archivos de configuración antes
        if not self.archivos():
            if self.b_beeps == True:
                zumbador.beep_stop()

            # Aseguramos que el botón toggle del simulador no quede presionado
            self.inicio_id_toggle_simulador.set_active(False)
            return True
         
        if (b_simulador == 0):
            if self.b_beeps == True:
                zumbador.beep_primordial()

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", " - Inicia simulador SM-13")
            registrador.info("Starting SM-13 simulator")
            
            self.inicio_etiqueta_estado_simulador.set_text("Enabled")

            if (self.ui_plan_col != 0 and self.ui_plan_row != 0):
                self.f_px_tubo, self.f_py_tubo, self.a_lista_px, self.a_lista_py, b_error_coordenada = leer_archivo_hx(self.ui_plan_col, self.ui_plan_row, self.s_archivo_hx)

                self.corregir_ordenadas_plan_par()

                # Se agrega el jog si lo hubiere
                depurador(1, "HMI", "- Jog aplicado a Px = " + str(round(self.f_incremento_acumulado_jog_col*self.f_x_pitch, 2)) + " in")
                self.f_px_tubo += self.f_incremento_acumulado_jog_col*self.f_x_pitch
                depurador(1, "HMI", "- Jog aplicado a Py = " + str(round(self.f_incremento_acumulado_jog_row*self.f_y_pitch, 2)) + " in")
                self.f_py_tubo += self.f_incremento_acumulado_jog_row*self.f_y_pitch
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
            simu.refrescar_pos_comandada(np.deg2rad(self.f_pole), np.deg2rad(self.f_arm))
            b_simulador = 1
            
            depurador(1, "HMI", " ")
            
            return True
            
        if (b_simulador == 1):
            if self.b_beeps == True:
                zumbador.beep_stop()

            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", " - Se cierra simulador SM-13")
            depurador(1, "HMI", " ")
            registrador.info("Closing SM-13 simulator")

            del simu
            self.inicio_etiqueta_estado_simulador.set_text("Disabled")
            b_simulador = 0
            
            return True

    ##
    # @brief Función que actualiza los valores de ángulos actuales de POLE y ARM
    # @param self Puntero al objeto HMI
    def refrescar_simulador(self):
        global simu
        
        global b_connect
        global b_simulador
        global f_angActArm
        global f_angActPole

        if b_connect == True:
            try:
                # Se actualizan las etiquetas de cuentas y ángulos en las tres solapas
                self.actualizar_etiquetas_enc_ang(f_angActPole, f_angActArm, f_resActPole, f_resActArm)
            except:
                pass

        if (b_simulador):
            try:
                simu.refrescar_pos_actual(np.deg2rad(f_angActPole), np.deg2rad(f_angActArm))
            except:
                pass

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
        if str(self.s_hx_type) == "_cne_purificador_3335_HX2":
            combo_plan.append_text("_cne_purificador_3335_HX2/InspPlan1_M_1")
            combo_plan.append_text("_cne_purificador_3335_HX2/InspPlan2_M_1")
            combo_plan.append_text("_cne_purificador_3335_HX2/InspPlan3_M_2")
            combo_plan.append_text("_cne_purificador_3335_HX2/InspPlan4_M_2")

        
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
        global b_simulador
        global ui_pole_rdc_offset
        global ui_arm_rdc_offset
        global a_HMIDataByte

        if self.b_beeps == True:
            zumbador.beep_primordial()
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Archivo hx        : "+self.s_archivo_hx)
        depurador(1, "HMI", "- Plan de Inspección: "+self.s_archivo_plan)
        depurador(1, "HMI", "- Tipo Fixture      : "+self.s_archivo_fixture)

        registrador.info("System files loaded:")
        registrador.info("HX file        : "+self.s_archivo_hx)
        registrador.info("Inspection Plan: "+self.s_archivo_plan)
        registrador.info("Fixture Model  : "+self.s_archivo_fixture)
        
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
        if (b_simulador == 1):
            del simu
            simu = simulador_SM13(self.s_archivo_fixture, self.ui_montaje, self.s_archivo_hx)
            simu.refrescar_pos_comandada(np.deg2rad(self.f_pole), np.deg2rad(self.f_arm))
            
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
        # Se extraen la variables que contendrán los offset de los ejes, es decir, los valores de cuenta resolver para 0ª.
        trash, ui_pole_rdc_offset, ui_arm_rdc_offset = offsets_lines[1].split(";")
        ui_arm_rdc_offset = abs(int(ui_arm_rdc_offset))
        ui_pole_rdc_offset = abs(int(ui_pole_rdc_offset))

        depurador(1, "HMI", "- Se cargó POLE offset: "+ str(ui_pole_rdc_offset) + ", ARM offset: " + str(ui_arm_rdc_offset))
        depurador(1, "HMI", "- ")

        self.enviar_offsets_a_rtu()

        self.actualizar_etiquetas_msg("System files loaded successfully!", "green")
        
        return True
    
    
    ##
    # @brief Función que implementa la selección de los radioButtons de incremento
    # de Jog.
    # @param self Puntero al objeto HMI
    # @param button Botones radio fino/grueso
    # @return none
    def seleccion_incremento_jog(self, button):
        global a_HMIDataByte
        global a_HMIDataString
        global simu
        global b_simulador

        if button.get_active():
            if (button.get_label() == "off"):
                if self.b_beeps == True:
                    zumbador.beep_stop()

                self.f_incremento_jog = 0.0
                self.actualizar_etiquetas_msg("Jog control disabled...", "green")

                # Restablecemos la posición de la boquilla sin ningún Jog.
                self.f_px_tubo -= self.f_incremento_acumulado_jog_col*self.f_x_pitch
                self.f_py_tubo -= self.f_incremento_acumulado_jog_row*self.f_y_pitch
                
                # Calcula los ángulos de los ejes POLE y ARM en base al corrimiento
                self.f_pole, self.f_arm, self.b_ik_success = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, self.s_name_pivot_type)   
                
                # Se convierten los ágnulos de las articulaciones a grados y se redondea a 3 decimales
                self.f_pole = round(np.rad2deg(self.f_pole), 3)
                self.f_arm = round(np.rad2deg(self.f_arm), 3)
                # A veces pasa que la conversión da números negativos así que se corrige 
                if self.f_pole < 0:
                    self.f_pole = self.f_pole + 360
                if self.f_arm < 0:
                    self.f_arm = self.f_arm + 360

                if not self.b_ik_success:
                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    b_on_condition = False
                    self.actualizar_etiquetas_msg("Coordinate unreacheable...", "red")
                    registrador.error("Inverse kinematics error at px " + str(self.f_px_tubo) + ", py " + str(self.f_py_tubo))

                    return True

                if (b_simulador == 1):
                    # Se refresca simulador ya que se está haciendo un ajuste fino de la boquilla.
                    # no se suma el jog porque ya se lo hace arriba en esta misma función.
                    simu.refrescar_pos_comandada(np.deg2rad(self.f_pole), np.deg2rad(self.f_arm)) 

                # Se convierten a [º] los ángulos POLE y ARM, se redondea a 3 decimales y
                # se actualizan las variables posCmd así las toma el módulo HMIcomRTU y las envía a RTU.    
                a_HMIDataByte[1] = self.f_pole
                a_HMIDataByte[0] = self.f_arm

                # Si todo salió bien, se activa el modo Automático para
                # resetear el jog, pero antes se asegura apagar motor LIFT (si se encontrara activado)
                if a_HMIDataString[0] == "LIFT":
                    a_HMIDataString[0] = "STOP"
                    self.actualizar_etiquetas_msg("Stopping LIFT first, try again...", "red")
                    self.b_fr_boton_lift_up_total.set_active(False)
                    self.b_fr_boton_lift_down_total.set_active(False)
                    self.b_manual_boton_lift_up_total.set_active(False)
                    self.b_manual_boton_lift_down_total.set_active(False)
                    self.b_inspection_boton_lift_up_total.set_active(False)
                    self.b_inspection_boton_lift_down_total.set_active(False)
                else:
                    a_HMIDataString[0] = "AUTOMATIC"
                
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
            depurador(1, "HMI", "- ")     
        
        return True
    
    ##
    # @brief Función que implementa la sección Jog Control
    # @param self Puntero al objeto HMI
    # @param button Boton Pueden ser cuatro: jog_to_north, jog_to_east, jog_to_south, jog_to_west
    # @return none
    def jog_control(self, button):
        
        global a_HMIDataByte
        global a_HMIDataString
        global simu
        global b_simulador

        # Verificaciones previas al movimiento
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():

            if self.b_beeps == True:
                zumbador.beep_stop()

            return True
        
        # Antes de mover verifica si está activado el control principal
        if (a_HMIDataString[3] == "DISABLE_CONTROL"):
            if self.b_beeps == True:
                zumbador.beep_stop()

            b_on_condition = False
            self.actualizar_etiquetas_msg("Fixture control is disabled...", "red")

            return True
        
        # Antes de mover verifica si no está atascado el telemanipulador
        if (b_stallAlm == True and a_HMIDataString[4] == "STALL_ENABLE"):
            if self.b_beeps == True:
                zumbador.beep_stop()

            # En caso de atasque, se desactiva el control principal, se despinta el botón y se advierte
            a_HMIDataString[3] = "DISABLE_CONTROL"
            self.inicio_toggle_fixture_control.set_active(False)
            self.inicio_etiqueta_estado_main_control.set_text("Disabled")
            # Se indica en todas las etiquetas de msg del hmi
            b_on_condition = False
            self.actualizar_etiquetas_msg("Fixture harness is stalled, main control disabled...", "red")
            registrador.warning("Stall detect while jogging")

            return True
        
        # Si no hay jog acumulado no hay nada que hacer aquí.
        if (self.f_incremento_jog == 0.0):
            b_on_condition = False
            self.actualizar_etiquetas_msg("Select an increment to jog the end effector...", "red")
            return True

        if self.b_beeps == True:
            zumbador.beep_button()
            
            
        # Obtiene el nombre del boton jog presionado
        s_boton_jog = button.get_name()
        
        if (s_boton_jog == "jog_to_east"):
            self.f_px_tubo += self.f_incremento_jog*self.f_x_pitch
            self.f_incremento_acumulado_jog_col += self.f_incremento_jog
            
        elif (s_boton_jog == "jog_to_west"):
            self.f_px_tubo -= self.f_incremento_jog*self.f_x_pitch
            self.f_incremento_acumulado_jog_col -= self.f_incremento_jog
            
        if (s_boton_jog == "jog_to_north"):
            self.f_py_tubo += self.f_incremento_jog*self.f_y_pitch
            self.f_incremento_acumulado_jog_row += self.f_incremento_jog
        
        elif (s_boton_jog == "jog_to_south"):
            self.f_py_tubo -= self.f_incremento_jog*self.f_y_pitch
            self.f_incremento_acumulado_jog_row -= self.f_incremento_jog
            
        try:
            # Se actualizan etiquetas en solapas Manual e Inspection
            self.manual_etiqueta_valor_jog_row.set_text(str(round(self.f_incremento_acumulado_jog_row, 2)))
            self.manual_etiqueta_valor_jog_col.set_text(str(round(self.f_incremento_acumulado_jog_col, 2)))
            self.inspection_etiqueta_valor_jog_row.set_text(str(round(self.f_incremento_acumulado_jog_row, 2)))
            self.inspection_etiqueta_valor_jog_col.set_text(str(round(self.f_incremento_acumulado_jog_col, 2)))
            registrador.info("Jogging the end effector position")
            registrador.info("Jog col total: " + str(round(self.f_incremento_acumulado_jog_col, 2)))
            registrador.info("Jog row total: " + str(round(self.f_incremento_acumulado_jog_row, 2)))
        except:
            pass
        
        self.actualizar_etiquetas_msg("Jogging the end effector position...", "green")
            
        # Calcula los ángulos de los ejes POLE y ARM en base al corrimiento
        self.f_pole, self.f_arm, self.b_ik_success = ik_SM13(self.f_px_tubo, self.f_py_tubo, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, self.s_name_pivot_type)   

        # Se convierten los ágnulos de las articulaciones a grados y se redondea a 3 decimales
        self.f_pole = round(np.rad2deg(self.f_pole), 3)
        self.f_arm = round(np.rad2deg(self.f_arm), 3)

        if not self.b_ik_success:
            if self.b_beeps == True:
                zumbador.beep_stop()

            b_on_condition = False
            self.actualizar_etiquetas_msg("Can not move positioner in that direction...", "red")
            registrador.error("Inverse kinematics error at px " + str(self.f_px_tubo) + ", py " + str(self.f_py_tubo))

            return True

        if (b_simulador == 1):
            # Se refresca simulador ya que se está haciendo un ajuste fino de la boquilla.
            # no se suma el jog porque ya se lo hace arriba en esta misma función.
            simu.refrescar_pos_comandada(np.deg2rad(self.f_pole), np.deg2rad(self.f_arm))
        
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Ajustando fino con incremento jog: "+str(self.f_incremento_jog))
        depurador(1, "HMI", "- Jog acumulado en COL: "+str(self.f_incremento_acumulado_jog_col))
        depurador(1, "HMI", "- Jog acumulado en ROW: "+str(self.f_incremento_acumulado_jog_row))
        depurador(1, "HMI", "- Se calcularon ángulos POLE = "+str(self.f_pole)+"°"+", ARM = "+str(self.f_arm)+"°")
        depurador(1, "HMI", "- Para ubicar boquilla en Px = "+str(round(self.f_px_tubo, 3))+" in"+", Py = "+str(round(self.f_py_tubo, 3))+" in")
        depurador(1, "HMI", " ") 
        

        # se actualizan las variables posCmd con el JOG aplicado.    
        a_HMIDataByte[1] = self.f_pole
        a_HMIDataByte[0] = self.f_arm

        # Si todo salió bien, se activa el modo Automático para
        # realizar el jog, pero antes se asegura apagar motor LIFT (si se encontrara activado)
        if a_HMIDataString[0] == "LIFT":
            a_HMIDataString[0] = "STOP"
            self.actualizar_etiquetas_msg("Stopping LIFT first, try again...", "red")
            self.b_fr_boton_lift_up_total.set_active(False)
            self.b_fr_boton_lift_down_total.set_active(False)
            self.b_manual_boton_lift_up_total.set_active(False)
            self.b_manual_boton_lift_down_total.set_active(False)
            self.b_inspection_boton_lift_up_total.set_active(False)
            self.b_inspection_boton_lift_down_total.set_active(False)

            return True

        a_HMIDataString[0] = "AUTOMATIC"
        
        return True
    
    ##
    # @brief Función que almacena el offset que hay entre cuentas de 
    # resolver POLE y ARM cuando el robot se encuentra plegado.
    # @param self Puntero al objeto HMI
    # @param button Boton Set Offset
    # @return none
    def cal_home_offset(self, button):
        
        global a_HMIDataByte
        global ui_pole_rdc_offset
        global ui_arm_rdc_offset
        global f_resActArm
        global f_resActPole


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

        # Condicionales para depuración de calibraciones
        if self.s_name_pivot_type == "main":
            # ejemplo corrido en campo
            # ejemplo campo tubo ]1, 1] M2
            #a_RTUDataRx[1] = 48000
            #a_RTUDataRx[0] = 63000
            # ejemplo campo tubo ]55, 1] M2
            #a_RTUDataRx[1] = 22657
            #a_RTUDataRx[0] = 55801
            #ejemplo campo tubo ]20, 9] M2
            #a_RTUDataRx[1] = 1635
            #a_RTUDataRx[0] = 45522
            pass

        if self.s_name_pivot_type == "alternative":
            # ejemplo corrido en campo
            # ejemplo campo tubo ]1, 1] M2
            #a_RTUDataRx[1] = 31851
            #a_RTUDataRx[0] = 20949
            # ejemplo campo tubo ]55, 1] M2
            #a_RTUDataRx[1] = 64477
            #a_RTUDataRx[0] = 26263
            pass

        # Se calculan los offset (trasladados a home) en base a la diferencia, en cuentas de resolver, de la posición real del tubo tomado 
        # como referencia y el valor de cuenta resultante del cálculo matermático del mismo tubo.
        ui_pole_rdc_offset = int(f_resActPole - self.ui_pole_res_for_cal) 
        ui_arm_rdc_offset = int(f_resActArm - self.ui_arm_res_for_cal) 

        # Se evitan valores negativos de offset
        if ui_pole_rdc_offset < 0:
            ui_pole_rdc_offset = ui_pole_rdc_offset + self.ui_MAX_CUENTAS
        if ui_arm_rdc_offset < 0:
            ui_arm_rdc_offset = ui_arm_rdc_offset + self.ui_MAX_CUENTAS

        # Se evitan valores de offset por encima del máximo de cuentas posibles 
        if ui_pole_rdc_offset > self.ui_MAX_CUENTAS:
            ui_pole_rdc_offset = ui_pole_rdc_offset - self.ui_MAX_CUENTAS
        if ui_arm_rdc_offset > self.ui_MAX_CUENTAS:
            ui_arm_rdc_offset = ui_arm_rdc_offset - self.ui_MAX_CUENTAS

        depurador(1, "HMI", "- Pivot type: " + self.s_name_pivot_type)
        depurador(1, "HMI", "- Valor resolver teórico: POLE "+ str(self.ui_pole_res_for_cal) + ", ARM " + str(self.ui_arm_res_for_cal))
        depurador(1, "HMI", "- Valor resolver actual:  POLE "+ str(f_resActPole) + ", ARM " + str(f_resActArm))
        depurador(1, "HMI", " ")
        registrador.info("Pivot type: " + self.s_name_pivot_type)
        registrador.info("Resolver values calculation: POLE "+ str(self.ui_pole_res_for_cal) + ", ARM " + str(self.ui_arm_res_for_cal))
        registrador.info("Current resolver values    : POLE "+ str(f_resActPole) + ", ARM " + str(f_resActArm))
        registrador.info("Offsets calculation    : POLE "+ str(ui_pole_rdc_offset) + ", ARM " + str(ui_arm_rdc_offset))

        # Se comprueba que el cálculo de offset de un valor coherente
        if (ui_pole_rdc_offset > self.ui_MAX_CUENTAS or ui_arm_rdc_offset > self.ui_MAX_CUENTAS) or (ui_pole_rdc_offset < 0 or ui_arm_rdc_offset < 0):
            # Se limpia la bandera de on_condition
            b_on_condition = False
            depurador(1, "HMI", "- Error en el cálculo de offsets, se mantiene valor anterior") 
            depurador(1, "HMI", "-  ") 
            self.actualizar_etiquetas_msg("calibration point error...", "red")
            registrador.error("Calibration point error")


            return True
        
        # Se comprueban cambios.
        if (ui_pole_rdc_offset == int(s_x_pole_offset) and ui_arm_rdc_offset == int(s_x_arm_offset)):
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
            new_offsets_line = str(s_robot) + ";" + str(ui_pole_rdc_offset) + ";" + str(ui_arm_rdc_offset)
            new_offsets_file.write("ROBOT;POLE HOME OFFSET;ARM HOME OFFSET\n")
            new_offsets_file.write(new_offsets_line)
            new_offsets_file.close()
            

            # Se abre el archivo robots_home_offsets.csv en modo lectura, se lo lee
            # para asegurarse que se implementó el cambio de offsets y se lo cierra. 
            new_offsets_file = open(self.s_project_path + "/hmi/cfg_files/Robots/robots_home_offsets.csv", "r")
            new_offsets_lines = new_offsets_file.readlines()
            new_offsets_file.close()
            
            # Se actualizan la variables principales que contienen los valores de offset 
            trash, ui_pole_rdc_offset, ui_arm_rdc_offset = new_offsets_lines[1].split(";")

            ui_pole_rdc_offset = int(ui_pole_rdc_offset)
            ui_arm_rdc_offset = int(ui_arm_rdc_offset)
            
            # Se envía el comando de "CAL_SET", para establecer la calibración del punto seleccionados en la RTU
            a_HMIDataString[6] = "CAL_SET"
            
            depurador(1, "HMI", "- POLE offset anterior: "+ str(s_x_pole_offset) + ", ARM offset anterior: " + str(s_x_arm_offset))
            depurador(1, "HMI", "- POLE offset nuevo   : "+ str(ui_pole_rdc_offset) + ", ARM offset nuevo   : " + str(ui_arm_rdc_offset))
            registrador.info("POLE offset previous: "+ str(s_x_pole_offset) + ", ARM offset previous: " + str(s_x_arm_offset))
            registrador.info("POLE offset now     : "+ str(ui_pole_rdc_offset) + ", ARM offset now     : " + str(ui_arm_rdc_offset))

            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.enviar_offsets_a_rtu()

        return True

    ##
    # @brief Función que coloca los offsets en la trama para actualizarlos en la rtu
    # @param self puntero al objeto HMI
    # @return none
    def enviar_offsets_a_rtu(self):
        global a_HMIDataByte
        global a_HMIDataString
        global ui_pole_rdc_offset
        global ui_arm_rdc_offset

        # Se actualiza byte de Tx con el ángulo de offset correspondiente, luego el conversor lo pasará a cuentas
        # sin corregir con offset 
        a_HMIDataByte[1] = float((ui_pole_rdc_offset/self.ui_MAX_CUENTAS)*self.f_MAX_GRADOS)
        a_HMIDataByte[0] = float((ui_arm_rdc_offset/self.ui_MAX_CUENTAS)*self.f_MAX_GRADOS)
        # Se envía el comando de "CAL_SET" para establecer la calibración del punto seleccionados en la RTU
        a_HMIDataString[6] = "CAL_SET"
        if self.ui_plan_col != 0 or self.ui_plan_row != 0:
            # para que no imprimia set cal cuando no se han ingresado archivos de cfg
            self.actualizar_etiquetas_msg("Calibration point success at COL: "+ str(self.ui_plan_col) + ", ROW: "+ str(self.ui_plan_row), "green")
            registrador.info("Calibration point success at COL: "+ str(self.ui_plan_col) + ", ROW: "+ str(self.ui_plan_row))
    
    ##
    # @brief Función que indica si se han cargado archivos de configuración o no
    # @param self puntero al objeto HMI
    # @return True o False si se han cargado o no los tres archivos de configuración
    def archivos(self):

        if (self.s_archivo_hx == 'none' or self.s_archivo_fixture == 'none' or self.s_archivo_plan == 'none'):
            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("No configuration files has been loaded...", "red")

            if self.b_beeps == True:
                zumbador.beep_stop()

            return False
        else:
            return True
    
    ##
    # @brief Función que optimiza la actualización de las etiquetas de temperatura
    # simultáneamente en todas las solapas
    # @param self puntero al objeto HMI
    # @param  ui_T valor de temperatura que se desea mostrar 
    # @return none
    def actualizar_etiquetas_temperatura(self, ui_T):
        if(f_temperatura < 50):
            self.manual_etiqueta_valor_temperatura.set_markup("<span foreground='green'> " + str(ui_T) + " </span>")
            self.inspection_etiqueta_valor_temperatura.set_markup("<span foreground='green'> " + str(ui_T) + " </span>")
        else:
            self.inspection_etiqueta_valor_temperatura.set_markup("<span foreground='orange'> " + str(ui_T) + " </span>")
            self.inspection_etiqueta_valor_temperatura.set_markup("<span foreground='orange'> " + str(ui_T) + " </span>")

            return True

    ##
    # @brief Función que optimiza la actualización de mensajes generales
    # simultáneamente en todas las etiquetas de msg de las diferentes solapas
    # @param self puntero al objeto HMI
    # @param s_msg Mensaje general a imprimir en todas las etiquetas de las solapas
    # @param s_color Color del mensaje, verde mensajes de información, rojo mensajes de alerta
    # @return none
    def actualizar_etiquetas_msg(self, s_msg, s_color): 

        if s_color == "red":
            self.inicio_etiqueta_msg.set_markup("<span foreground='red'> " + s_msg + " </span>")
            self.fr_etiqueta_msg.set_markup("<span foreground='red'> " + s_msg + " </span>")
            self.manual_etiqueta_msg.set_markup("<span foreground='red'> " + s_msg + " </span>")
            self.inspection_etiqueta_msg.set_markup("<span foreground='red'> " + s_msg + " </span>")
        if s_color == "green":
            self.inicio_etiqueta_msg.set_markup("<span foreground='green'> " + s_msg + " </span>")
            self.fr_etiqueta_msg.set_markup("<span foreground='green'> " + s_msg + " </span>")
            self.manual_etiqueta_msg.set_markup("<span foreground='green'> " + s_msg + " </span>")
            self.inspection_etiqueta_msg.set_markup("<span foreground='green'> " + s_msg + " </span>")
          
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
    # @param f_ang_act_pole ángulo de la articulación POLE en grados
    # @param f_ang_act_arm ángulo de la articulación ARM en grados
    # @param ui_res_act_pole valor de cuenta actual del resolver POLE (se tiene en cuenta offset)
    # @param ui_res_act_arm valor de cuenta actual del resolver ARM (se tiene en cuenta offset)
    # @return none

    def actualizar_etiquetas_enc_ang(self, f_ang_act_pole, f_ang_act_arm, ui_res_act_pole, ui_res_act_arm):
                
        self.fr_etiqueta_valor_ang_arm.set_text(str(round(f_ang_act_arm, 3)))
        self.manual_etiqueta_valor_ang_arm.set_text(str(round(f_ang_act_arm, 3)))
        self.inspection_etiqueta_valor_ang_arm.set_text(str(round(f_ang_act_arm, 3)))
                    
        self.fr_etiqueta_valor_enc_arm.set_text(str(round(ui_res_act_arm, 0)))
        self.manual_etiqueta_valor_enc_arm.set_text(str(round(ui_res_act_arm, 0)))
        self.inspection_etiqueta_valor_enc_arm.set_text(str(round(ui_res_act_arm, 0)))
                    
        self.fr_etiqueta_valor_ang_pole.set_text(str(round(f_ang_act_pole, 3)))
        self.manual_etiqueta_valor_ang_pole.set_text(str(round(f_ang_act_pole, 3)))
        self.inspection_etiqueta_valor_ang_pole.set_text(str(round(f_ang_act_pole, 3)))
                    
        self.fr_etiqueta_valor_enc_pole.set_text(str(round(ui_res_act_pole, 0)))
        self.manual_etiqueta_valor_enc_pole.set_text(str(round(ui_res_act_pole, 0)))
        self.inspection_etiqueta_valor_enc_pole.set_text(str(round(ui_res_act_pole, 0)))
        
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
    # Las variables s_ip y s_puerto se hacen globales para que sean visibles tanto por el 
    # thread del HMI como por el del TM.
    # @param self Puntero al objeto HMI
    # @param button Botón "modificar"
    # @return none
    def modificar_red(self, button):
        global s_port
        global s_ip

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
            s_ip, s_port = new_address_lines[1].split(";")
        
            depurador(1, "HMI", "- IP anterior: "+ s_xip + ", Puerto anterior: " + s_xpuerto)
            depurador(1, "HMI", "- IP nueva   : "+ s_ip + ", Puerto nuevo: " + s_port)
            registrador.info("Changing RTU IP/PORT parameters:")
            registrador.info("Previous IP: "+ s_xip + ", Previous Port: " + s_xpuerto)
            registrador.info("Current  IP: "+ s_ip  + ", Current Port : " + s_port)

            
        return True
    
    ##
    # @brief Función que implementa el comando del eje LIFT
    # @param self Puntero al objeto HMI
    # @param button Botones Lift: "UP/DOWN/UP-TOT/DOWN-TOT"
    # @return none
    def control_lift(self, button):
        
        global a_HMIDataString
        global b_limitUp
        global b_limitDown

        # Retardo para que RTU vea tramas de STOP del botòn toggle opuesto
        ui_DELAY_ms_TOGGLE = 100

        # Antes de mover verifica si está activado el control principal
        if (a_HMIDataString[3] == "DISABLE_CONTROL"):

            if self.b_beeps == True:
                zumbador.beep_stop()

            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("Fixture control is disabled...", "red")

            # Si no está habilitado el control principal aseguramos que no queden 
            # presionados los botones toggle de las diferentes solapas
            self.b_fr_boton_lift_up_total.set_active(False)
            self.b_fr_boton_lift_down_total.set_active(False)
            self.b_manual_boton_lift_up_total.set_active(False)
            self.b_manual_boton_lift_down_total.set_active(False)
            self.b_inspection_boton_lift_up_total.set_active(False)
            self.b_inspection_boton_lift_down_total.set_active(False)

            return True   

        # Si el SM-13 está moviendo ARM y/o POLE no se permite el movimiento LIFT.
        if (a_HMIDataString[0] == "FREE_RUN" or a_HMIDataString[0] == "AUTOMATIC"):
            
            # Se desactiva botón para que no se muestre presionado
            self.b_fr_boton_lift_up_total.set_active(False)
            self.b_fr_boton_lift_down_total.set_active(False)
            self.b_manual_boton_lift_up_total.set_active(False)
            self.b_manual_boton_lift_down_total.set_active(False)
            self.b_inspection_boton_lift_up_total.set_active(False)
            self.b_inspection_boton_lift_down_total.set_active(False)

            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("Wait for ARM or POLE stop movement!", "red")

            if self.b_beeps == True:
                zumbador.beep_stop()

            return True     
        
        # Si se pasan todas las precondiciones, se procede con el movimiento
        if self.b_beeps == True:
            zumbador.beep_button()

        # Obtiene el nombre del boton jog presionado
        s_boton_lift = button.get_name()
        
        # botones tipo pulsador afectados por temporizador
        if (s_boton_lift == "lift_up"):
            if(b_limitUp == True):
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[5] = "LIFT_UP"
                self.inspection_etiqueta_msg.set_text("Lift upper limit alarm!")
                depurador(3, "HMI", "****************************************")
                depurador(3, "HMI", "- Límite superior alcanzado en LIFT")
                depurador(3, "HMI", " ")
                return
            else:
                # Se desactiva cualquier toggle que esté presionado
                # Se desactiva botón para que no se muestre presionado
                self.b_fr_boton_lift_up_total.set_active(False)
                self.b_fr_boton_lift_down_total.set_active(False)
                self.b_manual_boton_lift_up_total.set_active(False)
                self.b_manual_boton_lift_down_total.set_active(False)
                self.b_inspection_boton_lift_up_total.set_active(False)
                self.b_inspection_boton_lift_down_total.set_active(False)

                # si ZS no acusa alarma desde RTU se mueve lift
                a_HMIDataString[0] = "LIFT"
                a_HMIDataString[5] = "LIFT_UP"
                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos, "LIFT_UP")
                
        elif (s_boton_lift == "lift_down"):
            if(self.b_limitDwn == True):
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[5] = "LIFT_DOWN"
                self.inspection_etiqueta_msg.set_text("Lift lower limit alarm!")
                depurador(3, "HMI", "****************************************")
                depurador(3, "HMI", "- Límite inferior alcanzado en LIFT")
                depurador(3, "HMI", " ")
                return
            else:
                # Se desactiva cualquier toggle que esté presionado
                # Se desactiva botón para que no se muestre presionado
                self.b_fr_boton_lift_up_total.set_active(False)
                self.b_fr_boton_lift_down_total.set_active(False)
                self.b_manual_boton_lift_up_total.set_active(False)
                self.b_manual_boton_lift_down_total.set_active(False)
                self.b_inspection_boton_lift_up_total.set_active(False)
                self.b_inspection_boton_lift_down_total.set_active(False)
                # si ZS no acusa alarma desde RTU se mueve lift
                a_HMIDataString[0] = "LIFT"
                a_HMIDataString[5] = "LIFT_DOWN"
                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos, "LIFT_DOWN")
            
        # botones tipo toggle no afectados por temporizador            
        if (s_boton_lift == "lift_up_total"):
            # se verifica si el botón esta presionado o suelto
            b_boton_lift_up_tot = button.get_active()
            if (b_boton_lift_up_tot == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[5] = "LIFT_DOWN"
                #Damos tiempo para que se envíe al menos una trama y tome el comando la RTU
                time.sleep(ui_DELAY_ms_TOGGLE/1000)

                self.b_fr_boton_lift_down_total.set_active(False)
                self.b_manual_boton_lift_down_total.set_active(False)
                self.b_inspection_boton_lift_down_total.set_active(False)
                if(b_limitUp == True):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[5] = "LIFT_UP"
                    self.inspection_etiqueta_msg.set_text("Lift upper limit alarm!")
                    depurador(3, "HMI", "****************************************")
                    depurador(3, "HMI", "- Límite superior alcanzado en LIFT")
                    depurador(3, "HMI", " ")
                    return
                else:
                    # si ZS no acusa alarma desde RTU se mueve lift
                    a_HMIDataString[0] = "LIFT"
                    a_HMIDataString[5] = "LIFT_UP"
            if (b_boton_lift_up_tot == False):
                self.detener_movimientos("LIFT_UP")
                return
                
        elif (s_boton_lift == "lift_down_total"):
            # se verifica si el botón esta presionado o suelto
            b_boton_lift_down_tot = button.get_active()
            if (b_boton_lift_down_tot == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[5] = "LIFT_UP"
                #Damos tiempo para que se envíe al menos una trama y tome el comando la RTU
                time.sleep(ui_DELAY_ms_TOGGLE/1000)

                self.b_fr_boton_lift_up_total.set_active(False)
                self.b_manual_boton_lift_up_total.set_active(False)
                self.b_inspection_boton_lift_up_total.set_active(False)
                if(self.b_limitDwn == True):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[5] = "LIFT_DOWN"
                    self.actualizar_etiquetas_msg("Lift lower limit alarm!", "red")
                    depurador(3, "HMI", "****************************************")
                    depurador(3, "HMI", "- Límite inferior alcanzado en LIFT")
                    depurador(3, "HMI", " ")
                    return
                else:
                    # si ZS no acusa alarma desde RTU se mueve lift
                    a_HMIDataString[0] = "LIFT"
                    a_HMIDataString[5] = "LIFT_DOWN"
            if (b_boton_lift_down_tot == False):
                self.detener_movimientos("LIFT_DOWN")
                return
            
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Moving "+ a_HMIDataString[5])
        depurador(1, "HMI", " ")
        
        b_on_condition = False
        self.actualizar_etiquetas_msg("Moving "+ a_HMIDataString[5] + "...", "green")
        registrador.info("Moving "+ a_HMIDataString[5])
        
    
        return 
    
    ##
    # @brief Función que coloca la variable de direccion del eje LIFT en STOP.
    # también sirve para detener el temporizado de los pulsadores asociados
    # al eje LIFT mediante el retorno de un False.
    # @param self Puntero al objeto HMI
    # @param s_FR_Axis eje/s que se desee/n detener
    # @return False y detiene el GLib.timeout_add() que creo la tarea.
    def detener_movimientos(self, s_FR_Axis):
        global a_HMIDataString

        if s_FR_Axis == "POLE":
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[1] = "POLE"
        if s_FR_Axis == "ARM":
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[1] = "ARM"
        if s_FR_Axis == "BOTH":
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[1] = "POLE"
            # Se esperan suficientes ms para dar tiempo a que la función de telemetría envíe el comando anterior
            time.sleep(0.1)
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[1] = "ARM"
        if s_FR_Axis == "LIFT_UP":
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[5] = "LIFT_UP"
        if s_FR_Axis == "LIFT_DOWN":
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[5] = "LIFT_DOWN"
        if s_FR_Axis == "TOTAL":
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[1] = "POLE"
            # Se esperan suficientes ms para dar tiempo a que la función de telemetría envíe el comando anterior
            time.sleep(0.1)
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[1] = "ARM"
            # Se esperan suficientes ms para dar tiempo a que la función de telemetría envíe el comando anterior
            time.sleep(0.1)
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[5] = "LIFT_UP"
            # Se esperan suficientes ms para dar tiempo a que la función de telemetría envíe el comando anterior
            time.sleep(0.1)
            a_HMIDataString[0] = "STOP"
            a_HMIDataString[5] = "LIFT_DOWN"

            a_HMIDataString[3] = "DISABLE_CONTROL"

            # Cambia a OFF el widget del interruptor principal
            self.inicio_toggle_fixture_control.set_active(False)
            # Cambia el mensaje de la etiqueta del interruptor
            self.inicio_etiqueta_estado_main_control.set_text("Disabled") 

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
        
        depurador(4, "HMI", "****************************************")
        depurador(4, "HMI", "- Modo: " + a_HMIDataString[0])
        depurador(4, "HMI", " ")
        
        # Se limpia la bandera de on_condition
        b_on_condition = False
        self.actualizar_etiquetas_msg("Stopping movement..." + s_FR_Axis, "red")
        registrador.info("Stopping movement " + s_FR_Axis)
    
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
        
        global a_HMIDataByte
        global a_HMIDataString
        global b_on_condition
        global b_cwLimitArm
        global b_cwLimitPole
        global b_ccwLimitArm
        global b_ccwLimitPole
     
        
        

        ui_DELAY_ms_TOGGLE = 100

        # Antes de mover verifica si está activado el control principal
        if (a_HMIDataString[3] == "DISABLE_CONTROL"):
            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("Fixture control is disabled...", "red")

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
        a_HMIDataByte[2] = int(self.ui_fr_escala_vel_arm.get_value())
        a_HMIDataByte[3] = int(self.ui_fr_escala_vel_pole.get_value())        
        
        # Obtiene el nombre del boton jog presionado
        s_boton_fr = button.get_name()

        # botones tipo pulsador afectados por temporizador
        if (s_boton_fr == "fr_push_arm_cw"):
            # Si la velocidad seteada es 0 no hay nada que hacer aquí
            if(a_HMIDataByte[2] == 0):
                a_HMIDataString[0] = "STOP"   
                a_HMIDataString[1] = "ARM"     
                # Se limpia la bandera de on_condition
                b_on_condition = False
                self.actualizar_etiquetas_msg("The set speed is 0 for this movement...", "red")

                if self.b_beeps == True:
                    zumbador.beep_stop()

                return
                
            # Si hay un límite de movimiento por software se cancela el movimiento
            elif(b_cwLimitArm == True):
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "ARM"
                self.fr_etiqueta_msg.set_text("ARM CW limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite horario alcanzado en ARM")
                depurador(1, "HMI", " ")

                if self.b_beeps == True:
                    zumbador.beep_alarm()

                return
    
            else:
                # si no hay alarmas de software desde RTU se mueve ARM CW
                a_HMIDataString[0] = "FREE_RUN"
                a_HMIDataString[1] = "ARM"
                a_HMIDataString[2] = "DIR_CW"

                if self.b_beeps == True:
                    zumbador.beep_button()

                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos, "ARM")
                
        elif (s_boton_fr == "fr_push_arm_ccw"):
            
            # Si la velocidad seteada es 0 no hay nada que hacer aquí
            if(a_HMIDataByte[2] == 0):
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "ARM"
                # Se limpia la bandera de on_condition
                b_on_condition = False
                self.actualizar_etiquetas_msg("The set speed is 0 for this movement...", "red")

                if self.b_beeps == True:
                    zumbador.beep_stop()

                return
                
            elif (b_ccwLimitArm == True):
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "ARM"
                self.fr_etiqueta_msg.set_text("ARM CCW limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite anti-horario alcanzado en ARM")
                depurador(1, "HMI", " ")

                if self.b_beeps == True:
                    zumbador.beep_alarm()

                return
            
            else:
                # si no hay alarmas de software desde RTU se mueve lift
                a_HMIDataString[0] = "FREE_RUN"
                a_HMIDataString[1] = "ARM"
                a_HMIDataString[2] = "CCW_DIR"

                if self.b_beeps == True:
                    zumbador.beep_button()

                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos, "ARM")
                
        if (s_boton_fr == "fr_push_pole_cw"):
            # Si la velocidad seteada es 0 no hay nada que hacer aquí
            if(a_HMIDataByte[3] == 0):
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "POLE"

                if self.b_beeps == True:
                    zumbador.beep_stop()

                # Se limpia la bandera de on_condition
                b_on_condition = False
                self.actualizar_etiquetas_msg("The set speed is 0 for this movement...", "red")
                return
            
            if(b_cwLimitPole == True):
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "POLE"
                self.fr_etiqueta_msg.set_text("POLE CW limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite horario alcanzado en POLE")
                depurador(1, "HMI", " ")

                if self.b_beeps == True:
                    zumbador.beep_alarm()

                return
    
            else:
                # si no hay alarmas de software desde RTU se mueve ARM CW
                a_HMIDataString[0] = "FREE_RUN"
                a_HMIDataString[1] = "POLE"
                a_HMIDataString[2] = "DIR_CW"

                if self.b_beeps == True:
                    zumbador.beep_button()

                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos, "POLE")
                
        elif (s_boton_fr == "fr_push_pole_ccw"):
            # Si la velocidad seteada es 0 no hay nada que hacer aquí
            if(a_HMIDataByte[3] == 0):
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "POLE"
                # Se limpia la bandera de on_condition
                b_on_condition = False
                self.actualizar_etiquetas_msg("The set speed is 0 for this movement...", "red")

                if self.b_beeps == True:
                    zumbador.beep_stop()

                return
            
            elif (b_ccwLimitPole == True):
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "POLE"
                self.fr_etiqueta_msg.set_text("POLE CCW limit alarm!")
                depurador(1, "HMI", "****************************************")
                depurador(1, "HMI", "- Límite anti-horario alcanzado en POLE")
                depurador(1, "HMI", " ")

                if self.b_beeps == True:
                    zumbador.beep_alarm()

                return
            else:
                # si no hay alarmas de software desde RTU se mueve POLE
                a_HMIDataString[0] = "FREE_RUN"
                a_HMIDataString[1] = "POLE"
                a_HMIDataString[2] = "CCW_DIR"

                if self.b_beeps == True:
                    zumbador.beep_button()

                GLib.timeout_add(self.ui_DELAY_ms_PULSADOR, self.detener_movimientos, "POLE")
            
        # botones tipo toggle NO afectados por temporizador            
        if (s_boton_fr == "fr_toggle_arm_cw"):
            # se verifica si el botón esta presionado o suelto
            b_toggle_arm_cw = button.get_active()

            if (b_toggle_arm_cw == False):
                self.detener_movimientos("ARM")
                return True

            elif (b_toggle_arm_cw == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "ARM"
                # se asegura que se envíe al menos una trama antes de enviar el siguiente STOP sino la RTU no se va a enterar
                time.sleep(ui_DELAY_ms_TOGGLE/1000) 

                self.b_boton_toggle_arm_ccw.set_active(False)

                # Si la velocidad seteada es 0 no hay nada que hacer aquí
                if(a_HMIDataByte[2] == 0):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[1] = "ARM"
                    # Si no hay velocidad seleccionada aseguramos que no quede presionado el toggle
                    self.b_boton_toggle_arm_cw.set_active(False)

                    # Se limpia la bandera de on_condition
                    b_on_condition = False
                    self.actualizar_etiquetas_msg("The set speed is 0 for this movement...", "red")

                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    return True

                if(b_cwLimitArm == True):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[1] = "ARM"
                    self.fr_etiqueta_msg.set_text("ARM CW limit alarm!")

                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite horario alcanzado en ARM")
                    depurador(1, "HMI", " ")

                    if self.b_beeps == True:
                        zumbador.beep_alarm()

                    # Se desactiva botón toggle para que no quede coloreado
                    self.b_boton_toggle_arm_cw.set_active(False)

                    return True

                else:
                    # si no hay alarmas de límite por software desde RTU se mueve ARM CW
                    a_HMIDataString[0] = "FREE_RUN"
                    a_HMIDataString[1] = "ARM"
                    a_HMIDataString[2] = "DIR_CW"

                    if self.b_beeps == True:
                        zumbador.beep_button()
                
        elif (s_boton_fr == "fr_toggle_arm_ccw"):
            # se verifica si el botón esta presionado o suelto
            b_toggle_arm_ccw = button.get_active()

            if (b_toggle_arm_ccw == False):
                self.detener_movimientos("ARM")
                return True

            elif (b_toggle_arm_ccw == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "ARM"
                # se asegura que se envíe al menos una trama antes de enviar el siguiente STOP sino la RTU no se va a enterar
                time.sleep(ui_DELAY_ms_TOGGLE/1000)

                self.b_boton_toggle_arm_cw.set_active(False)

                # Si la velocidad seteada es 0 no hay nada que hacer aquí
                if(a_HMIDataByte[2] == 0):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[1] = "ARM"

                    # Si no hay velocidad seleccionada aseguramos que no quede presionado el toggle
                    self.b_boton_toggle_arm_ccw.set_active(False)

                    # Se limpia la bandera de on_condition
                    b_on_condition = False
                    self.actualizar_etiquetas_msg("The set speed is 0 for this movement...", "red")

                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    return True
                
                elif(b_ccwLimitArm == True):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[1] = "ARM"
                    self.fr_etiqueta_msg.set_text("ARM CCW limit alarm!")
                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite anti-horario alcanzado en ARM")
                    depurador(1, "HMI", " ")

                    if self.b_beeps == True:
                        zumbador.beep_alarm()

                    # Se desactiva botón toggle para que no quede coloreado
                    self.b_boton_toggle_arm_ccw.set_active(False)

                    return True
                else:
                    # si no hay alarmas de límite por software desde RTU se mueve ARM CCW
                    a_HMIDataString[0] = "FREE_RUN"
                    a_HMIDataString[1] = "ARM"
                    a_HMIDataString[2] = "CCW_DIR"

                    if self.b_beeps == True:
                        zumbador.beep_button()
            
        if (s_boton_fr == "fr_toggle_pole_cw"):
            # se verifica si el botón esta presionado o suelto
            b_toggle_pole_cw = button.get_active()

            if (b_toggle_pole_cw == False):
                self.detener_movimientos("POLE")
                return

            elif (b_toggle_pole_cw == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "POLE"
                # se asegura que se envíe al menos una trama antes de enviar el siguiente STOP sino la RTU no se va a enterar
                time.sleep(ui_DELAY_ms_TOGGLE/1000)

                self.b_boton_toggle_pole_ccw.set_active(False)
                # Si la velocidad seteada es 0 no hay nada que hacer aquí
                if(a_HMIDataByte[3] == 0):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[1] = "POLE"
                    # Si no hay velocidad seleccionada aseguramos que no quede presionado el toggle
                    self.b_boton_toggle_pole_cw.set_active(False)

                    # Se limpia la bandera de on_condition
                    b_on_condition = False
                    self.actualizar_etiquetas_msg("The set speed is 0 for this movement...", "red")

                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    return True
                
                elif(b_cwLimitPole == True):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[1] = "POLE"
                    self.fr_etiqueta_msg.set_text("POLE CW limit alarm!")
                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite horario alcanzado en POLE")
                    depurador(1, "HMI", " ")

                    if self.b_beeps == True:
                        zumbador.beep_alarm()

                    # Se desactiva botón toggle para que no quede coloreado
                    self.b_boton_toggle_pole_cw.set_active(False)

                    return True
                else:
                    # si no hay alarmas de límite por software desde RTU se mueve POLE CW
                    a_HMIDataString[0] = "FREE_RUN"
                    a_HMIDataString[1] = "POLE"
                    a_HMIDataString[2] = "DIR_CW"

                    if self.b_beeps == True:
                        zumbador.beep_button()
                
        elif (s_boton_fr == "fr_toggle_pole_ccw"):
            # se verifica si el botón esta presionado o suelto
            b_toggle_pole_ccw = button.get_active()

            if (b_toggle_pole_ccw == False):
                self.detener_movimientos("POLE")
                return

            elif (b_toggle_pole_ccw == True):
                # Si se activó un toggle_tot primero aseguro que el opuesto
                # se desactive
                a_HMIDataString[0] = "STOP"
                a_HMIDataString[1] = "POLE"
                # se asegura que se envíe al menos una trama antes de enviar el siguiente STOP sino la RTU no se va a enterar
                time.sleep(ui_DELAY_ms_TOGGLE/1000)

                self.b_boton_toggle_pole_cw.set_active(False)

                # Si la velocidad seteada es 0 no hay nada que hacer aquí
                if(a_HMIDataByte[3] == 0):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[1] = "POLE"

                    # Si no hay velocidad seleccionada aseguramos que no quede presionado el toggle
                    self.b_boton_toggle_pole_ccw.set_active(False)

                    # Se limpia la bandera de on_condition
                    b_on_condition = False
                    self.actualizar_etiquetas_msg("The set speed is 0 for this movement...", "red")

                    if self.b_beeps == True:
                        zumbador.beep_stop()

                    return True
                
                if(b_ccwLimitPole == True):
                    a_HMIDataString[0] = "STOP"
                    a_HMIDataString[1] = "POLE"
                    self.fr_etiqueta_msg.set_text("POLE CCW limit alarm!")
                    depurador(1, "HMI", "****************************************")
                    depurador(1, "HMI", "- Límite anti-horario alcanzado en POLE")
                    depurador(1, "HMI", " ")

                    if self.b_beeps == True:
                        zumbador.beep_alarm()

                    # Se desactiva botón toggle para que no quede coloreado
                    self.b_boton_toggle_pole_ccw.set_active(False)

                    return True
                else:
                    # si no hay alarmas de límite por software desde RTU se mueve POLE CCW
                    a_HMIDataString[0] = "FREE_RUN"
                    a_HMIDataString[1] = "POLE"
                    a_HMIDataString[2] = "CCW_DIR"

                    if self.b_beeps == True:
                        zumbador.beep_button()
            
        depurador(1, "HMI", "****************************************")
        depurador(1, "HMI", "- Moviendo "+ a_HMIDataString[1] + " en sentido " + a_HMIDataString[2])
        depurador(1, "HMI", " ")
        
        b_on_condition = False
        self.actualizar_etiquetas_msg("Moving "+ a_HMIDataString[1] + " , " + a_HMIDataString[2], "green")
        registrador.info("Moving "+ a_HMIDataString[1] + " , " + a_HMIDataString[2])
        

        return True


    ##
    # @brief Función que administra los botones de control COL y ROW
    # en solapa Manual.
    # @param self Puntero al objeto HMI
    # @param button Botones de la solapa Manual, sección Guide Tube Position.
    def control_manual(self, button):
        global simu
        
        global a_HMIDataString

        # Verificaciones previas al movimiento
        # no  si no se han elegido todos los archivos de configuración no se avanza
        if not self.archivos():

            if self.b_beeps == True:
                zumbador.beep_stop()

            return True
        
        # Antes de mover verifica si está activado el control principal
        if (a_HMIDataString[3] == "DISABLE_CONTROL"):
            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.actualizar_etiquetas_msg("Fixture control is disabled...", "red")

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

        registrador.info("Manual selection for col " + str(self.ui_plan_col) + ", row " + str(self.ui_plan_row))

        return True

    ##
    # @brief Función se ejecuta cada ui_REFRESCO_ms_RELOJ para actualizar las etiquetas de hora,
    # fecha y de mensajes de estado de conexiòn con RTU
    # @param self Puntero al objeto HMI
    def leer_reloj(self):
        global b_connect
        global b_on_condition
        
        global a_HMIDataString
        global a_HMIDataByte

        s_fecha, s_hora = str(dt.datetime.now()).split(' ')
        s_anio, s_mes, s_dia = s_fecha.split('-')
        s_hr, s_min, s_seg = s_hora.split(':') 

        # Se actualizarn las etiquetas de hora y fecha en todas las solapas
        self.actualizar_etiquetas_reloj(s_hr+":"+s_min, s_dia+"/"+s_mes+"/"+s_anio) 

        self.actualizar_etiquetas_temperatura(f_temperatura)              

        # Si no hay conexión con RTU se avisa constantemente, si se conectó, la variable
        # print_status permite que se muestre el aviso de conexión solo una vez.
        if(b_connect  == False and self.b_print_status == False):
            self.detener_movimientos("TOTAL")
            self.actualizar_etiquetas_msg("Attempting to connect to NFC...", "red")
            self.b_print_status = True
            self.inicio_etiqueta_estado_red.set_markup("<span foreground='red'> " + "Disconnected :-(" + " </span>")

            registrador.error("Attempting to connect to NFC")

        elif(b_connect == True and self.b_print_status == True):
            if(self.b_beeps == True):
                zumbador.beep_primordial()

            # Cuando no hay conexión con RTU intenta 
            depurador(1, "HMI", "****************************************")
            depurador(1, "HMI", "- Conexión HMI-RTU: " + str(b_connect))
            self.actualizar_etiquetas_msg("HMI connected to NFC !", "green")
            self.inicio_etiqueta_estado_red.set_markup("<span foreground='green'> " + "Connected :-)" + " </span>")
            self.b_print_status = False

            registrador.info("HMI connected to NFC successfully")

        if (b_on_condition == True):
            self.actualizar_etiquetas_msg("Fixture is ON CONDITION", "green")
            registrador.info("Fixture is ON CONDITION")


        # Se monitorea continuamente si viene alguna señal de atasque desde RTU
        if (b_stallAlm == True and a_HMIDataString[4] == "STALL_ENABLE"):
            if self.b_beeps == True:
                zumbador.beep_stop()

            # Se limpia la bandera de on_condition
            b_on_condition = False
            self.detener_movimientos("TOTAL")
            # Se indica en todas las etiquetas de msg del hmi
            self.actualizar_etiquetas_msg("Fixture harness is stalled, main control disabled...", "red")
            registrador.info("Stall detected")

        if a_HMIDataString[6] == "CAL_SET":   
            # Se aguarda un tiempo mayor al inter-trama para asegurar el envío de al menos una trama de calibración
            time.sleep(0.015)
            # Se limpia la bandera de calibración, la cual solo se activa una vez al presionar botón SET CAL POINT o al iniciar el sistema
            a_HMIDataString[6] = "NOP_CAL"  

        return True

    ##
    # @brief Función que se ejecuta al cerrar la ventana del HMI
    # y elimina todos los procesos de python corriendo
    # @param self Puntero al objeto HMI
    # @param button botón de cierre de ventana arriba a la derecha
    # @return None 
    def cerrar_hmi(self, button):
        registrador.info("---------- Closing HUMAN-MACHINE-INTERFACE for SM-13 -----------")
        os.system('pkill -9 python3')
        return True

##
# @brief Función que se ejecuta en un hilo separado del HMI. Inicia la conexión con RTU
# y gestiona el intercambio de datos entre HMI y RTU.
# @param None Conoce los datos que HMI desea transmitir mediante array globales
# @return None Comunica los datos recibidos desde RTU al HMI mediante array global
def tm():
    global a_HMIDataByte
    global a_HMIDataString
    global b_connect
    global b_on_condition
    global s_sock
    global s_port
    global s_ip
    global b_simulador
    global ui_pole_rdc_offset
    global ui_arm_rdc_offset
    
    
    # Valores ethernet desde RTU
    global f_angActPole
    global f_angActArm
    global f_temperatura 
    global f_velActPole 
    global b_cwLimitArm 
    global b_ccwLimitArm 
    global b_cwLimitPole 
    global b_ccwLimitPole 
    global b_limitUp 
    global b_limitDown 
    global b_stallAlm 
    global b_onCondition 
    global ui_armRdcStatus 
    global ui_poleRdcStatus
    global ui_rtuStatus 
    global ui_armRdcStatus
    global ui_poleRdcStatus
    
#                           RX FROM RTU
#   --------------------------------------
#   f_resActArm     <--     a_RTUDataRx[0] *
#   f_resActPole    <--     a_RTUDataRx[1] *
#   f_temperatura   <--     a_RTUDataRx[2] *
#   f_velActPole    <--     a_RTUDataRx[3] *
#   b_cwLimitArm    <--     a_RTUDataRx[4] *
#   b_ccwLimitArm   <--     a_RTUDataRx[5] *
#   b_cwLimitPole   <--     a_RTUDataRx[6] *
#   b_ccwLimitPole  <--     a_RTUDataRx[7] *
#   b_limitUp       <--     a_RTUDataRx[8] *
#   b_limitDown     <---     a_RTUDataRx[9] *
#   b_stallAlm      <--     a_RTUDataRx[10] *
#   b_onCondition   <--     a_RTUDataRx[11] *
#   rtu_b_armRdcStatus  <--     a_RTUDataRx[12] *
#   rtu_b_poleRdcStatus <--     a_RTUDataRx[13] *
#   b_rtuStatus     <--     a_RTUDataRx[14] *
#
    
    ui_PERIODO_ms_TRAMA = 10 # (mili-segundos)
    ui_PERIODO_ms_ANGULOS_SIMULADOS = 150
    f_ERROR_ANGULOS_SIMULADOS = 3 # [grados]
    f_INCREMENTO_ANGULOS_SIMULADOS = 1 # [grados]
    ui_CMD_ACT_ERROR = 6 # (cuentas)
    ui_STABLE_CONTROL = 50 # (cantidad de tramas para determinar si el control es estable)

    a_HMIDataByte[0] = 0
    a_HMIDataByte[1] = 0

    a_HMIDataByteTx=[0, 0, 0 ,0]

    # Varialbe local para enviar un SET_CAL una sola vez con CONTROL_ENABLE
    toggle_enable = False

    while(True):                          
#        RX FROM RTU
#   --------------------------------------
#   f_resActArm     <--     a_RTUDataRx[0] *
#   f_resActPole    <--     a_RTUDataRx[1] *
#   f_temperatura   <--     a_RTUDataRx[2] *
#   f_velActPole    <--     a_RTUDataRx[3] *
#   b_cwLimitArm    <--     a_RTUDataRx[4] *
#   b_ccwLimitArm   <--     a_RTUDataRx[5] *
#   b_cwLimitPole   <--     a_RTUDataRx[6] *
#   b_ccwLimitPole  <--     a_RTUDataRx[7] *
#   b_limitUp       <--     a_RTUDataRx[8] *
#   b_limitDown     <---     a_RTUDataRx[9] *
#   b_stallAlm      <--     a_RTUDataRx[10] *
#   b_onCondition   <--     a_RTUDataRx[11] *
#   rtu_b_armRdcStatus  <--     a_RTUDataRx[12] *
#   rtu_b_poleRdcStatus <--     a_RTUDataRx[13] *
#   b_rtuStatus     <--     a_RTUDataRx[14] *
#
        # Código de simulación de variación en ángulos actuales, sólo funciona off-line
        # Pemite que si no está activo el simulador y no hay conexión, intente reconectar más rápido.
        if (b_connect == False and b_simulador == False):
            f_angActPole = a_HMIDataByte[1]
            f_angActArm = a_HMIDataByte[0]

        # Si no hay conexión con RTU y el operador quiere simular el SM-13 se le da más prioridad que
        # la reconexión
        
        if (b_connect == False and b_simulador == True):
            try:
                if a_HMIDataByte[1] < 0:
                    a_HMIDataByte[1] = a_HMIDataByte[1] + 360
                if a_HMIDataByte[0] < 0:
                    a_HMIDataByte[0] = a_HMIDataByte[0] + 360

                if f_angActArm == a_HMIDataByte[0]:
                    pass
                else:
                    if np.abs(a_HMIDataByte[0]) > f_angActArm:
                        f_angActArm += f_INCREMENTO_ANGULOS_SIMULADOS
                        if f_angActArm >= 360:
                            f_angActArm = 0
                    elif np.abs(a_HMIDataByte[0]) < f_angActArm:
                        f_angActArm -= f_INCREMENTO_ANGULOS_SIMULADOS
                        if f_angActArm < 0:
                            f_angActArm = 0

                    if np.abs(f_angActArm - a_HMIDataByte[0]) <= f_ERROR_ANGULOS_SIMULADOS:
                        f_angActArm = a_HMIDataByte[0]

                if f_angActPole == a_HMIDataByte[1]:
                    pass
                else:
                    if np.abs(a_HMIDataByte[1]) > f_angActPole:
                        f_angActPole += f_INCREMENTO_ANGULOS_SIMULADOS
                        if f_angActPole >= 360:
                            f_angActPole = 0
                    elif np.abs(a_HMIDataByte[1]) < f_angActPole:
                        f_angActPole -= f_INCREMENTO_ANGULOS_SIMULADOS
                        if f_angActPole < 0:
                            f_angActPole = 0

                    if np.abs(f_angActPole - a_HMIDataByte[1]) <= f_ERROR_ANGULOS_SIMULADOS:
                        f_angActPole = a_HMIDataByte[1]
                # fin código de simulación de ángulos actuales SM-13
            except:
                pass

            time.sleep(ui_PERIODO_ms_ANGULOS_SIMULADOS/1000)
        
        # Si no se está jugando con el simulador de ángulos actuales y no hay conexión, se le da 
        # prioridad a la reconexión.
        if (b_connect == False and f_angActPole == a_HMIDataByte[1] and f_angActArm == a_HMIDataByte[0]):
            # Cuando no hay conexión con RTU intenta 
            depurador(1, "TM", "****************************************")
            depurador(1, "TM", "- Intentando conectar con RTU...")
            depurador(1, "TM", "- IP actual: " + s_ip + " / Port: " + s_port)
            depurador(1, "TM", "- Estado conexión con RTU: " + str(b_connect))
            depurador(1, "TM", " ")

            try:
                s_sock, b_connect = RTU_connect(b_connect, s_ip, int(s_port))
                pass
            except socket.error as e:
                depurador(1, "TM", "****************************************")
                depurador(1, "TM", "- No se puede establecer comunicación con RTU: " + str(e))
                depurador(1, "TM", " ")

            # Sea_HMIDataString[6] = "CAL_SET" asegura que ante una desconexión el modo esté en STOP 
            b_on_condition = False
            a_HMIDataString[0] = "STOP"

        elif (b_connect == True):
            try:
                depurador(4, "TM", "- OFFSETs POLE, ARM: " + str(ui_pole_rdc_offset) + ", " + str(ui_arm_rdc_offset)) 
                depurador(4, "TM", "--> Paquete Data_String: " + str(a_HMIDataString))

                if a_HMIDataString[3] == "CONTROL_ENABLE" and toggle_enable == True:
                    # Se actualiza byte de Tx con el ángulo de offset correspondiente, luego el conversor lo pasará a cuentas
                    # sin corregir con offset 
                    a_HMIDataByte[1] = float((ui_pole_rdc_offset/65535)*360.0)
                    a_HMIDataByte[0] = float((ui_arm_rdc_offset/65535)*360.0)
                    # Se envía el comando de "CAL_SET" para establecer la calibración del punto seleccionados en la RTU
                    a_HMIDataString[6] = "CAL_SET"
                    toggle_enable = False
                
                if a_HMIDataString[3] == "DISABLE_CONTROL" and toggle_enable == False:
                    toggle_enable = True

                # ***************************************************************************************************************************
                # Antes de enviar los ángulos comandandos en a_HMIDataByte[0]/[1] se deben convertir a cuentas de resolver
                a_HMIDataByteTx[1], a_HMIDataByteTx[0] = conversor(0, 0, a_HMIDataByte[1], a_HMIDataByte[0], 0, 0, "angulo_a_cuenta")
                a_HMIDataByteTx[2] = a_HMIDataByte[2]
                a_HMIDataByteTx[3] = a_HMIDataByte[3]
                
                if (a_HMIDataString[6] == "CAL_SET"):
                    depurador(1, "TM", "****************************************")
                    depurador(1, "TM", "- Enviando trama " + str(a_HMIDataString[6]) + ", POLE " + str(a_HMIDataByteTx[1]) + ", ARM " + str(a_HMIDataByteTx[0])) 
                    #registrador.info("Enviando trama " + str(a_HMIDataString[6]) + ", POLE " + str(a_HMIDataByteTx[1]) + ", ARM " + str(a_HMIDataByteTx[0]))

                # Se envían los paquetes DataBytes y DataStrings hacia RTU y se recibe un paquete proveniente de RTU 
                a_RTUDataRx, b_connect, s_sock = enviar_a_y_recibir_de_rtu(a_HMIDataByteTx, a_HMIDataString, b_connect, s_sock, s_ip, s_port)#'192.168.0.193', 5020)
                
                
                f_resActArm = a_RTUDataRx[0]  # Cuentas de Resolver
                f_resActPole = a_RTUDataRx[1] # Cuentas de Resolver
                f_temperatura = a_RTUDataRx[2]
                f_velActPole = a_RTUDataRx[3] # TODO Eliminar
                b_cwLimitArm = a_RTUDataRx[4] # TODO Eliminar
                b_ccwLimitArm = a_RTUDataRx[5]
                b_cwLimitPole = a_RTUDataRx[6]
                b_ccwLimitPole = a_RTUDataRx[7]
                b_limitUp = a_RTUDataRx[8]
                b_limitDown = a_RTUDataRx[9]
                b_stallAlm  = a_RTUDataRx[10]
                b_onCondition = a_RTUDataRx[11]
                ui_armRdcStatus = a_RTUDataRx[12]
                ui_poleRdcStatus = a_RTUDataRx[13]
                #b_rtuStatus = a_RTUDataRx[14]
              
                
                
                # Apenas se actualiza a_RTUDataRx se convierten los elementos [0] y [1] de cuentas resolver a ángulos.
                f_angActPole, f_angActArm = conversor(f_resActPole, f_resActArm, 0, 0, ui_pole_rdc_offset, ui_arm_rdc_offset, "cuenta_a_angulo")
                # y también se actualizan los demás datos
                
                
                # Se monitorean continuamente los valores de cuenta comandada y recibida para establecer o no
                # el estado de ON_CONDITION del FH.
                ui_cmd_pole, ui_cmd_arm = 0, 0
                try:
                    ui_cmd_pole, ui_cmd_arm  = conversor(0, 0, a_HMIDataByte[1], a_HMIDataByte[0], ui_pole_rdc_offset, ui_arm_rdc_offset, "angulo_a_cuenta")
                    ui_error_control_arm = abs(int(ui_cmd_arm) - int(f_resActArm))
                    ui_error_control_pole = abs(int(ui_cmd_pole) - int(f_resActPole))
                except:
                    ui_error_control_arm = ui_CMD_ACT_ERROR
                    ui_error_control_pole = ui_CMD_ACT_ERROR

                # Solo ON CONDITION si el FH no está en STOW
                #if (a_HMIDataByteTx[0] != 0):# TODO: and a_HMIDataByteTx[1] != 0):

                
                # Esta porción de código manda un Stop en la trama cuando se alcanzó el umbral de control, en modo automático
                # y se muestra msg ON CONDITION en HMI.        
                if (a_HMIDataString[0] == "AUTOMATIC" and b_onCondition == True):  
                    b_on_condition = True
                    depurador(1, "TM", "****************************************")
                    depurador(1, "TM", "- Fixture is ON CONDITION ")
                    depurador(1, "TM", "- ")

                    # Si RTU dice que se está on-el FH alcanzó el ON CONDITION se pasa el modo de AUTO a STOP
                    a_HMIDataString[0] = "STOP"
                
                if (a_HMIDataString[0] == "FREE_RUN" or a_HMIDataString[0] == "LIFT"): 
                    b_on_condition = False
                
                # ***************************************************************************************************************************

                depurador(4, "TM", "****************************************")
                depurador(4, "TM", "- POLE pos cmd [res] : " + str(a_HMIDataByteTx[1]).zfill(4) + "\t| POLE pos cmd [ang] (con offset): " + str(a_HMIDataByte[1]) + "º")
                depurador(4, "TM", "- POLE pos act [res] : " + str(f_resActPole).zfill(4) + "\t| POLE pos act [ang] (con offset): " + str(round(f_angActPole, 3)) + "º")
                
                
                depurador(4, "TM", " ")                
                depurador(4, "TM", "- ARM  pos cmd [res] : " + str(a_HMIDataByteTx[0]).zfill(4) + "\t|  ARM pos cmd [ang] (con offset): " + str(a_HMIDataByte[0]) + "º")
                depurador(4, "TM", "- ARM  pos act [res] : " + str(f_resActArm).zfill(4) + "\t|  ARM pos act [ang] (con offset): " + str(round(f_angActArm, 3)) + "º")
                depurador(4, "TM", "- ARM  vel cmd: " +str(a_HMIDataByte[2]) + "\t\t\t|  ARM  vel act: " + str(f_temperatura))
                depurador(4, "TM", " ")  

            except Exception as e:
                b_connect = False
                depurador(2, "TM", "****************************************")
                depurador(2, "TM", "- Error durante la comunicación con RTU: " + str(e))
                depurador(2, "TM", "- CONNECT : " + str(b_connect))
                depurador(2, "TM", "- SOCKET : " + str(s_sock))
                depurador(2, "TM", "- IP : " + str(s_ip))
                depurador(2, "TM", "- PORT : " + str(s_port))
                depurador(2, "TM", " ")

            time.sleep(ui_PERIODO_ms_TRAMA/1000)
            
        # Esta porción de código permite que no se detenga el HMI con la llegada de
        # alguna trama corrupta+
        for x in range(0, 14):
            try:
                if a_RTUDataRx[x] == True:
                    pass
            except Exception as e:
                depurador(1, "TM", "****************************************")
                depurador(1, "TM", "- Error en trama recibida de RTU: " + str(e))   
                ## Variable global contiene la lista de datos que llegan desde RTU:
                # [f_angActArm, f_angActPole, f_temperatura, 
                # b_cwLimitArm, b_ccwLimitArm, b_cwLimitPole, b_ccwLimitPole, b_limitUp, b_limitDown, b_stallAlm, ui_status]
                a_RTUDataRx = [0, 0, 0, 0, False, False, False, False, False, False, False, False, 0, ui_armRdcStatus, ui_poleRdcStatus, 0]
                pass  
        
                
if __name__ == "__main__":
    
    interfaz_hombre_maquina = hmi_SM13()

    telemetria = threading.Thread(target=tm).start()
        
    Gtk.main()

    #telemetria.join()

    
