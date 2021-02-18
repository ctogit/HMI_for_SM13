##
# @file zumbador.py
#
# @brief Este paquete gestiona sonidos acorde a los eventos que se producen
# en la HMI. Para que funcione hay que instalar el paquete "sox", en caso
# de Linux SO. Queda pendiente adaptarlo a Win.
#
# @return none
#
# @author Cristian Torres Barrios
# creado Mie 4 Nov 20:38:00 2020

import os
from depurador import *

##
# @brief Función emite sonidos al detener movimientos del robot
# @param none
# @return none
def beep_stop():
	try:
		os.system('play -nq -t alsa synth %s sine %f' %( 0.3, 500))
	except:
		depurador(3, "zumbador", "****************************************")
		depurador(3, "zumbador", "- Error al emitir beep de stop")
		depurador(3, "zumbador", " ")

    #return True

##
# @brief Función emite sonidos al presionar botones en la interfaz
# @param none
# @return none
def beep_button():
	try:
		os.system('play -nq -t alsa synth %s sine %f' %( 0.05, 1000))
	except:
		depurador(3, "zumbador", "****************************************")
		depurador(3, "zumbador", "- Error al emitir beep de botón")
		depurador(3, "zumbador", " ")
	return True

##
# @brief Función emite sonidos cuando un evento de alarma ocurre
# @param none
# @return none
def beep_alarm():
	try:
		os.system('play -nq -t alsa synth %s sine %f' %( 0.15, 500))
		os.system('play -nq -t alsa synth %s sine %f' %( 0.15, 500))
		os.system('play -nq -t alsa synth %s sine %f' %( 0.15, 500))
	except:
		depurador(3, "zumbador", "****************************************")
		depurador(3, "zumbador", "- Error al emitir beep de alarma")
		depurador(3, "zumbador", " ")
	return True

##
# @brief Función emite sonidos a eventos importantes de la interfaz
# @param none
# @return none
def beep_primordial():
	try:
		os.system('play -nq -t alsa synth %s sine %f' %( 0.05, 500))
		os.system('play -nq -t alsa synth %s sine %f' %( 0.1, 1000))
		os.system('play -nq -t alsa synth %s sine %f' %( 0.15, 1500))
	except:
		depurador(3, "zumbador", "****************************************")
		depurador(3, "zumbador", "- Error al emitir beep de importancia")
		depurador(3, "zumbador", " ")
	return True


