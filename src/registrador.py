#~ Cristian A. Torres 2021/03/19

##
# Módulo de registro de variables de interés del HMI

import logging

FORMAT='%(asctime)s - %(levelname)s - %(message)s'
logger = logging.getLogger(__file__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/home/gspc/Proyectos/SM-13/hmi/logs/data.log')
handler.setFormatter(logging.Formatter(FORMAT))
logger.addHandler(handler)
logger.setLevel(logging.DEBUG)

##
# @brief Función que registra información de tipo "debug"
# @param data Información que se desea registrar en el programa
# @return none
def debug(data):
	logger.debug(data)

##
# @brief Función que registra información de tipo "info"
# @param data Información que se desea registrar en el programa
# @return none
def info(data):
	logger.info(data)

##
# @brief Función que registra información de tipo "warning"
# @param data Información que se desea registrar en el programa
# @return none
def warning(data):
	logger.warning(data)

##
# @brief Función que registra información de tipo "error"
# @param data Información que se desea registrar en el programa
# @return none
def error(data):
	logger.error(data)

##
# @brief Función que registra información de tipo "critical"
# @param data Información que se desea registrar en el programa
# @return none
def critical(data):
	logger.critical(data)
