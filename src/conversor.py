##
##
# @file conversor.py
#
# @brief Función que convierte de ángulo a cuenta y viceversa según la indicación de s_msg.
# Se tiene en cuenta la calibración de offset para cada eje, es decir, el valor de cuenta resolver para 0ª

# @param ui_res_act_pole valor de conversión real del RDC POLE que llega de RTU y se que desea converti a ángulo
# @param ui_res_act_arm valor de conversión real del RDC ARM que llega de RTU y se que desea converti a ángulo
# @param f_ang_cmd_pole ángulo comandado POLE que se quiere convertir a cuentas resolver para enviar por trama
# @param f_ang_cmd_arm ángulo comandado ARM que se quiere convertir a cuentas resolver para enviar por trama
# @param s_msg tipo de conversión (angulo_a_cuenta o cuenta_a_angulo)
# @return f_ang_act_pole ángulo convertido (en grados) actuales del eje POLE 
# @return f_ang_act_arm ángulo convertido (en grados) actuales del eje ARM
# @return ui_res_cmd_pole cuenta convertida para comandar el eje POLE
# @return ui_res_cmd_arm cuenta convertida para comandar el eje ARM

from depurador import *

def conversor(ui_res_act_pole, ui_res_act_arm, f_ang_cmd_pole, f_ang_cmd_arm, ui_offset_1, ui_offset_2, s_msg):

    ui_MAX_CUENTAS = 65535
    f_MAX_GRADOS = 359.999
    f_PENDIENTE_RES_ARM = float(65536.0/360.0)#182.04 # (ctas/grados)
    f_PENDIENTE_RES_POLE = float(65536.0/360.0) # (ctas/grados)
    ui_cruce_por_cero_res_pole = 0
    ui_cruce_por_cero_res_arm = 0

    depurador(3, "CONVERSOR", "****************************************")
    depurador(3, "CONVERSOR", "- Offset POLE: " + str(ui_offset_1))
    depurador(3, "CONVERSOR", "- Offset ARM : " + str(ui_offset_2))
    depurador(3, "CONVERSOR", " ")

    ui_offset_pole = int(ui_offset_1)
    ui_offset_arm = int(ui_offset_2)

    if(s_msg == "cuenta_a_angulo"):
        # Se verifica el valor actual de encoder para saber qué valor de cruce por cero utilizar.
        # Más info en RDCvsANG.xls
        if ui_res_act_arm >= 0 and ui_res_act_arm < ui_offset_arm:
            ui_cruce_por_cero_res_arm = ui_offset_arm - ui_MAX_CUENTAS
        elif ui_res_act_arm >= ui_offset_arm and ui_res_act_arm <= ui_MAX_CUENTAS:
            ui_cruce_por_cero_res_arm = ui_offset_arm

        if ui_res_act_pole >= 0 and ui_res_act_pole < ui_offset_pole:
            ui_cruce_por_cero_res_pole = ui_offset_pole - ui_MAX_CUENTAS
        elif ui_res_act_pole >= ui_offset_pole and ui_res_act_pole <= ui_MAX_CUENTAS:
            ui_cruce_por_cero_res_pole = ui_offset_pole

        depurador(3, "CONVERSOR", "****************************************")
        depurador(3, "CONVERSOR", "- Cruce POLE: " + str(ui_cruce_por_cero_res_pole))
        depurador(3, "CONVERSOR", "- Cruce ARM : " + str(ui_cruce_por_cero_res_arm))
        depurador(3, "CONVERSOR", " ")

        # angulo = (Nres - intersecciòn)/pendiente
        f_ang_act_arm = float(ui_res_act_arm - ui_cruce_por_cero_res_arm)/f_PENDIENTE_RES_ARM
        f_ang_act_pole = float(ui_res_act_pole - ui_cruce_por_cero_res_pole)/f_PENDIENTE_RES_POLE 

        depurador(3, "CONVERSOR", "****************************************")
        depurador(3, "CONVERSOR", "- Ángulo ACT POLE: " + str(f_ang_act_pole))
        depurador(3, "CONVERSOR", "- Ángulo ACT ARM : " + str(f_ang_act_arm))
        depurador(3, "CONVERSOR", " ")

        return f_ang_act_pole, f_ang_act_arm   

    elif(s_msg == "angulo_a_cuenta"):

        f_ang_arm_offset = float((ui_MAX_CUENTAS - ui_offset_arm))/f_PENDIENTE_RES_ARM
        f_ang_pole_offset = float((ui_MAX_CUENTAS - ui_offset_pole))/f_PENDIENTE_RES_POLE
                
        # según el ángulo que se quiera convertir se deberá distinguir qué valor de cruce por cero usar.
        # Más info en RDCvsANG.xls
        if f_ang_cmd_arm >= 0 and f_ang_cmd_arm <= f_ang_arm_offset:
            ui_cruce_por_cero_res_arm = ui_offset_arm
        elif f_ang_cmd_arm > f_ang_arm_offset and f_ang_cmd_arm <= f_MAX_GRADOS :
            ui_cruce_por_cero_res_arm = ui_offset_arm - ui_MAX_CUENTAS

        if f_ang_cmd_pole >= 0 and f_ang_cmd_pole <= f_ang_pole_offset:
            ui_cruce_por_cero_res_pole = ui_offset_pole 
        elif f_ang_cmd_pole > f_ang_pole_offset and f_ang_cmd_pole <= f_MAX_GRADOS :
            ui_cruce_por_cero_res_pole = ui_offset_pole - ui_MAX_CUENTAS

        # cuenta = pendiente*cuenta + interseccióm
        ui_res_cmd_arm  = abs(int(float(f_PENDIENTE_RES_ARM)*f_ang_cmd_arm + ui_cruce_por_cero_res_arm))
        ui_res_cmd_pole = abs(int(float(f_PENDIENTE_RES_POLE)*f_ang_cmd_pole + ui_cruce_por_cero_res_pole))  

        depurador(2, "CONVERSOR", "****************************************")
        depurador(2, "CONVERSOR", "- Cuenta CMD POLE: " + str(ui_res_cmd_pole))
        depurador(2, "CONVERSOR", "- Cuenta CMD ARM : " + str(ui_res_cmd_arm))
        depurador(2, "CONVERSOR", " ") 

        return int(ui_res_cmd_pole), int(ui_res_cmd_arm) 

#if __name__ == "__main__":
#    conversor(2855, 10000, 10, 10, 2851, 9900, "cuenta_a_angulo")
#    conversor(2855, 10000, 10, 10, 2851, 9900, "angulo_a_cuenta")