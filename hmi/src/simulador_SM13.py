from dk_SM13 import *
from ik_SM13 import *
from leer_archivo_hx import *
from leer_datos_SM13 import *
from leer_specs_tubos import *
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import math
import time
from depurador import *

##
# Clase Simulador_SM13
# @brief La clase que implementa el simulador SM-13
#
class simulador_SM13(object):
    
    ##
    # El destructor de la clase simulador
    #
    # @brief Esta función destruye el objeto simulador cuando se presiona el
    # botón Simulator y estaba creado el mismo.
    #
    # @param self Puntero al objeto
    #
    # @return none
    #
    # @author Cristian Torres Barrios
    # creado Vie 18 Sep 20:35:00 2020
    def __del__(self):
        plt.close(self.fig)
        
    ##
    # El constructor de la clase simulador
    #
    # @brief Esta función inicializa el simulador SM-13 cuando
    # se presiona el botón Simulator
    #
    # @param self Puntero al objeto
    # @param f_x Coordenada X en pulgadas
    # @param f_y Coordenada Y en pulgadas
    # @param s_f_file Dirección del archivo que contiene el tipo de fixture
    # @param ui_m El montaje del fixture respecto al hx
    # @param s_hx_file Dirección del archivo que contiene los datos de los tubos del hx
    #
    # @return none
    #
    # @author Cristian Torres Barrios
    # creado Vie 18 Sep 20:35:00 2020
    def __init__(self, f_x, f_y, s_f_file, ui_m, s_hx_file):        
        
        plt.ion()
        self.f_x_coor = f_x
        self.f_y_coor = f_y
        #self.a_lista_tubos_x = a_hx_x
        #self.a_lista_tubos_y = a_hx_y
        self.ui_tipo_montaje = ui_m
        self.s_tipo_fixture = s_f_file
 
        # Si se cambia el path, procurar que tenga la misma cantidad de carpetas
        trash, s_folder1, s_folder2, s_folder3, s_folder4, s_folder5, s_folder6, s_folder7, s_folder8, trash = s_hx_file.split("/")
        
        # Obtiene las dimensiones físicas del telemanipulador y su posición de montaje
        self.f_Lx, self.f_Ly, self.f_Lp, self.f_La, f_w, f_h = leer_datos_SM13(self.s_tipo_fixture, self.ui_tipo_montaje)
        
        # Initialize plot and line objects for target, end effector, and arm.
        # Turn on interactive plotting and show plot.   
        self.fig, ax = plt.subplots(figsize=(6,4.7))
        self.fig.subplots_adjust(left=0, bottom=0, right=1, top=1)
        ax.grid(False)
        
        self.tubo, = ax.plot([], [], marker='o', c='r')
        self.codo, = ax.plot([], [], marker='o', c='b', lw=4)
        self.sonda, = ax.plot([], [], marker='o', markerfacecolor='w', c='b', lw=4)
        
        # Lee archivo de especificaciones de los tubos del hx seleccionado
        s_tube_specs_path = "/"+s_folder1+"/"+s_folder2+"/"+s_folder3+"/"+s_folder4+"/"+s_folder5+"/"+s_folder6+"/"+s_folder7+"/"+s_folder8+"/tube_specs.csv"
        a_specs_tube = leer_specs_tubos(s_tube_specs_path)
        f_tube_od = float(a_specs_tube[0])
        f_y_pitch = float(a_specs_tube[1])
        f_x_pitch = float(a_specs_tube[2])
        f_calle_ancha = float(a_specs_tube[4])
            
        # Limites físicos del intercambiador para incorporárlos al simulador
        # (a la imagen se le da medio tubo_od para cada lado por eso se suma un tube_od)
        f_xmin = float(a_specs_tube[8]) + f_tube_od
        f_xmax = float(a_specs_tube[9]) - f_tube_od 
        f_ymin = float(a_specs_tube[10]) + f_tube_od
        f_ymax = float(a_specs_tube[11]) - f_tube_od
            
        # Imagen de fondo para el simulador
        background = "/"+s_folder1+"/"+s_folder2+"/"+s_folder3+"/"+s_folder4+"/"+s_folder5+"/"+s_folder6+"/"+s_folder7+"/"+s_folder8+"/tubesheet_image.png"
        
        # Se fijan los límites del gráfico del simulador en base al hx
        ax.set_xlim(f_xmax, f_xmin)
        ax.set_ylim(f_ymin, f_ymax)
        
        # Add SM-13 base plate
        ax.add_patch(Rectangle((self.f_Lx-f_w/2, self.f_Ly-f_h/2), width=f_w, height=f_h, facecolor='c', alpha=1))
        circulo_no = plt.Circle((self.f_Lx-8*f_x_pitch, self.f_Ly-4*f_y_pitch), f_tube_od/2, ls='-', fill=True, color='k')
        circulo_ne = plt.Circle((self.f_Lx+8*f_x_pitch, self.f_Ly-4*f_y_pitch), f_tube_od/2, ls='-', fill=True, color='k')
        circulo_so = plt.Circle((self.f_Lx-8*f_x_pitch, self.f_Ly+4*f_y_pitch), f_tube_od/2, ls='-', fill=True, color='k')
        circulo_se = plt.Circle((self.f_Lx+8*f_x_pitch, self.f_Ly+4*f_y_pitch), f_tube_od/2, ls='-', fill=True, color='k')
        ax.add_artist(circulo_no)
        ax.add_artist(circulo_ne)
        ax.add_artist(circulo_so)
        ax.add_artist(circulo_se)
        
        # Add dashed circle to plot indicating reaches min and max.
        #Determine maximum reach of arm.
        f_alcance_max = self.f_Lp + self.f_La
        f_alcance_min = self.f_La - self.f_Lp
        circulo_max = plt.Circle((self.f_Lx, self.f_Ly), f_alcance_max, ls='dashed', fill=False, color='r')
        ax.add_artist(circulo_max)

        circulo_min = plt.Circle((self.f_Lx, self.f_Ly), f_alcance_min, ls='dashed', fill=False, color='r')
        ax.add_artist(circulo_min)
        """       
        img_hx = True
        if img_hx == False:
            j = 0
            for i in range(0,len(self.a_lista_tubos_x)-1):
                circulo_tubo_I_IV = plt.Circle((float(self.a_lista_tubos_x[i]), float(self.a_lista_tubos_y[i])), f_tube_od/2, ls='-', fill=False)
                circulo_tubo_II_III = plt.Circle((float(self.a_lista_tubos_x[i]), float(self.a_lista_tubos_y[i]) - f_calle_ancha - 2*j*f_y_pitch), f_tube_od/2, ls='-', fill=False)
                if(float(self.a_lista_tubos_y[i+1]) != float(self.a_lista_tubos_y[i])):
                    j += 1
                    
                ax.add_artist(circulo_tubo_I_IV)
                ax.add_artist(circulo_tubo_II_III)
                
        elif img_hx == True:
        """
        try:
            img = plt.imread(background)
            ax.imshow(img, extent=[f_xmin, f_xmax, f_ymax, f_ymin])
        except:
            depurador(1, "Simulador", "****************************************")
            depurador(1, "Simulador", "- Error al cargar imagen del HX")
            depurador(1, "Simulador", " ")
            pass
                   
        #    ax.plot(float(a_lista_px[x]), float(a_lista_py[x]), marker='o', c='g')

        
        plt.ion()
        plt.show()
    
    ##
    # Método refrescar_grafico
    #
    # @brief Esta función permite refrescar el simulador sin tener que inicializarlo
    # cada vez que hay una actualización del target.
    #
    # @param self Puntero al objeto
    # @param f_x Coordenada X en pulgadas
    # @param f_y Coordenada Y en pulgadas
    # @param f_p Ángulo de la articulación POLE respecto a la base
    # @param f_a Ángulo de la articulación ARM respecto al POLE
    #
    # @return none
    #
    # @author Cristian Torres Barrios
    # creado Vie 18 Sep 20:35:00 2020
    def refrescar_grafico(self, f_x, f_y, f_p, f_a):
          
        self.f_pole = f_p
        self.f_arm = f_a
        self.f_x_coor = f_x
        self.f_y_coor = f_y
        
        f_px_codo, f_py_codo, f_px_sonda, f_py_sonda = dk_SM13(self.f_pole, self.f_arm, self.f_Lx, self.f_Ly, self.f_Lp, self.f_La)
            
        self.tubo.set_data(self.f_x_coor, self.f_y_coor)
        self.codo.set_data([self.f_Lx, f_px_codo], [self.f_Ly, f_py_codo])
        self.sonda.set_data([f_px_codo, f_px_sonda], [f_py_codo, f_py_sonda])
        
        #plt.ion()
        plt.show()
            
        #self.fig.canvas.get_tk_widget().update()
        self.fig.canvas.flush_events()   
    
#if __name__ == "__main__":
#    main()
    
