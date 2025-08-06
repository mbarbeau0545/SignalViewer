"""
#  @file        main.py
#  @brief       Template_BriefDescription.
#  @details     TemplateDetailsDescription.\n
#
#  @author      mba
#  @date        jj/mm/yyyy
#  @version     1.0
"""
#------------------------------------------------------------------------------
#                                       IMPORT
#------------------------------------------------------------------------------
import sys
import os 
import time 
from IHM.IhmSigViewer import SignalViewer, QApplication
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
#------------------------------------------------------------------------------
#                                       CONSTANT
#------------------------------------------------------------------------------
import math
# CAUTION : Automatic generated code section: Start #

# CAUTION : Automatic generated code section: End #
#------------------------------------------------------------------------------
#                                       CLASS
#------------------------------------------------------------------------------
CST_MHZ_TO_HZ = 1_000_000
FMKTIM_TIMER_PWM_ARR_TARGET_16_BIT = 65535
FMKTIM_TIMER_PWM_ARR_TARGET_32_BIT = 4294967295
FMKTIM_MAX_LOOP_DECREASING = 10
FMKTIM_ARR_DECREASING_VAL_16B = 100
FMKTIM_FREQ_COMPUTE_DELTA_ACCEPTANCE = 5.0  # Hz, à ajuster
CST_MAX_UINT_16BIT = 0xFFFF
CST_MAX_UINT_32BIT = 0xFFFFFFFF

# Fréquence timer en MHz
timerFreqMHz = 128  # Exemple : 72 MHz

MAX_ARR_POSSIBLE = 41000
# Utilitaires
def get_arr_register(timer_freq_mhz, prescaler, pwm_freq):
    timer_clk_hz = timer_freq_mhz * CST_MHZ_TO_HZ
    return (timer_clk_hz / (prescaler + 1)) / pwm_freq - 1

def get_prescaler(timer_freq_mhz, arr, pwm_freq):
    timer_clk_hz = timer_freq_mhz * CST_MHZ_TO_HZ
    return (timer_clk_hz / ((arr + 1) * pwm_freq)) - 1

def get_pwm_freq(timer_freq_mhz, prescaler, arr):
    timer_clk_hz = timer_freq_mhz * CST_MHZ_TO_HZ
    return timer_clk_hz / ((prescaler + 1) * (arr + 1))

def decompose_float(value):
    integer_part = int(value)
    delta = value - integer_part
    return integer_part, delta

def get_pwm_timer_init_param(pwm_freq):
    max_arr = FMKTIM_TIMER_PWM_ARR_TARGET_16_BIT
    decreasing_value = FMKTIM_ARR_DECREASING_VAL_16B
    max_bits = CST_MAX_UINT_16BIT

    # Étape 1 : Calcul du ARR cible
    target_arr = get_arr_register(timerFreqMHz, 0, pwm_freq)
    target_arr = min(target_arr, MAX_ARR_POSSIBLE)
    
    freq_max_supported = (timerFreqMHz * CST_MHZ_TO_HZ) / (target_arr + 1)
    
    # Boucle pour réduire le ARR si la fréquence demandée est trop haute
    loop_count = 0
    while pwm_freq > freq_max_supported and loop_count < FMKTIM_MAX_LOOP_DECREASING:
        target_arr -= decreasing_value
        freq_max_supported = (timerFreqMHz * CST_MHZ_TO_HZ) / (target_arr + 1)
        loop_count += 1

    if loop_count >= FMKTIM_MAX_LOOP_DECREASING:
        raise ValueError('ggg')

    # Étape 2 : Calcul du prescaler
    prescaler_theo = get_prescaler(timerFreqMHz, target_arr, pwm_freq)
    real_prescaler, delta_prescaler = decompose_float(prescaler_theo)

    # Étape 3 : Gestion cas prescaler = 0
    if real_prescaler == 0:
        real_arr = int((timerFreqMHz * CST_MHZ_TO_HZ) / pwm_freq - 1)
    else:
        real_arr = int(target_arr + (target_arr / real_prescaler) * delta_prescaler)

    if real_prescaler > max_bits or real_arr > max_bits or real_arr == 0:
        raise ValueError('Out of range')

    # Étape 4 : Vérification de la fréquence réelle obtenue
    real_freq = get_pwm_freq(timerFreqMHz, real_prescaler - 1, real_arr)
    delta_freq = abs(pwm_freq - real_freq)

    if delta_freq > FMKTIM_FREQ_COMPUTE_DELTA_ACCEPTANCE:
        raise ValueError("delta overr")

    # Résultat final : valeurs ARR et Prescaler
    return real_arr, real_prescaler - 1, "RC_OK"

def get_best_arr_psc(pwm_freq_hz, timer_freq_mhz=128, max_val=65535, freq_tol_hz=1.0):
    timer_clk_hz = timer_freq_mhz * 1_000_000
    target_product = timer_clk_hz / pwm_freq_hz

    best_arr = 0
    best_psc = 0
    best_error = float("inf")

    # Balayage autour de la racine carrée de N
    sqrt_target = int(math.sqrt(target_product))
    idx_loop = 0

    for arr_candidate in range(max(1, sqrt_target - 1000), min(max_val, sqrt_target + 1000)):
        idx_loop += 1
        psc_candidate = target_product / (arr_candidate + 1) - 1
        psc_rounded = int(round(psc_candidate))

        if 0 <= psc_rounded <= max_val:
            actual_freq = timer_clk_hz / ((psc_rounded + 1) * (arr_candidate + 1))
            error = abs(actual_freq - pwm_freq_hz)

            if error < best_error:
                best_error = error
                best_arr = arr_candidate
                best_psc = psc_rounded

                if error <= freq_tol_hz:
                    break  # Early exit si erreur acceptable

    if best_error > freq_tol_hz:
        raise ValueError('fff')
    
    return best_arr, best_psc, "RC_OK", idx_loop


import matplotlib.pyplot as plt
import numpy as np
MAX_ARR = 63000
def function(x):
    raw_prescaler = (128 * 1_000_000) / (x * (MAX_ARR + 1)) - 1

    delta_psc = raw_prescaler - round(raw_prescaler)
    prescaler = int(raw_prescaler)

    pos_delta_psc = delta_psc if delta_psc > 0 else -delta_psc
    add_on = False
    add_two = False
    add_3 = False

    if pos_delta_psc > 0.45 and pos_delta_psc < 0.55:
        prescaler += 1
        arr_value = MAX_ARR + (MAX_ARR / prescaler) * (delta_psc)
        freq_pwm = (128 * 1000000) / ((arr_value + 1) * (prescaler + 1))
        add_on = True

    else:
        if prescaler == 0:
            arr_value = (128 * 1000000) / (x - 1)
            freq_pwm = (128 * 1000000) / (arr_value + 1)
            add_two = True
        else:
            arr_value = MAX_ARR + (MAX_ARR / prescaler) * (delta_psc)
            freq_pwm = (128 * 1000000) / ((arr_value + 1) * (prescaler + 1))
            add_3 = True


    if freq_pwm > (x + 4) or freq_pwm < (x - 4) or arr_value > 65535:
        print(f' For {x}, freq compute {freq_pwm}, arr_value {arr_value}')
    return arr_value


x_values = np.linspace(1, 1000, 1000)  # Par exemple, des fréquences de 1 Hz à 1000 Hz
y_values = [function(x) for x in x_values]

# Tracé
plt.figure(figsize=(10, 6))
plt.plot(x_values, y_values, label='function(x)', color='blue')

# Infos du graphique
plt.title('Tracé de la fonction personnalisée')
plt.xlabel('x (ex: fréquence en Hz)')
plt.ylabel('Résultat de function(x)')
plt.grid(True)
plt.legend()
plt.tight_layout()

# Affichage
plt.show()
#------------------------------------------------------------------------------
#                             FUNCTION IMPLMENTATION
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
#			                MAIN
#------------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    viewer = SignalViewer("Doc\\project_cfg.json")
    try:
        viewer.show()
        sys.exit(app.exec())   
    except (KeyboardInterrupt):
        print("[INFO] : Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f'[ERROR] : An error occured {e}')
    finally:
        viewer.kill_all_thread()


    

if __name__ == '__main__':
    pass#main()
#------------------------------------------------------------------------------
#		                    END OF FILE
#------------------------------------------------------------------------------
#--------------------------
# Function_name
#--------------------------

"""
    @brief
    @details

    @params[in]
    @params[out]
    @retval
"""

