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
#------------------------------------------------------------------------------
#                                       CONSTANT
#------------------------------------------------------------------------------
import math
# CAUTION : Automatic generated code section: Start #

# CAUTION : Automatic generated code section: End #
#------------------------------------------------------------------------------
#                                       CLASS
#------------------------------------------------------------------------------

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
    main()
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

