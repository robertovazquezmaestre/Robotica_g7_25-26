#!/bin/bash

echo "Iniciando todos los componentes en pestañas de terminal..."

# Función para crear pestañas y ejecutar comandos
create_tab() {
    local title="$1"
    local command="$2"
    
    # Si es la primera pestaña, usa la terminal actual, sino crea nueva pestaña
    if [ -z "$FIRST_TAB_CREATED" ]; then
        FIRST_TAB_CREATED=1
        # Cambiar el título de la primera pestaña
        echo -ne "\033]0;$title\007"
        eval "$command"
    else
        gnome-terminal --tab --title="$title" -- bash -c "$command; exec bash"
    fi
}

# Iniciar rcnode en la primera pestaña
echo "Iniciando rcnode..."
create_tab "rcnode" "echo 'Iniciando rcnode...'; cd ~/robocomp/tools/rcnode && export ROBOCOMP=/home/adri/robocomp || exit; ./rcnode.sh"
RCNODE_PID=$!
echo "rcnode lanzado con PID $RCNODE_PID"
echo "rcnode listo."

sleep 3

# Iniciar Webots en segunda pestaña
echo "Abriendo Webots..."
create_tab "Webots" "cd ~/robocomp/components/webots-shadow/worlds && ~/bin/prime-run webots SimpleWorld.wbt"
#create_tab "Webots" "cd ~/robocomp/components/webots-shadow/worlds && ~/bin/prime-run webots Shadow.wbt"                                                                                                                     

sleep 2

# Iniciar Webots2Robocomp en tercera pestaña
echo "Iniciando Webots2Robocomp..."
create_tab "Webots2Robocomp" "cd ~/robocomp/components/webots-bridge && bin/Webots2Robocomp etc/config"

sleep 2

# Iniciar Lidar3D en cuarta pestaña
echo "Iniciando Lidar3D..."
create_tab "Lidar3D" "cd ~/robocomp/components/robocomp-robolab/components/hardware/laser/lidar3D && bin/Lidar3D etc/config_pearl_webots"

sleep 2

# Iniciar chocachoca en quinta pestaña
echo "Iniciando componente chocachoca..."
create_tab "chocachoca" "cd ~/robocomp/components/Robotica_g7_25-26/Prueba && bin/localiser etc/config"

sleep 2

#Iniciar aspirador en sexta pestaña
echo "Iniciando aspirador..."
create_tab "Aspirador" "cd ~/robocomp/components/beta-robotica-class/aspirador && bin/aspirador etc/config"

# Iniciar joystick en séptima pestaña
echo "Iniciando joystick..."
create_tab "Joystick" "cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish && bin/JoystickPublish etc/config_shadow"

echo "Todos los componentes han sido lanzados en pestañas separadas de la misma terminal."
echo "Puedes navegar entre pestañas con Ctrl+PageUp/PageDown o Ctrl+Tab/Ctrl+Shift+Tab"

