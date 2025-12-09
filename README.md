# 🤖 Franka Panda Pick-and-Place Autonomo (ROS/MoveIt! - Simulazione Gazebo)

Questo progetto implementa un sistema robusto di **Pick-and-Place** per il robot a 7 gradi di libertà **Franka Emika Panda** in ambiente simulato (Gazebo). Il sistema utilizza la visione artificiale per localizzare oggetti colorati o marcatori ArUco e la libreria **MoveIt!** per la pianificazione e l'esecuzione di traiettorie collision-free.

L'obiettivo è dimostrare competenze avanzate in **integrazione software robotica**, **motion planning** e **visione artificiale 3D**.

## ✨ Caratteristiche Tecniche

* **Robot & Piattaforma:** Franka Emika Panda (7 DoF) su ROS Melodic/Noetic.
* **Motion Planning:** **MoveIt!** per pianificazione cinematica inversa e traiettorie collision-free.
* **Architettura:** Nodi ROS separati (Percezione, Planner, Controller, State Machine) per alta modularità.
* **Visione (Perception):**
    * Riconoscimento di oggetti tramite **Color Thresholding** (HSV/OpenCV).
    * Riconoscimento di oggetti tramite **Marcatori ArUco** (libreria OpenCV).
    * **Localizzazione 3D:** Calcolo della posa (XYZ + Orientamento) tramite **Ray-Casting** e **TF (ROS Transformations)**.
* **Controller:** Gestione diretta del gruppo `panda_arm` e `panda_hand` (gripper).
* **Configurazione:** Tutti i parametri critici (offset, pose target, range HSV) sono gestiti tramite file **YAML** esterni per una facile calibrazione.

## 📐 Architettura del Sistema (ROS Nodes)

La logica è orchestrata da una **State Machine** che coordina i seguenti nodi:


| Nodo | Scopo | Interfacce Principali | File Corrispondente |
| :--- | :--- | :--- | :--- |
| `perception_colore` / `perception_aruco` | Rilevazione e localizzazione 3D del target. | **Pubblica:** `/cube_pose_stamped` (`PoseStamped`) | `colore.py`, `marker.py` |
| `controller_planner` | Funzioni a basso livello di movimento (MoveIt!) e controllo del gripper. | **Espone:** Funzioni di movimento (interne o come Service/Action). | `movimento.py` (Rinominato) |
| `state_machine` | Gestione sequenziale dell'operazione Pick-and-Place. | **Logica:** Chiama le funzioni di movimento del planner quando riceve un nuovo target dal topic di percezione. | `state_machine.py` (NUOVO) |

## 🚀 Esecuzione del Progetto (Simulazione)

### Prerequisiti

Assicurati di avere un'installazione funzionante di **ROS** (es. Noetic/Melodic), **Gazebo** e la configurazione **MoveIt!** per la Franka Panda (es. `panda_moveit_config`).

### 1. Installazione e Compilazione

```bash
# Clona il workspace
git clone [https://github.com/tuo_nome_utente/franka_pick_and_place_ws.git](https://github.com/tuo_nome_utente/franka_pick_and_place_ws.git)
cd franka_pick_and_place_ws/src
git submodule update --init --recursive # Se hai sottomoduli (es. librerie Franka)
cd ..

# Installazione delle dipendenze del pacchetto 'challenge'
rosdep install --from-paths src --ignore-src -r -y 

# Compilazione
catkin build 
source devel/setup.bash