import os
import time

# Configuration
NODE_NAME = "/cartesian_velocity_node"
PARAM_NAME = "target_velocity"
SPEED = 0.05  # 5 cm/s
DURATION = 2.0  # 2 secondes par côté

def set_velocity(vx, vy, vz):
    # On construit la commande ROS 2 pour changer le paramètre
    # Format: [vx, vy, vz, wx, wy, wz]
    cmd = f"ros2 param set {NODE_NAME} {PARAM_NAME} \"[{vx}, {vy}, {vz}, 0.0, 0.0, 0.0]\""
    print(f"Executing: {vx}, {vy}, {vz}")
    os.system(cmd)

def main():
    print(" DÉMARRAGE DU CARRÉ dans 2 secondes...")
    time.sleep(2)

    try:
        # Côté 1 : Avancer en X
        set_velocity(SPEED, 0.0, 0.0)
        time.sleep(DURATION)

        # Côté 2 : Aller à gauche en Y
        set_velocity(0.0, SPEED, 0.0)
        time.sleep(DURATION)

        # Côté 3 : Reculer en X
        set_velocity(-SPEED, 0.0, 0.0)
        time.sleep(DURATION)

        # Côté 4 : Revenir en Y
        set_velocity(0.0, -SPEED, 0.0)
        time.sleep(DURATION)

    except KeyboardInterrupt:
        print("\nArrêt d'urgence détecté !")

    finally:
        # ÉTAPE CRUCIALE : On arrête le robot quoi qu'il arrive
        print("STOP FINAL")
        set_velocity(0.0, 0.0, 0.0)

if __name__ == "__main__":
    main()