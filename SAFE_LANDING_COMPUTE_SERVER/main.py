import safe_zone
import compute_yolo
import compute_track_and_pid
import subprocess

def init():
    # Запуск внешнего скрипта без ожидания его завершения
    subprocess.Popen(
        ["python3", "/home/matvey/ONYX/SAFE_LANDING_RPI/FLIGHT/flight.py"]
    )
    
    if not compute_yolo.compute_yolo_step():
        return False
    
    landing_point = safe_zone.recognize_landing_point()
    if landing_point is None:
        print("Безопасная зона для посадки не найдена.")
        return False
    
    if not compute_track_and_pid.initialize_tracker(landing_point):
        print("Ошибка инициализации трекера")
        return False
    
    return True

def main_loop():
    if not init():
        return

    while True:
        if not compute_yolo.compute_yolo_step():
            break
        if not compute_track_and_pid.track_point_step():
            break

if __name__ == "__main__":
    main_loop()

